use std::collections::{HashMap, HashSet};
use std::sync::{Arc, Mutex};

use bevy::ecs::resource::Resource;
use bevy::math::IVec3;
use crossbeam_channel::{unbounded, Receiver, Sender};

use voxel_data::grid::GridId;
use voxel_data::voxels::Voxels;

use crate::loader::{ChunkLoadRequest, GeneratedLodLoadRequest, LodLoadRequest, PresenceLoadRequest, SourceRequestChannel};

use crate::handle::{SourceLodResult, SourceMessage};
use crate::source::{ChunkSource, SourceId, VoxelLodGenerator};

pub(crate) type SharedSource = Arc<dyn ChunkSource>;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub(crate) struct LodRequestKey {
	pub grid: GridId,
	pub min: IVec3,
	pub size: IVec3,
	pub lod_bits: u32,
}

impl LodRequestKey {
	pub fn new(grid: GridId, min: IVec3, size: IVec3, lod: f32) -> Self {
		Self { grid, min, size, lod_bits: lod.to_bits() }
	}
}

pub(crate) enum PendingLodJob {
	Direct {
		requests: Vec<GeneratedLodLoadRequest>,
	},
	Composite {
		requests: Vec<GeneratedLodLoadRequest>,
		expected: HashSet<SourceId>,
		received: HashMap<SourceId, Option<Voxels>>,
		final_lod: f32,
		intermediate_lod: f32,
	},
}

/// LOD requests awaiting results, shared with the serve worker threads.
pub(crate) type PendingLod = Arc<Mutex<HashMap<LodRequestKey, PendingLodJob>>>;

#[derive(Resource)]
pub(crate) struct SourceRegistry {
	pub(crate) sources: Vec<SharedSource>,
	pub(crate) lod_generator: Arc<dyn VoxelLodGenerator>,
	
	pub(crate) message_tx: Sender<SourceMessage>,
	pub(crate) message_rx: Receiver<SourceMessage>,
	pub(crate) pending_lod: PendingLod,
	pub(crate) active_presence_loads: Arc<Mutex<HashMap<GridId, u32>>>,
	pub(crate) generations: Arc<Mutex<HashMap<GridId, u64>>>,
	pub(crate) requests: SourceRequestChannel,
}

impl Default for SourceRegistry {
	fn default() -> Self {
		let (message_tx, message_rx) = unbounded();
		Self {
			sources: Vec::new(),
			lod_generator: Arc::new(IdentityVoxelLodGenerator),
			message_tx,
			message_rx,
			pending_lod: Arc::new(Mutex::new(HashMap::new())),
			active_presence_loads: Arc::new(Mutex::new(HashMap::new())),
			generations: Arc::new(Mutex::new(HashMap::new())),
			requests: SourceRequestChannel::default(),
		}
	}
}

impl SourceRegistry {
	pub(crate) fn push(&mut self, source: SharedSource) {
		self.sources.push(source);
	}

	pub(crate) fn set_lod_generator(&mut self, generator: Arc<dyn VoxelLodGenerator>) {
		self.lod_generator = generator;
	}

	pub fn try_recv_message(&self) -> Option<SourceMessage> {
		self.message_rx.try_recv().ok()
	}

	#[allow(unused)]
	pub fn lod_generator(&self) -> &dyn VoxelLodGenerator {
		self.lod_generator.as_ref()
	}

	pub fn request_presence(&self, request: PresenceLoadRequest) {
		self.requests.request_presence(request);
	}

	pub fn request_chunk(&self, request: ChunkLoadRequest) {
		let generation = self.next_generation(request.grid);
		self.requests.request_chunk(request, generation);
	}

	pub fn request_lod(&self, request: LodLoadRequest) {
		let generation = self.next_generation(request.grid);
		self.requests.request_lod(request, generation);
	}

	pub fn chunk_requests_sent(&self) -> u64 {
		self.requests.chunk_sent_count()
	}

	pub fn lod_requests_sent(&self) -> u64 {
		self.requests.lod_sent_count()
	}

	pub fn take_pending_lod_completion(
		&self,
		result: SourceLodResult,
	) -> Option<(Vec<GeneratedLodLoadRequest>, f32, Option<Voxels>)> {
		let key = LodRequestKey::new(result.grid, result.min, result.size, result.lod);
		let mut pending_lod = self.pending_lod.lock().unwrap();
		let job = pending_lod.get_mut(&key)?;
		match job {
			PendingLodJob::Direct { requests } => {
				let requests = std::mem::take(requests);
				pending_lod.remove(&key);
				Some((requests, result.lod, result.voxels))
			}
			PendingLodJob::Composite { requests, expected, received, final_lod, intermediate_lod } => {
				received.insert(result.source, result.voxels);
				if !expected.iter().all(|source| received.contains_key(source)) {
					None
				} else {
					let requests = std::mem::take(requests);
					let final_lod = *final_lod;
					let intermediate_lod = *intermediate_lod;
					let parts: Vec<_> = received.values().filter_map(Clone::clone).collect();
					pending_lod.remove(&key);
					let merged = merge_voxels(parts);
					let voxels = if final_lod <= intermediate_lod {
						(!merged.is_empty()).then_some(merged)
					} else {
						self.lod_generator.generate(&merged, final_lod - intermediate_lod)
					};
					Some((requests, final_lod, voxels))
				}
			}
		}
	}

	pub(crate) fn next_generation(&self, grid: GridId) -> u64 {
		let mut generations = self.generations.lock().unwrap();
		let generation = generations.entry(grid).or_default();
		*generation += 1;
		*generation
	}

	pub fn route_save(&self, grid: GridId, chunk: IVec3, voxels: &Voxels) {
		if voxels.is_empty() {
			for source in &self.sources {
				source.forget(grid, chunk);
			}
			let generation = self.next_generation(grid);
			let _ = self.message_tx.send(SourceMessage::ChunkChanged(crate::ChunkChanged {
				grid,
				min: chunk,
				size: IVec3::ONE,
				kind: crate::ChunkChangeKind::Removed { generation },
				from_save: true,
			}));
			return;
		}
		for (i, source) in self.sources.iter().enumerate() {
			if source.can_save() {
				source.save(grid, chunk, voxels);
				self.forget_others(SourceId(i), grid, chunk, IVec3::ONE);
				let generation = self.next_generation(grid);
				let _ = self.message_tx.send(SourceMessage::ChunkChanged(crate::ChunkChanged {
					grid,
					min: chunk,
					size: IVec3::ONE,
					kind: crate::ChunkChangeKind::Changed { generation },
					from_save: true,
				}));
				break;
			}
		}
	}

	pub(crate) fn finish_presence_load(&self, grid: GridId) -> bool {
		let mut loads = self.active_presence_loads.lock().unwrap();
		let Some(count) = loads.get_mut(&grid) else { return false };
		*count = count.saturating_sub(1);
		if *count == 0 {
			loads.remove(&grid);
			true
		} else {
			false
		}
	}

	pub fn forget_others(&self, keep: SourceId, grid: GridId, min: IVec3, size: IVec3) {
		for (i, source) in self.sources.iter().enumerate() {
			if i != keep.0 {
				for x in min.x..min.x + size.x {
					for y in min.y..min.y + size.y {
						for z in min.z..min.z + size.z {
							source.forget(grid, IVec3::new(x, y, z));
						}
					}
				}
			}
		}
	}
}

pub(crate) fn cheapest(sources: &[SharedSource], grid: GridId, chunk: IVec3) -> Option<SourceId> {
	sources
		.iter()
		.enumerate()
		.filter_map(|(i, s)| s.cost(grid, chunk).map(|c| (c, SourceId(i))))
		.min_by_key(|(c, _)| *c)
		.map(|(_, id)| id)
}

pub(crate) fn lod_sources_with_any_chunks(sources: &[SharedSource], grid: GridId, min: IVec3, size: IVec3, lod: f32) -> Vec<SourceId> {
	sources
		.iter()
		.enumerate()
		.filter_map(|(i, source)| source.cost_lod(grid, min, size, lod).is_some().then_some(SourceId(i)))
		.collect()
}

fn merge_voxels(parts: Vec<Voxels>) -> Voxels {
	let mut merged = Voxels::new();
	for voxels in parts {
		let areas: Vec<_> = voxels
			.grid_tree()
			.iter()
			.map(|(pos, size, id)| (pos, bevy::math::U16Vec3::splat(size), id))
			.collect();
		merged.add_palette_areas(&areas, voxels.palette());
	}
	merged
}

struct IdentityVoxelLodGenerator;

impl VoxelLodGenerator for IdentityVoxelLodGenerator {
	fn generate(&self, voxels: &Voxels, _lod: f32) -> Option<Voxels> {
		(!voxels.is_empty()).then(|| voxels.clone())
	}
}

