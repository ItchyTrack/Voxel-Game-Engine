use std::collections::{HashMap, HashSet};
use std::sync::{Arc, Mutex, RwLock};

use bevy::ecs::resource::Resource;
use bevy::math::IVec3;
use crossbeam_channel::{unbounded, Receiver, Sender};

use voxel_data::grid::GridId;
use voxel_data::{grid_tree::GridRegion, voxels::{VoxelTypeInfo, Voxels}};
use voxel_tasks::CancellationToken;

use crate::loader::{ChunkLoadRequest, GeneratedLodLoadRequest, LodCancellation, LodLoadRequest, PresenceLoadRequest, SourceRequestChannel};

use crate::handle::{SourceLodResult, SourceMessage};
use crate::source::{ChunkSource, SourceId, VoxelLodGenerator, VoxelLodGenerators};

pub(crate) type SharedSource = Arc<dyn ChunkSource>;

const CHUNK_SIZE: i32 = 64;

type ChunkRequestKey = (GridId, IVec3);

struct ActiveChunkLoad {
	generation: u64,
	cancellation: CancellationToken,
}

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
		generation: u64,
		cancellation: CancellationToken,
	},
	Composite {
		requests: Vec<GeneratedLodLoadRequest>,
		expected: HashSet<SourceId>,
		received: HashMap<SourceId, Option<Voxels>>,
		final_lod: f32,
		intermediate_lod: f32,
		generation: u64,
		cancellation: CancellationToken,
	},
}

/// LOD requests awaiting results, shared with the serve worker threads.
pub(crate) type PendingLod = Arc<Mutex<HashMap<LodRequestKey, PendingLodJob>>>;
pub(crate) type ActivePresenceLoads = Arc<Mutex<HashMap<GridId, u32>>>;

#[derive(Resource)]
pub(crate) struct SourceRegistry {
	pub(crate) sources: Vec<SharedSource>,
	pub(crate) generators: VoxelLodGenerators,

	pub(crate) message_tx: Sender<SourceMessage>,
	pub(crate) message_rx: Receiver<SourceMessage>,
	pub(crate) pending_lod: PendingLod,
	pub(crate) active_presence_loads: ActivePresenceLoads,
	active_chunk_loads: Mutex<HashMap<ChunkRequestKey, ActiveChunkLoad>>,
	pub(crate) generations: Arc<Mutex<HashMap<GridId, u64>>>,
	pub(crate) requests: SourceRequestChannel,
}

impl Default for SourceRegistry {
	fn default() -> Self {
		let (message_tx, message_rx) = unbounded();
		Self {
			sources: Vec::new(),
			generators: Arc::new(RwLock::new(HashMap::new())),
			message_tx,
			message_rx,
			pending_lod: Arc::new(Mutex::new(HashMap::new())),
			active_presence_loads: Arc::new(Mutex::new(HashMap::new())),
			active_chunk_loads: Mutex::new(HashMap::new()),
			generations: Arc::new(Mutex::new(HashMap::new())),
			requests: SourceRequestChannel::default(),
		}
	}
}

impl SourceRegistry {
	pub(crate) fn push(&mut self, source: SharedSource) {
		self.sources.push(source);
	}

	pub(crate) fn register_lod_generator(&mut self, generator: Arc<dyn VoxelLodGenerator>) {
		self.generators.write().unwrap().insert(generator.input_type_id(), generator);
	}

	pub fn try_recv_message(&self) -> Option<SourceMessage> {
		self.message_rx.try_recv().ok()
	}

	pub fn request_presence(&self, request: PresenceLoadRequest) {
		self.requests.request_presence(request);
	}

	pub fn request_chunk(&self, request: ChunkLoadRequest) -> CancellationToken {
		let generation = self.next_generation(request.grid);
		let cancellation = CancellationToken::new();
		let key = (request.grid, request.chunk);
		if let Some(previous) = self.active_chunk_loads.lock().unwrap().insert(key, ActiveChunkLoad {
			generation,
			cancellation: cancellation.clone(),
		}) {
			previous.cancellation.cancel();
		}
		self.requests.request_chunk(request, generation, cancellation.clone());
		cancellation
	}

	pub(crate) fn complete_chunk(&self, grid: GridId, chunk: IVec3, generation: u64) -> bool {
		let mut active = self.active_chunk_loads.lock().unwrap();
		let key = (grid, chunk);
		if !active.get(&key).is_some_and(|load| load.generation == generation && !load.cancellation.is_cancelled()) {
			return false;
		}
		active.remove(&key);
		true
	}

	pub fn request_lod(&self, request: LodLoadRequest) -> LodCancellation {
		let generation = self.next_generation(request.grid);
		let cancellation = CancellationToken::new();
		self.requests.request_lod(request, generation, cancellation)
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
			PendingLodJob::Direct { requests, generation, .. } => {
				if result.generation != *generation { return None; }
				let mut requests = std::mem::take(requests);
				requests.retain(|request| !request.cancellation.is_cancelled());
				pending_lod.remove(&key);
				Some((requests, result.lod, result.voxels))
			}
			PendingLodJob::Composite { requests, expected, received, final_lod, intermediate_lod, generation, .. } => {
				if result.generation != *generation { return None; }
				received.insert(result.source, result.voxels);
				if !expected.iter().all(|source| received.contains_key(source)) {
					None
				} else {
					let mut requests = std::mem::take(requests);
					requests.retain(|request| !request.cancellation.is_cancelled());
					let final_lod = *final_lod;
					let intermediate_lod = *intermediate_lod;
					let parts: Vec<_> = received.values().filter_map(Clone::clone).collect();
					pending_lod.remove(&key);
					let voxels = parts.first().map(|first| first.voxel_type_info()).map(|voxel_type_info| merge_voxels(voxel_type_info, parts));
					let voxels = if final_lod <= intermediate_lod {
						voxels.filter(|voxels| !voxels.is_empty())
					} else {
						self.downsample_if_needed(voxels, intermediate_lod, final_lod, key.size)
					};
					Some((requests, final_lod, voxels))
				}
			}
		}
	}

	fn downsample_if_needed(&self, voxels: Option<Voxels>, source_lod: f32, final_lod: f32, size: IVec3) -> Option<Voxels> {
		let voxels = voxels?;
		if final_lod <= source_lod {
			return (!voxels.is_empty()).then_some(voxels);
		}
		let generator = self.generators.read().unwrap().get(&voxels.voxel_type_id()).cloned()?;
		let delta_lod = final_lod - source_lod;
		generator.generate(IVec3::ZERO, size, delta_lod, &|chunk| chunk_from_region(&voxels, chunk))
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

fn chunk_from_region(voxels: &Voxels, chunk: IVec3) -> Option<Voxels> {
	let min = chunk * CHUNK_SIZE;
	let region = GridRegion::from_min_size(min, IVec3::splat(CHUNK_SIZE))?;
	let mut out = Voxels::new_with_type(voxels.voxel_type_info());
	out.merge_region_from(voxels, Some(region), -min);
	(!out.is_empty()).then_some(out)
}

fn merge_voxels(voxel_type_info: VoxelTypeInfo, parts: Vec<Voxels>) -> Voxels {
	let mut parts = parts.into_iter();
	let Some(mut merged) = parts.next() else {
		return Voxels::new_with_type(voxel_type_info);
	};
	for voxels in parts {
		merged.merge_from(&voxels, IVec3::ZERO);
	}
	merged
}

#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn cancelled_and_replaced_chunk_results_are_rejected() {
		let registry = SourceRegistry::default();
		let request = ChunkLoadRequest { grid: GridId::PLACEHOLDER, chunk: IVec3::new(1, 2, 3) };

		registry.request_chunk(request).cancel();
		assert!(!registry.complete_chunk(request.grid, request.chunk, 1));

		registry.request_chunk(request);
		registry.request_chunk(request);
		assert!(!registry.complete_chunk(request.grid, request.chunk, 2));
		assert!(registry.complete_chunk(request.grid, request.chunk, 3));
	}
}


