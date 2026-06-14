use std::collections::{HashMap, HashSet};
use std::sync::{Arc, Mutex, RwLock};

use bevy::math::IVec3;
use bevy::ecs::resource::Resource;
use crossbeam_channel::{unbounded, Receiver, Sender};

use voxel_data::grid::GridId;
use voxel_data::voxels::Voxels;
use voxel_streaming::LodLoadRequest;

use crate::handle::{SourceEvent, SourceLodResult, SourceResult};
use crate::source::{ChunkSource, GridKey, SourceId, VoxelLodGenerator};

/// [`ChunkSource`] takes `&self` and synchronizes internally.
pub(crate) type SharedSource = Arc<dyn ChunkSource>;

/// Reverse of `keys`, shared with the serve worker threads.
pub(crate) type GridKeyMap = Arc<RwLock<HashMap<GridId, GridKey>>>;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub(crate) struct LodRequestKey {
	pub grid: GridKey,
	pub min: IVec3,
	pub size: IVec3,
	pub lod_bits: u32,
}

impl LodRequestKey {
	pub fn new(grid: GridKey, min: IVec3, size: IVec3, lod: f32) -> Self {
		Self { grid, min, size, lod_bits: lod.to_bits() }
	}
}

pub(crate) enum PendingLodJob {
	Direct {
		requests: Vec<LodLoadRequest>,
	},
	Composite {
		requests: Vec<LodLoadRequest>,
		expected: HashSet<SourceId>,
		received: HashMap<SourceId, Option<Voxels>>,
		final_lod: f32,
		intermediate_lod: f32,
	},
}

/// LOD requests awaiting results, shared with the serve worker threads.
pub(crate) type PendingLod = Arc<Mutex<HashMap<LodRequestKey, PendingLodJob>>>;

#[derive(Resource)]
pub struct SourceRegistry {
	pub(crate) sources: Vec<SharedSource>,
	pub(crate) lod_generator: Arc<dyn VoxelLodGenerator>,
	pub(crate) keys: HashMap<GridKey, GridId>,
	pub(crate) grid_keys: GridKeyMap,
	pub(crate) event_tx: Sender<SourceEvent>,
	pub(crate) event_rx: Receiver<SourceEvent>,
	pub(crate) result_tx: Sender<SourceResult>,
	pub(crate) result_rx: Receiver<SourceResult>,
	pub(crate) lod_result_tx: Sender<SourceLodResult>,
	pub(crate) lod_result_rx: Receiver<SourceLodResult>,
	pub(crate) pending_lod: PendingLod,
}

impl Default for SourceRegistry {
	fn default() -> Self {
		let (event_tx, event_rx) = unbounded();
		let (result_tx, result_rx) = unbounded();
		let (lod_result_tx, lod_result_rx) = unbounded();
		Self {
			sources: Vec::new(),
			lod_generator: Arc::new(IdentityVoxelLodGenerator),
			keys: HashMap::new(),
			grid_keys: Arc::new(RwLock::new(HashMap::new())),
			event_tx,
			event_rx,
			result_tx,
			result_rx,
			lod_result_tx,
			lod_result_rx,
			pending_lod: Arc::new(Mutex::new(HashMap::new())),
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

	pub(crate) fn entity(&self, grid: GridKey) -> Option<GridId> {
		self.keys.get(&grid).copied()
	}

	pub(crate) fn forget_others(&self, keep: SourceId, grid: GridKey, chunk: IVec3) {
		for (i, source) in self.sources.iter().enumerate() {
			if i != keep.0 {
				source.forget(grid, chunk);
			}
		}
	}
}

pub(crate) fn cheapest(sources: &[SharedSource], grid: GridKey, chunk: IVec3) -> Option<SourceId> {
	sources
		.iter()
		.enumerate()
		.filter_map(|(i, s)| s.cost(grid, chunk).map(|c| (c, SourceId(i))))
		.min_by_key(|(c, _)| *c)
		.map(|(_, id)| id)
}

pub(crate) fn lod_sources_with_any_chunks(sources: &[SharedSource], grid: GridKey, min: IVec3, size: IVec3) -> Vec<SourceId> {
	sources
		.iter()
		.enumerate()
		.filter_map(|(i, source)| source.has_any_chunks_in_area(grid, min, size).then_some(SourceId(i)))
		.collect()
}

struct IdentityVoxelLodGenerator;

impl VoxelLodGenerator for IdentityVoxelLodGenerator {
	fn generate(&self, voxels: &Voxels, _lod: f32) -> Option<Voxels> {
		(!voxels.is_empty()).then(|| voxels.clone())
	}
}

