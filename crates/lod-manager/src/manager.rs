use std::collections::{HashMap, HashSet};

use bevy::prelude::*;
use gpu_voxel_data::LodVoxels;
use voxel_streaming::{ChunkConsumer, GridStreaming, LodRequestChannel, VoxelStreamingAppExt};

use crate::loaded::{LoadedLods, LodRetainCount};
use crate::request_tree::LoadedLodEvent;
use crate::{LodKey, LodRequestMap};

voxel_streaming::chunk_consumer!(pub LodManagerConsumer);

#[derive(Resource, Default)]
struct RequestedLods(HashSet<LodKey>);

#[derive(Resource, Default)]
struct LodRequestOwners {
	by_key: HashMap<LodKey, HashMap<Entity, f32>>,
	wants_gpu: HashMap<LodKey, HashSet<Entity>>,
}

impl LodRequestOwners {
	fn priority(&self, key: &LodKey) -> Option<f32> {
		self.by_key.get(key).map(|owners| owners.values().copied().fold(f32::NEG_INFINITY, f32::max))
	}

	fn is_requested(&self, key: &LodKey) -> bool {
		self.by_key.contains_key(key)
	}

	fn wants_gpu(&self, key: &LodKey) -> bool {
		self.wants_gpu.get(key).is_some_and(|owners| !owners.is_empty())
	}

	fn gpu_owners(&self, key: &LodKey) -> impl Iterator<Item = Entity> + '_ {
		self.wants_gpu.get(key).into_iter().flatten().copied()
	}
}

#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum LodManagerSet {
	Collect,
	Request,
	Receive,
	Retire,
}

#[derive(Default)]
pub struct LodManagerPlugin;

impl Plugin for LodManagerPlugin {
	fn build(&self, app: &mut App) {
		app.init_resource::<LoadedLods>()
			.init_resource::<RequestedLods>()
			.init_resource::<LodRequestOwners>()
			.register_chunk_consumer::<LodManagerConsumer>()
			.configure_sets(
				Update,
				(
					LodManagerSet::Collect,
					LodManagerSet::Request,
					LodManagerSet::Receive,
					LodManagerSet::Retire,
				)
					.chain(),
			)
			.add_systems(Startup, spawn_lod_manager_consumer)
			.add_systems(Update, collect_lod_requests.in_set(LodManagerSet::Collect))
			.add_systems(Update, request_missing_lods.in_set(LodManagerSet::Request))
			.add_systems(Update, receive_loaded_lods.in_set(LodManagerSet::Receive))
			.add_systems(Update, retire_unused_lods.in_set(LodManagerSet::Retire));
	}
}

fn spawn_lod_manager_consumer(mut commands: Commands) {
	commands.spawn(LodManagerConsumer::default());
}

fn collect_lod_requests(
	mut owners: ResMut<LodRequestOwners>,
	mut requested: ResMut<RequestedLods>,
	mut maps: Query<(Entity, &mut LodRequestMap)>,
) {
	owners.by_key.clear();
	owners.wants_gpu.clear();
	for (owner, mut map) in maps.iter_mut() {
		// Deltas are consumed by this central manager. Current state is rebuilt below,
		// making the system robust if a policy skipped emitting a delta.
		let _ = map.drain_added_delta();
		let _ = map.drain_removed_delta();

		for request in map.iter() {
			owners.by_key.entry(request.key).or_default().insert(owner, request.priority);
			if request.destination.wants_gpu() {
				owners.wants_gpu.entry(request.key).or_default().insert(owner);
			}
		}
	}
	requested.0.retain(|key| owners.is_requested(key));
}

fn request_missing_lods(
	consumer: Option<Single<Entity, With<LodManagerConsumer>>>,
	channel: Res<LodRequestChannel>,
	owners: Res<LodRequestOwners>,
	loaded: Res<LoadedLods>,
	mut requested: ResMut<RequestedLods>,
	grids: Query<&GridStreaming>,
) {
	let Some(consumer) = consumer else { return };
	let consumer = *consumer;

	for key in owners.by_key.keys().copied() {
		if !owners.wants_gpu(&key) || loaded.contains(&key) || requested.0.contains(&key) { continue; }
		let Some(priority) = owners.priority(&key) else { continue };
		let Ok(streaming) = grids.get(key.grid) else { continue };
		if streaming.fetch_lod(key.grid, consumer, &channel, key.min, key.size, key.lod(), priority) {
			requested.0.insert(key);
		}
	}
}

fn receive_loaded_lods(
	mut commands: Commands,
	consumer: Option<Single<&mut LodManagerConsumer>>,
	owners: Res<LodRequestOwners>,
	mut loaded: ResMut<LoadedLods>,
	mut requested: ResMut<RequestedLods>,
	mut maps: Query<&mut LodRequestMap>,
) {
	let Some(mut consumer) = consumer else { return };

	for result in consumer.drain_lod() {
		let key = LodKey::new(result.grid, result.min, result.size, result.lod);
		requested.0.remove(&key);
		if !owners.wants_gpu(&key) { continue; }
		let Some(voxels) = result.voxels else { continue; };

		if let Some(previous) = loaded.remove(&key) {
			commands.entity(previous).despawn();
		}
		let entity = commands
			.spawn((
				LodVoxels {
					voxels,
					grid: result.grid,
					min: result.min,
					size: result.size,
					lod: result.lod,
					priority: result.priority,
				},
				LodRetainCount::default(),
			))
			.id();
		loaded.insert(key, entity);

		for owner in owners.gpu_owners(&key) {
			if let Ok(mut map) = maps.get_mut(owner) {
				map.push_loaded(LoadedLodEvent { key, entity });
			}
		}
	}
}

fn retire_unused_lods(
	mut commands: Commands,
	owners: Res<LodRequestOwners>,
	mut loaded: ResMut<LoadedLods>,
	retains: Query<&LodRetainCount>,
) {
	let retiring: Vec<_> = loaded
		.keys()
		.copied()
		.filter(|key| !owners.wants_gpu(key))
		.filter(|key| {
			loaded
				.get(key)
				.and_then(|entity| retains.get(entity).ok())
				.is_none_or(|retain| !retain.is_retained())
		})
		.collect();

	for key in retiring {
		if let Some(entity) = loaded.remove(&key) {
			commands.entity(entity).despawn();
		}
	}
}
