use bevy::{ecs::{message::Message, system::SystemParam}, log::tracing::Instrument, prelude::*, tasks::AsyncComputeTaskPool};
use rustc_hash::{FxHashMap, FxHashSet};
use tile_data::{CHUNK_SIZE, NonZeroChunkRegion, TileBuilderRegistry, TileBuildingParameters, TileKey};
use voxel_data::grid::GridId;
use voxel_sources::edit::{GridEditIdManager, GridGeneration};
use voxel_tasks::CancellationToken;

use crate::{GridStreaming, streaming::{TileState, TileStatus}, tile_building::{TileBuildingCancellationToken, TileBuildingChannel, TileBuildingResult, TileVoxelSourceBridge, session}};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TileLoadStatus {
	Ready(Entity),
	Empty,
}

#[derive(Message, Clone, Copy, Debug, PartialEq, Eq)]
pub struct TileLoadUpdate {
	pub grid: GridId,
	pub requester: Entity,
	pub key: TileKey,
	pub status: TileLoadStatus,
}

#[derive(SystemParam)]
pub struct TileRequester<'w, 's> {
	bridge: Res<'w, TileVoxelSourceBridge>,
	builders: Res<'w, TileBuilderRegistry>,
	results: Res<'w, TileBuildingChannel>,
	tile_load_updates: MessageWriter<'w, TileLoadUpdate>,
	grids: Query<'w, 's, (&'static mut GridStreaming, &'static GridEditIdManager)>,
	commands: Commands<'w, 's>,
}

impl<'w, 's> TileRequester<'w, 's> {
	pub fn fetch_tile(
		&mut self,
		grid: GridId,
		requester: Entity,
		tile_key: TileKey,
		priority: f32,
		context: Option<&TileBuildingParameters>,
	) -> bool {
		if !valid_tile_key(tile_key) { return false; }
		let Ok((mut streaming, edit_id_manager)) = self.grids.get_mut(grid) else { return false; };
		if let Some(state) = streaming.tiles.get_mut(&tile_key) {
			let joined = state.requesters.insert(requester, priority).is_none();
			if joined && matches!(&state.status, TileStatus::Loaded) {
				let status = state.entity.map_or(TileLoadStatus::Empty, TileLoadStatus::Ready);
				self.tile_load_updates.write(TileLoadUpdate { grid, requester, key: tile_key, status });
			}
			return true;
		}

		let generation = edit_id_manager.latest_generation();
		let cancellation = Self::spawn(&self.bridge, &self.builders, &self.results, grid, tile_key, generation, context);
		streaming.retain_edit_interest_region(tile_key.region);
		streaming.tiles.insert(tile_key, TileState {
			requesters: FxHashMap::from_iter([(requester, priority)]),
			status: TileStatus::InFlight { generation, cancellation },
			entity: None,
		});
		true
	}

	pub(crate) fn dirty_stale_tiles(
		&mut self,
		grid: GridId,
		regions: impl IntoIterator<Item = NonZeroChunkRegion>,
		context: Option<&TileBuildingParameters>,
	) {
		let Ok((mut streaming, edit_id_manager)) = self.grids.get_mut(grid) else { return };
		let tile_keys: FxHashSet<TileKey> = regions.into_iter().flat_map(|region| streaming.tile_dependencies.tiles_using_region(region)).collect();
		for tile_key in tile_keys {
			let Some(state) = streaming.tiles.get_mut(&tile_key) else { continue };
			Self::dirty_tile_inner(
				&self.bridge,
				&self.builders,
				&self.results,
				state,
				edit_id_manager,
				grid,
				tile_key,
				context,
			);
		}
	}

	pub(crate) fn invalidate_building_context(&mut self, grid: GridId, context: Option<&TileBuildingParameters>) {
		let Ok((mut streaming, edit_id_manager)) = self.grids.get_mut(grid) else { return };
		let streaming = &mut *streaming;
		for (tile_key, state) in &mut streaming.tiles {
			streaming.tile_dependencies.remove(*tile_key);
			Self::dirty_tile_inner(
				&self.bridge,
				&self.builders,
				&self.results,
				state,
				edit_id_manager,
				grid,
				*tile_key,
				context,
			);
		}
	}

	fn dirty_tile_inner(
		bridge: &TileVoxelSourceBridge,
		builders: &TileBuilderRegistry,
		results: &TileBuildingChannel,
		state: &mut TileState,
		edit_id_manager: &GridEditIdManager,
		grid: GridId,
		tile_key: TileKey,
		context: Option<&TileBuildingParameters>,
	) {
		assert!(!state.requesters.is_empty());
		let generation = edit_id_manager.latest_generation();
		let old_status = std::mem::replace(&mut state.status, TileStatus::InFlight {
			generation,
			cancellation: Self::spawn(bridge, builders, results, grid, tile_key, generation, context),
		});
		if let TileStatus::InFlight { cancellation, .. } = old_status { cancellation.cancel(); }
	}

	fn spawn(
		bridge: &TileVoxelSourceBridge,
		builders: &TileBuilderRegistry,
		results: &TileBuildingChannel,
		grid: GridId,
		tile_key: TileKey,
		generation: GridGeneration,
		context: Option<&TileBuildingParameters>,
	) -> TileBuildingCancellationToken {
		let context = context.map_or_default(Clone::clone);
		let builder = builders.builder(tile_key.class);
		let cancellation = CancellationToken::new();
		let (session, cancellation_token) = session(grid, tile_key, generation, context.clone(), bridge.sender(), cancellation.clone());
		let result_tx = results.sender();
		let task_cancellation = cancellation.clone();
		AsyncComputeTaskPool::get().spawn(async move {
			let data = builder.build(session).await;
			if task_cancellation.is_cancelled() { return; }
			let _ = result_tx.send(TileBuildingResult { grid, tile_key, generation, context, data });
		}.instrument(bevy::log::info_span!("build tile"))).detach();
		cancellation_token
	}

	pub fn release_tile(&mut self, grid: GridId, requester: Entity, tile_key: TileKey) {
		let Ok((mut streaming, _)) = self.grids.get_mut(grid) else { return };
		let Some(state) = streaming.tiles.get_mut(&tile_key) else { return };
		if state.requesters.remove(&requester).is_none() || !state.requesters.is_empty() { return; }

		let state = streaming.tiles.remove(&tile_key).unwrap();
		streaming.tile_dependencies.remove(tile_key);
		streaming.release_edit_interest_region(tile_key.region);
		if let TileStatus::InFlight { cancellation, .. } = &state.status { cancellation.cancel(); }
		if let Some(entity) = state.entity { self.commands.entity(entity).despawn(); }
	}
}

#[derive(SystemParam)]
pub struct TileReleaser<'w, 's> {
	grids: Query<'w, 's, (&'static mut GridStreaming, &'static GridEditIdManager)>,
	commands: Commands<'w, 's>,
}

impl<'w, 's> TileReleaser<'w, 's> {
	pub fn release_tile(&mut self, grid: GridId, requester: Entity, tile_key: TileKey) {
		let Ok((mut streaming, _)) = self.grids.get_mut(grid) else { return };
		let Some(state) = streaming.tiles.get_mut(&tile_key) else { return };
		if state.requesters.remove(&requester).is_none() || !state.requesters.is_empty() { return; }

		let state = streaming.tiles.remove(&tile_key).unwrap();
		streaming.tile_dependencies.remove(tile_key);
		streaming.release_edit_interest_region(tile_key.region);
		if let TileStatus::InFlight { cancellation, .. } = &state.status { cancellation.cancel(); }
		if let Some(entity) = state.entity { self.commands.entity(entity).despawn(); }
	}
}

fn valid_tile_key(key: TileKey) -> bool {
	let factor = 1u32 << key.lod;
	let coarse_extent = (key.size() * CHUNK_SIZE as u32) / factor;
	!coarse_extent.cmplt(UVec3::ONE).any() && !coarse_extent.cmpgt(UVec3::splat(CHUNK_SIZE as u32)).any()
}