use bevy::{log::tracing::Instrument, prelude::*, tasks::AsyncComputeTaskPool};
use rustc_hash::{FxHashMap, FxHashSet};
use tile_data::{CHUNK_SIZE, NonZeroChunkRegion, TileBuilderRegistry, TileBuildingParameters, TileKey};
use voxel_data::grid::GridId;
use bevy::ecs::system::SystemParam;
use voxel_sources::edit::{GridGeneration, GridEditIdManager};
use voxel_tasks::CancellationToken;

use crate::{GridStreaming, streaming::{TileState, TileStatus}, tile_building::{TileBuildingCancellationToken, TileBuildingChannel, TileBuildingResult, TileVoxelSourceBridge, session}};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TileLoadStatus {
	Ready(Entity),
	Empty,
}

#[derive(Clone, Copy, Debug)]
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
	grids: Query<'w, 's, (&'static mut GridStreaming, &'static GridEditIdManager)>,
}

fn valid_tile_key(key: TileKey) -> bool {
	let factor = 1u32 << key.lod;
	let coarse_extent = (key.size() * CHUNK_SIZE as u32) / factor;
	!coarse_extent.cmplt(UVec3::ONE).any() && !coarse_extent.cmpgt(UVec3::splat(CHUNK_SIZE as u32)).any()
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
		let streaming = &mut *streaming;
		streaming.tile_dependencies.remove(tile_key);
		let mut tiles = std::mem::take(&mut streaming.tiles);
		let state = tiles.entry(tile_key).or_insert_with(|| {
			streaming.retain_edit_interest_region(tile_key.region.into());
			TileState {
				requesters: FxHashMap::default(),
				status: TileStatus::InFlight {
					generation: edit_id_manager.latest_generation(),
					cancellation: Self::spawn(&self.bridge, &self.builders, &self.results, grid, tile_key, edit_id_manager.latest_generation(), context),
				},
				entity: None,
			}
		});
		state.requesters.insert(requester, priority);
		streaming.tiles = tiles;
		true
	}

	pub(crate) fn dirty_stale_tiles(&mut self, grid: GridId, regions: impl IntoIterator<Item = NonZeroChunkRegion>, context: Option<&TileBuildingParameters>) {
		let Ok((mut streaming, edit_id_manager)) = self.grids.get_mut(grid) else { return };
		let tile_keys: FxHashSet<TileKey> = regions.into_iter().map(|region| streaming.tile_dependencies.tiles_using_region(region)).flatten().collect();
		for tile_key in tile_keys {
			let Some(state) = streaming.tiles.get_mut(&tile_key) else { return };
			Self::dirty_tile_inner(
				&self.bridge,
				&self.builders,
				&self.results,
				state,
				&edit_id_manager,
				grid,
				tile_key,
				context
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
				&edit_id_manager,
				grid,
				*tile_key,
				context
			);
		}
	}

	pub(crate) fn dirty_tile(&mut self, grid: GridId, context: Option<&TileBuildingParameters>, tile_key: TileKey) {
		let Ok((mut streaming, edit_id_manager)) = self.grids.get_mut(grid) else { return };
		let streaming = &mut *streaming;
		let Some(state) = streaming.tiles.get_mut(&tile_key) else { return };
		streaming.tile_dependencies.remove(tile_key);
		Self::dirty_tile_inner(
			&self.bridge,
			&self.builders,
			&self.results,
			state,
			&edit_id_manager,
			grid,
			tile_key,
			context
		);
	}

	fn dirty_tile_inner(
		bridge: &TileVoxelSourceBridge,
		builders: &TileBuilderRegistry,
		results: &TileBuildingChannel,
		state: &mut TileState,
		edit_id_manager: &GridEditIdManager,
		grid: GridId,
		tile_key: TileKey,
		context: Option<&TileBuildingParameters>
	) {
		assert!(!state.requesters.is_empty());
		let old_status = std::mem::replace(&mut state.status, TileStatus::InFlight {
			generation: edit_id_manager.latest_generation(),
			cancellation: Self::spawn(bridge, builders, results, grid, tile_key, edit_id_manager.latest_generation(), context)
		});
		if let TileStatus::InFlight { generation: _, cancellation } = old_status {
			cancellation.cancel();
		}
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
		let context = context.map_or_default(|context| context.clone());
		let builder = builders.builder(tile_key.class);
		let cancellation = CancellationToken::new();
		let (session, cancellation_token) = session(grid, tile_key, context.clone(), bridge.sender(), cancellation.clone());
		let result_tx = results.sender();
		let task_cancellation = cancellation.clone();
		AsyncComputeTaskPool::get().spawn(async move {
			let data = builder.build(session).await;
			if task_cancellation.is_cancelled() { return; }
			let _ = result_tx.send(TileBuildingResult { grid, tile_key, generation, context, data });
		}.instrument(bevy::log::info_span!("build tile"))).detach();
		cancellation_token
	}
}
