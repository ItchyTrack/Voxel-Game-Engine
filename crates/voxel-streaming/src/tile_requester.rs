use std::sync::{Arc, Mutex};

use bevy::{log::tracing::Instrument, prelude::*, tasks::AsyncComputeTaskPool};
use rustc_hash::{FxHashMap, FxHashSet};
use tile_data::{NonZeroChunkRegion, TileBuilderRegistry, TileBuildingParameters, TileKey};
use voxel_data::grid::GridId;
use bevy::ecs::system::SystemParam;
use voxel_tasks::CancellationToken;

use crate::{GridStreaming, streaming::{TileState, TileStatus}, tile_building::{TileBuildingChannel, TileBuildingMetadata, TileBuildingResult, TileVoxelSourceBridge, session}};

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
pub(crate) struct TileRequester<'w, 's> {
	bridge: Res<'w, TileVoxelSourceBridge>,
	builders: Res<'w, TileBuilderRegistry>,
	results: Res<'w, TileBuildingChannel>,
	grids: Query<'w, 's, &'static mut GridStreaming>,
}

impl<'w, 's> TileRequester<'w, 's> {
	pub fn fetch_tile(
		&mut self,
		grid: GridId,
		requester: Entity,
		key: TileKey,
		priority: f32,
		context: Option<&TileBuildingParameters>,
	) -> bool {
		if !crate::streaming::valid_tile_key(key) { return false; }
		let Ok(mut streaming) = self.grids.get_mut(grid) else { return false; };
		let state = streaming.tiles.entry(key).or_insert_with(|| {
			streaming.retain_edit_interest_region(key.region.into());
			TileState {
				requesters: FxHashMap::default(),
				status: Self::spawn(&self.bridge, &self.builders, &self.results, &mut streaming, grid, key, context),
				active: None,
			}
		});
		state.requesters.insert(requester, priority);
		true
	}

	pub(crate) fn dirty_stale_tiles(&mut self, grid: GridId, region: NonZeroChunkRegion, generation: u64, context: Option<&TileBuildingParameters>) {
		let Ok(mut streaming) = self.grids.get_mut(grid) else { return };
		let keys: FxHashSet<_> = streaming.tile_dependencies.stale_tiles(region, generation).collect();
		for key in keys {
			Self::dirty_tile_inner(
				&self.bridge,
				&self.builders,
				&self.results,
				&mut streaming,
				grid,
				key,
				context
			);
		}
	}

	pub(crate) fn invalidate_building_context(&mut self, grid: GridId, context: Option<&TileBuildingParameters>) {
		let Ok(mut streaming) = self.grids.get_mut(grid) else { return };
		let keys: Vec<_> = streaming.tiles.keys().copied().collect();
		for key in keys {
			Self::dirty_tile_inner(
				&self.bridge,
				&self.builders,
				&self.results,
				&mut streaming,
				grid,
				key,
				context
			);
		}
	}

	fn dirty_tile_inner(
		bridge: &TileVoxelSourceBridge,
		builders: &TileBuilderRegistry,
		results: &TileBuildingChannel,
		streaming: &mut GridStreaming,
		grid: GridId,
		key: TileKey,
		context: Option<&TileBuildingParameters>
	) {
		let Some(state) = streaming.tiles.get_mut(&key) else { return };
		if state.requesters.is_empty() { return; }
		if let TileStatus::InFlight { tag, cancellation } = std::mem::replace(&mut state.status, TileStatus::Dirty) {
			streaming.inflight_tiles_by_tag.remove(&tag);
			cancellation.cancel();
		}
		Self::spawn(bridge, builders, results, streaming, grid, key, context);
	}

	fn spawn(
		bridge: &TileVoxelSourceBridge,
		builders: &TileBuilderRegistry,
		results: &TileBuildingChannel,
		streaming: &mut GridStreaming,
		grid: GridId,
		key: TileKey,
		context: Option<&TileBuildingParameters>,
	) -> TileStatus {
		streaming.next_tile_tag += 1;
		let tag = streaming.next_tile_tag;
		let context = context.map_or_default(|context| context.clone());
		let builder = builders.builder(key.class);
		let cancellation = CancellationToken::new();
		let metadata = Arc::new(Mutex::new(TileBuildingMetadata::default()));
		let (session, generation_cancellation) = session(grid, key, context.clone(), bridge.sender(), cancellation.clone(), metadata.clone());
		let result_tx = results.sender();
		let task_cancellation = cancellation.clone();
		AsyncComputeTaskPool::get().spawn(async move {
			let data = builder.build(session).await;
			if task_cancellation.is_cancelled() { return; }
			let mut metadata = metadata.lock().unwrap();
			let dependencies = std::mem::take(&mut metadata.dependencies);
			drop(metadata);
			let _ = result_tx.send(TileBuildingResult { grid, tag, context, dependencies, data });
		}.instrument(bevy::log::info_span!("build tile"))).detach();
		streaming.inflight_tiles_by_tag.insert(tag, key);
		TileStatus::InFlight { tag, cancellation: generation_cancellation }
	}
}
