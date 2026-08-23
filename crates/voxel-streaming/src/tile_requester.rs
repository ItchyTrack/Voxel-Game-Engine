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
		let (new_requester, first_requester, ready_status, should_spawn) = {
			let state = streaming.tiles.entry(key).or_insert_with(|| TileState {
				requesters: FxHashMap::default(),
				status: TileStatus::Requested,
				active: None,
			});
			let new_requester = state.requesters.insert(requester, priority).is_none();
			let ready_status = match (&state.status, state.active) {
				(TileStatus::Loaded, Some(entity)) => Some(TileLoadStatus::Ready(entity)),
				(TileStatus::Empty, _) => Some(TileLoadStatus::Empty),
				_ => None,
			};
			let should_spawn = ready_status.is_none() && matches!(&state.status, TileStatus::Requested | TileStatus::Dirty);
			(new_requester, new_requester && state.requesters.len() == 1, ready_status, should_spawn)
		};
		if first_requester { streaming.retain_edit_interest_region(key.region.into()); }
		if new_requester && let Some(status) = ready_status {
			streaming.queued_tile_updates.push(TileLoadUpdate { grid: Entity::PLACEHOLDER, requester, key, status });
		}
		if should_spawn {
			Self::spawn(&self.bridge, &self.builders, &self.results, &mut streaming, grid, key, context);
		}
		true
	}

	pub(crate) fn dirty_stale_tiles(&mut self, grid: GridId, region: NonZeroChunkRegion, generation: u64, context: Option<&TileBuildingParameters>) {
		let Ok(mut streaming) = self.grids.get_mut(grid) else { return };
		let keys: FxHashSet<_> = streaming.tile_dependencies.stale_tiles(region, generation).collect();
		for key in keys { self.dirty_tile_inner(&mut streaming, grid, key, context); }
	}

	pub(crate) fn invalidate_building_context(&mut self, grid: GridId, context: Option<&TileBuildingParameters>) {
		let Ok(mut streaming) = self.grids.get_mut(grid) else { return };
		let keys: Vec<_> = streaming.tiles.keys().copied().collect();
		for key in keys { self.dirty_tile_inner(&mut streaming, grid, key, context); }
	}

	fn dirty_tile_inner(&self, streaming: &mut GridStreaming, grid: GridId, key: TileKey, context: Option<&TileBuildingParameters>) {
		let Some(state) = streaming.tiles.get_mut(&key) else { return };
		if state.requesters.is_empty() { return; }
		if let TileStatus::InFlight { tag, cancellation } = std::mem::replace(&mut state.status, TileStatus::Dirty) {
			streaming.inflight_tiles_by_tag.remove(&tag);
			cancellation.cancel();
		}
		Self::spawn(&self.bridge, &self.builders, &self.results, streaming, grid, key, context);
	}

	fn spawn(
		bridge: &TileVoxelSourceBridge,
		builders: &TileBuilderRegistry,
		results: &TileBuildingChannel,
		streaming: &mut GridStreaming,
		grid: GridId,
		key: TileKey,
		context: Option<&TileBuildingParameters>,
	) {
		streaming.next_tile_tag = streaming.next_tile_tag.wrapping_add(1).max(1);
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
		streaming.tiles.get_mut(&key).unwrap().status = TileStatus::InFlight { tag, cancellation: generation_cancellation };
	}
}
