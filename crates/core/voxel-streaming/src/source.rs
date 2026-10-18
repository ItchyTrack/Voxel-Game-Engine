use std::{collections::HashMap, future::Future, pin::Pin, sync::Arc};

use bevy::{ecs::component::{ComponentId, ComponentIdFor}, prelude::*, tasks::AsyncComputeTaskPool};
use bevy_trait_query::RegisterExt;
use bile_math::NonZeroRegion;
use bile_tiles::{BileTilesSchedule, BileTilesSystemSet, CancelRequestMessage, ComponentSource, DesiredComponents, FinishedRequestMessage, RegisterTileClass, TileKey};
use crossbeam_channel::{Receiver, Sender, unbounded};
use futures::{StreamExt, channel::mpsc::{UnboundedReceiver, UnboundedSender, unbounded as async_unbounded}};
use tile_data::{TileBuilder, TileBuildingSession, TileBuildingVoxelReader, VoxelRegionRequest, VoxelRegionResult, chunk_aligned};
use voxel_data::grid::GridId;
use voxel_sources::{RequestId, SourceManager, SourceResult, SourceResultData, edit::{GridEditIdManager, GridEditMessage, GridGeneration}};
use voxel_tasks::CancellationToken;

use crate::{ChunkEditInterest, tile_dependency_index::{TileComponent, TileDependencyIndex}};

type TileInsert = Box<dyn FnOnce(&mut EntityCommands) + Send>;
type BuildFuture = Pin<Box<dyn Future<Output = Option<TileInsert>> + Send>>;

struct RegisteredBuilder {
	build: Arc<dyn Fn(TileBuildingSession) -> BuildFuture + Send + Sync>,
	rebuild_on_edit: bool,
}

struct TileRequest {
	grid: GridId,
	tile: Entity,
	key: TileKey,
	component: ComponentId,
}

struct InFlight {
	build: u64,
	cancellation: CancellationToken,
	wake: UnboundedSender<VoxelLoadEvent>,
}

impl InFlight {
	fn cancel(&self) {
		self.cancellation.cancel();
		let _ = self.wake.unbounded_send(VoxelLoadEvent::Cancelled);
	}
}

struct TileState {
	key: TileKey,
	finished: bool,
	in_flight: Option<InFlight>,
}

#[derive(Default)]
pub(crate) struct GridTiles {
	tiles: HashMap<TileComponent, TileState>,
	dependencies: TileDependencyIndex,
	edit_interest: ChunkEditInterest,
	pub(crate) queued_edit_interest: Vec<(NonZeroRegion, bool)>,
	pub(crate) presence_request: Option<RequestId>,
}

impl GridTiles {
	fn release(&mut self, tile: TileComponent) {
		let Some(state) = self.tiles.remove(&tile) else { return };
		if let Some(in_flight) = state.in_flight { in_flight.cancel(); }
		self.dependencies.remove(tile);
		if self.edit_interest.release(state.key.region) {
			self.queued_edit_interest.push((state.key.region, false));
		}
	}
}

struct BuildResult {
	grid: GridId,
	tile: TileComponent,
	build: u64,
	insert: Option<TileInsert>,
}

enum VoxelLoadEvent {
	Result(VoxelRegionResult),
	Loaded,
	Cancelled,
}

struct VoxelRequest {
	grid: GridId,
	tile: TileComponent,
	build: u64,
	generation: GridGeneration,
	request: VoxelRegionRequest,
	events: UnboundedSender<VoxelLoadEvent>,
	cancellation: CancellationToken,
}

struct VoxelRoute {
	events: UnboundedSender<VoxelLoadEvent>,
	cancellation: CancellationToken,
}

#[derive(Resource)]
pub struct VoxelTileSource {
	builders: HashMap<ComponentId, RegisteredBuilder>,
	requested: Vec<TileRequest>,
	pub(crate) grids: HashMap<GridId, GridTiles>,
	next_build: u64,
	results: (Sender<BuildResult>, Receiver<BuildResult>),
	voxel_requests: (Sender<VoxelRequest>, Receiver<VoxelRequest>),
	voxel_routes: HashMap<RequestId, VoxelRoute>,
}

impl Default for VoxelTileSource {
	fn default() -> Self {
		Self {
			builders: HashMap::new(),
			requested: Vec::new(),
			grids: HashMap::new(),
			next_build: 0,
			results: unbounded(),
			voxel_requests: unbounded(),
			voxel_routes: HashMap::new(),
		}
	}
}

impl ComponentSource for VoxelTileSource {
	fn component_cost(&self, _entity: Entity, _tile_key: TileKey, component: ComponentId) -> Option<f32> {
		self.builders.contains_key(&component).then_some(1.0)
	}

	fn request_component(&mut self, entity: Entity, tile_entity: Entity, tile_key: TileKey, component: ComponentId) {
		self.requested.push(TileRequest { grid: entity, tile: tile_entity, key: tile_key, component });
	}
}

impl VoxelTileSource {
	pub fn edit_interest(&self, grid: GridId) -> Option<&ChunkEditInterest> {
		self.grids.get(&grid).map(|tiles| &tiles.edit_interest)
	}

	pub fn update_tiles(&mut self, grid: GridId, region: NonZeroRegion, generation: GridGeneration) {
		let Some(tiles) = self.grids.get(&grid) else { return };
		let stale: Vec<_> = tiles.dependencies.tiles_using_region(chunk_aligned(region)).into_iter().filter(|tile| {
			let state = &tiles.tiles[tile];
			state.in_flight.is_some() || (state.finished && self.builders[&tile.1].rebuild_on_edit)
		}).collect();
		for tile in stale {
			let key = self.grids[&grid].tiles[&tile].key;
			self.start_build(grid, tile, key, generation);
		}
	}

	fn start_build(&mut self, grid: GridId, tile: TileComponent, key: TileKey, generation: GridGeneration) {
		let build = self.next_build;
		self.next_build += 1;
		let builder = self.builders[&tile.1].build.clone();
		let cancellation = CancellationToken::new();
		let (events_tx, events_rx) = async_unbounded();
		let reader = StreamingVoxelReader {
			grid,
			tile,
			build,
			generation,
			requests: self.voxel_requests.0.clone(),
			events_tx: events_tx.clone(),
			events_rx,
			outstanding: 0,
			cancellation: cancellation.clone(),
		};
		let session = TileBuildingSession::new(grid, key, Box::new(reader));
		let results = self.results.0.clone();
		let task_cancellation = cancellation.clone();
		AsyncComputeTaskPool::get().spawn(async move {
			let insert = builder(session).await;
			if task_cancellation.is_cancelled() { return; }
			let _ = results.send(BuildResult { grid, tile, build, insert });
		}).detach();

		let tiles = self.grids.entry(grid).or_default();
		tiles.dependencies.remove(tile);
		let state = tiles.tiles.entry(tile).or_insert_with(|| {
			tiles.edit_interest.retain(key.region);
			tiles.queued_edit_interest.push((key.region, true));
			TileState { key, finished: false, in_flight: None }
		});
		if let Some(old) = state.in_flight.replace(InFlight { build, cancellation, wake: events_tx }) { old.cancel(); }
	}
}

struct StreamingVoxelReader {
	grid: GridId,
	tile: TileComponent,
	build: u64,
	generation: GridGeneration,
	requests: Sender<VoxelRequest>,
	events_tx: UnboundedSender<VoxelLoadEvent>,
	events_rx: UnboundedReceiver<VoxelLoadEvent>,
	outstanding: usize,
	cancellation: CancellationToken,
}

impl TileBuildingVoxelReader for StreamingVoxelReader {
	fn request_voxels(&mut self, request: VoxelRegionRequest) {
		if self.cancellation.is_cancelled() { return; }
		self.outstanding += 1;
		if self.requests.send(VoxelRequest {
			grid: self.grid,
			tile: self.tile,
			build: self.build,
			generation: self.generation,
			request,
			events: self.events_tx.clone(),
			cancellation: self.cancellation.clone(),
		}).is_err() {
			self.outstanding -= 1;
		}
	}

	fn receive_voxels(&mut self) -> tile_data::ReceiveVoxelsFuture<'_> {
		Box::pin(async move {
			while self.outstanding > 0 {
				match self.events_rx.next().await.expect("voxel result channel closed with requests outstanding") {
					VoxelLoadEvent::Cancelled => {
						self.outstanding = 0;
						return None;
					}
					VoxelLoadEvent::Loaded => self.outstanding -= 1,
					VoxelLoadEvent::Result(result) => return Some(result),
				}
			}
			None
		})
	}
}

pub trait TileBuilderAppExt {
	fn register_tile_builder<B: TileBuilder>(&mut self, builder: B) -> &mut Self;
	fn register_tile_class_component<Class: Component, C: Component>(&mut self) -> &mut Self;
}

impl TileBuilderAppExt for App {
	fn register_tile_builder<B: TileBuilder>(&mut self, builder: B) -> &mut Self {
		let component = self.world_mut().register_component::<B::Output>();
		let builder = Arc::new(builder);
		let build = Arc::new(move |session: TileBuildingSession| -> BuildFuture {
			let builder = builder.clone();
			Box::pin(async move {
				builder.build(session).await.map(|output| Box::new(move |entity: &mut EntityCommands| { entity.try_insert(output); }) as TileInsert)
			})
		});
		let previous = self.world_mut()
			.get_resource_or_init::<VoxelTileSource>()
			.builders
			.insert(component, RegisteredBuilder { build, rebuild_on_edit: B::REBUILD_ON_EDIT });
		assert!(previous.is_none(), "tile builder already registered for {}", std::any::type_name::<B::Output>());
		self.add_observer(release_removed_tile::<B::Output>)
	}

	fn register_tile_class_component<Class: Component, C: Component>(&mut self) -> &mut Self {
		self.world_mut().register_component::<Class>();
		self.world_mut().register_component::<C>();
		self.register_tile_class::<Class>()
			.add_systems(BileTilesSchedule, desire_class_component::<Class, C>.in_set(BileTilesSystemSet::UpdateWantedComponents))
	}
}

fn desire_class_component<Class: Component, C: Component>(
	class: ComponentIdFor<Class>,
	component: ComponentIdFor<C>,
	mut tiles: Query<&mut DesiredComponents, Added<Class>>,
) {
	for mut desired in &mut tiles {
		desired.add_desired_component(class.get(), component.get());
	}
}

pub(crate) fn register_source(app: &mut App) {
	app.world_mut().get_resource_or_init::<VoxelTileSource>();
	app.register_component_as::<dyn ComponentSource, VoxelTileSource>();
}

fn release_removed_tile<C: Component>(
	remove: On<Remove, C>,
	component: ComponentIdFor<C>,
	tiles: Query<(&ChildOf, &DesiredComponents)>,
	mut source: ResMut<VoxelTileSource>,
) {
	let Ok((child_of, desired)) = tiles.get(remove.entity) else { return };
	if desired.is_component_desired(component.get()) { return; }
	if let Some(tiles) = source.grids.get_mut(&child_of.parent()) {
		tiles.release((remove.entity, component.get()));
	}
}

pub(crate) fn release_removed_grid(remove: On<Remove, bile_tiles::TileInstances>, mut source: ResMut<VoxelTileSource>) {
	let Some(tiles) = source.grids.remove(&remove.entity) else { return };
	for state in tiles.tiles.into_values() {
		if let Some(in_flight) = state.in_flight { in_flight.cancel(); }
	}
}

pub(crate) fn start_requested_builds(
	mut source: ResMut<VoxelTileSource>,
	grids: Query<&GridEditIdManager>,
) {
	for request in std::mem::take(&mut source.requested) {
		let generation = grids.get(request.grid).map(|ids| ids.latest_generation()).unwrap_or_default();
		if let Some(state) = source.grids.get_mut(&request.grid).and_then(|tiles| tiles.tiles.get_mut(&(request.tile, request.component))) {
			state.finished = false;
		}
		source.start_build(request.grid, (request.tile, request.component), request.key, generation);
	}
}

pub(crate) fn cancel_requests(
	mut cancels: MessageReader<CancelRequestMessage>,
	mut source: ResMut<VoxelTileSource>,
) {
	for cancel in cancels.read() {
		let Some(tiles) = source.grids.get_mut(&cancel.entity) else { continue };
		tiles.release((cancel.tile_entity, cancel.component_id));
	}
}

pub(crate) fn update_edited_tiles(
	mut edits: MessageReader<GridEditMessage>,
	mut source: ResMut<VoxelTileSource>,
) {
	for edit in edits.read() {
		source.update_tiles(edit.grid_id(), edit.edit().affected_region(), edit.grid_generation());
	}
}

pub(crate) fn submit_voxel_requests(
	mut source: ResMut<VoxelTileSource>,
	mut sources: ResMut<SourceManager>,
) {
	let source = &mut *source;
	for request in source.voxel_requests.1.try_iter() {
		let current = source.grids.get(&request.grid)
			.and_then(|tiles| tiles.tiles.get(&request.tile))
			.and_then(|state| state.in_flight.as_ref())
			.is_some_and(|in_flight| in_flight.build == request.build);
		if !current || request.cancellation.is_cancelled() {
			let _ = request.events.unbounded_send(VoxelLoadEvent::Cancelled);
			continue;
		}
		source.grids.get_mut(&request.grid).unwrap().dependencies.add(request.tile, request.request.area);
		let request_id = sources.request_voxels(
			request.grid,
			request.request.area,
			request.request.lod,
			request.request.voxel_type,
			request.generation,
		);
		source.voxel_routes.insert(request_id, VoxelRoute { events: request.events, cancellation: request.cancellation });
	}

	source.voxel_routes.retain(|request_id, route| {
		if !route.cancellation.is_cancelled() { return true; }
		sources.cancel_voxels(*request_id);
		false
	});
}

pub(crate) fn route_voxel_results(
	mut source: ResMut<VoxelTileSource>,
	mut results: MessageReader<SourceResult>,
) {
	for result in results.read() {
		let Some(route) = source.voxel_routes.get(&result.request_id) else { continue };
		if route.cancellation.is_cancelled() { continue; }
		match &result.data {
			SourceResultData::Voxels { region, lod, voxels, .. } => {
				let _ = route.events.unbounded_send(VoxelLoadEvent::Result(VoxelRegionResult { area: *region, lod: *lod, voxels: voxels.clone() }));
			}
			SourceResultData::VoxelsLoaded { .. } => {
				let route = source.voxel_routes.remove(&result.request_id).unwrap();
				let _ = route.events.unbounded_send(VoxelLoadEvent::Loaded);
			}
			SourceResultData::Presence { .. } | SourceResultData::PresenceLoaded => {}
		}
	}
}

pub(crate) fn receive_tile_builds(
	mut commands: Commands,
	mut source: ResMut<VoxelTileSource>,
	mut finished: MessageWriter<FinishedRequestMessage>,
) {
	let results: Vec<_> = source.results.1.try_iter().collect();
	for result in results {
		let Some(state) = source.grids.get_mut(&result.grid).and_then(|tiles| tiles.tiles.get_mut(&result.tile)) else { continue };
		if state.in_flight.as_ref().is_none_or(|in_flight| in_flight.build != result.build) { continue; }
		state.in_flight = None;
		let (tile, component) = result.tile;
		match result.insert {
			Some(insert) => insert(&mut commands.entity(tile)),
			None if state.finished => { commands.entity(tile).remove_by_id(component); }
			None => {}
		}
		if !state.finished {
			state.finished = true;
			finished.write(FinishedRequestMessage { entity: result.grid, tile_entity: tile, tile_key: state.key, component_id: component });
		}
	}
}
