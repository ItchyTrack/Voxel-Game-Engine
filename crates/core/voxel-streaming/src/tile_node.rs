use std::{
	any::TypeId,
	collections::{HashMap, HashSet},
	marker::PhantomData,
	sync::Arc,
};

use bevy::{
	ecs::component::ComponentIdFor,
	prelude::*,
	tasks::{AsyncComputeTaskPool, Task, futures::check_ready},
};
use bile_components::{AddComponentData, ComponentData};
use bile_data::{
	DataType, DirtyState, Request,
	diff::{BileRegisterDiff, DiffApplyingNode},
	simple::{BileRegisterSimple, DataDirtier, DataPasser, DataProducingNode},
	streaming::{self, BileRegisterStreaming},
};
use bile_tiles::{BileTilesSchedule, BileTilesSystemSet, DesireForClass, RegisterTileClass, TileKey};
use tile_data::VoxelRegion;
use voxel_data::grid::GridId;
use voxel_sources::{StreamCollector, VoxelData, VoxelRequest, VoxelRequester, VoxelSourceNodeSet, edit::VoxelEdit};

/// Every region streamed for a [`VoxelRequest`].
type VoxelRegions = Arc<Vec<VoxelRegion>>;

/// The grid and the tile within it.
pub type TileParams = (GridId, TileKey);

/// Bile data for a tile component built from voxels. Requested through the tile's class. The diffs are voxel edits.
pub struct TileComponentData<C>(PhantomData<C>);

impl<C: Component + Clone> DataType for TileComponentData<C> {
	type Params = TileParams;
	type Data = Option<C>;
	type Diff = VoxelEdit;
}

impl<C: Component + Clone> ComponentData for TileComponentData<C> {
	type Component = C;
}

/// Builds a tile component from the voxels of a single [`VoxelRequest`].
pub trait VoxelTileBuilder: Send + Sync + 'static {
	type Output: Component + Clone;

	fn voxels(&self, grid: GridId, key: TileKey) -> VoxelRequest;
	fn build(&self, grid: GridId, key: TileKey, voxels: &[VoxelRegion]) -> Option<Self::Output>;
}

/// A [`VoxelTileBuilder`] that can apply voxel edits to a finished tile instead of building it again.
/// Registered with [`TileBuilderAppExt::register_tile_editor`], its tiles are edited through Bile diffs.
pub trait VoxelTileEditor: VoxelTileBuilder {
	fn apply_edits(&self, grid: GridId, key: TileKey, output: &mut Option<Self::Output>, edits: &[VoxelEdit]);
}

enum TileNodeEvent {
	Request(TileParams),
	Drop(TileParams),
	Voxels(VoxelRequest, VoxelRegions),
	DirtyVoxels(VoxelRequest, Option<VoxelEdit>),
}

struct TileState<C> {
	voxels: VoxelRequest,
	waiting: bool,
	passed: bool,
	build: Option<Task<Option<C>>>,
}

/// A Bile node that produces `B::Output` for tiles by consuming [`VoxelData`].
#[derive(Component)]
pub struct VoxelTileNode<B: VoxelTileBuilder> {
	builder: Arc<B>,
	events: Vec<TileNodeEvent>,
	tiles: HashMap<TileParams, TileState<B::Output>>,
	tiles_by_voxels: HashMap<VoxelRequest, HashSet<TileParams>>,
	voxels: StreamCollector<VoxelRequest, VoxelRegion>,
	/// Finished tiles are edited through their diffs instead of being built again.
	applies_edits: bool,
}

impl<B: VoxelTileBuilder> VoxelTileNode<B> {
	fn new(builder: B, applies_edits: bool) -> Self {
		Self {
			builder: Arc::new(builder),
			events: Vec::new(),
			tiles: HashMap::new(),
			tiles_by_voxels: HashMap::new(),
			voxels: StreamCollector::default(),
			applies_edits,
		}
	}

	fn start_build(&mut self, tile: TileParams, voxels: VoxelRegions) {
		let Some(state) = self.tiles.get_mut(&tile) else { return };
		state.waiting = false;
		let builder = self.builder.clone();
		state.build = Some(AsyncComputeTaskPool::get().spawn(async move { builder.build(tile.0, tile.1, &voxels) }));
	}

	/// The voxels are streamed back, also when Bile already has them.
	fn request_voxels(&mut self, tile: TileParams, entity: Entity, requester: &mut VoxelRequester) {
		let Some(state) = self.tiles.get_mut(&tile) else { return };
		state.build = None;
		state.waiting = true;
		requester.request(state.voxels, entity);
	}
}

impl<B: VoxelTileBuilder> DataProducingNode<TileComponentData<B::Output>> for VoxelTileNode<B> {
	fn cost(&self, _request: &Request<TileComponentData<B::Output>>) -> Option<f32> { Some(1.0) }

	fn request(&mut self, request: Request<TileComponentData<B::Output>>) {
		self.events.push(TileNodeEvent::Request(request.params));
	}

	fn drop(&mut self, params: &TileParams) {
		self.events.push(TileNodeEvent::Drop(*params));
	}
}

impl<B: VoxelTileEditor> DiffApplyingNode<TileComponentData<B::Output>> for VoxelTileNode<B> {
	fn diff_cost(&self, _request: &Request<TileComponentData<B::Output>>, _diffs: &[VoxelEdit]) -> Option<f32> { Some(0.5) }

	fn apply(&mut self, tile: &TileParams, output: &mut Option<B::Output>, edits: Vec<VoxelEdit>) -> bool {
		self.builder.apply_edits(tile.0, tile.1, output, &edits);
		true
	}
}

impl<B: VoxelTileBuilder> streaming::DataConsumingNode<VoxelData> for VoxelTileNode<B> {
	fn start_data(&mut self, params: &VoxelRequest, tick: u64) { self.voxels.start(params, tick); }

	fn push_data(&mut self, params: &VoxelRequest, region: VoxelRegion) { self.voxels.push(params, region); }

	fn finish_data(&mut self, params: &VoxelRequest) {
		if let Some(regions) = self.voxels.finish(params) {
			self.events.push(TileNodeEvent::Voxels(*params, Arc::new(regions)));
		}
	}

	fn dirty_data(&mut self, params: &VoxelRequest, last_clean_tick: u64, edit: Option<&VoxelEdit>) {
		self.voxels.dirty(params, last_clean_tick);
		self.events.push(TileNodeEvent::DirtyVoxels(*params, edit.cloned()));
	}
}

fn update_voxel_tiles<B: VoxelTileBuilder>(
	node: Single<(Entity, &mut VoxelTileNode<B>)>,
	mut requester: VoxelRequester,
	mut tile_components: ParamSet<(DataDirtier<TileComponentData<B::Output>>, DataPasser<TileComponentData<B::Output>>)>,
) {
	let (entity, node) = node.into_inner();
	let node = node.into_inner();
	for event in std::mem::take(&mut node.events) {
		match event {
			TileNodeEvent::Request(tile) => {
				let state = node.tiles.entry(tile).or_insert_with(|| {
					let voxels = node.builder.voxels(tile.0, tile.1);
					node.tiles_by_voxels.entry(voxels).or_default().insert(tile);
					TileState { voxels, waiting: false, passed: false, build: None }
				});
				if state.waiting || state.build.is_some() { continue; }
				node.request_voxels(tile, entity, &mut requester);
			}
			TileNodeEvent::Drop(tile) => {
				let Some(state) = node.tiles.remove(&tile) else { continue };
				let Some(tiles) = node.tiles_by_voxels.get_mut(&state.voxels) else { continue };
				tiles.remove(&tile);
				if tiles.is_empty() {
					node.tiles_by_voxels.remove(&state.voxels);
					node.voxels.forget(&state.voxels);
					requester.cancel(&state.voxels, entity);
				}
			}
			TileNodeEvent::Voxels(voxels, data) => {
				let Some(tiles) = node.tiles_by_voxels.get(&voxels) else { continue };
				let waiting: Vec<_> = tiles.iter().copied().filter(|tile| node.tiles[tile].waiting).collect();
				for tile in waiting {
					node.start_build(tile, data.clone());
				}
			}
			TileNodeEvent::DirtyVoxels(voxels, edit) => {
				let Some(tiles) = node.tiles_by_voxels.get(&voxels) else { continue };
				let tiles: Vec<_> = tiles.iter().copied().collect();
				for tile in tiles {
					let passed = node.tiles[&tile].passed;
					if passed {
						tile_components.p0().mark_dirty(&tile, edit.as_ref());
					}
					// A finished tile with an edit is edited by Bile through the diff.
					if !(passed && node.applies_edits && edit.is_some()) {
						node.request_voxels(tile, entity, &mut requester);
					}
				}
			}
		}
	}

	for (&tile, state) in &mut node.tiles {
		let Some(build) = &mut state.build else { continue };
		let Some(output) = check_ready(build) else { continue };
		state.build = None;
		state.passed = true;
		tile_components.p1().pass_data(entity, &tile, DirtyState::UpToDate, output);
	}
}

#[derive(Resource, Default)]
struct RegisteredTileBuilders(HashSet<TypeId>);

pub trait TileBuilderAppExt {
	fn register_tile_builder<B: VoxelTileBuilder>(&mut self, builder: B) -> &mut Self;
	fn register_tile_editor<B: VoxelTileEditor>(&mut self, builder: B) -> &mut Self;
	fn register_tile_class_component<Class: Component, C: Component + Clone>(&mut self) -> &mut Self;
}

impl TileBuilderAppExt for App {
	fn register_tile_builder<B: VoxelTileBuilder>(&mut self, builder: B) -> &mut Self {
		add_tile_builder(self, builder, false)
	}

	fn register_tile_editor<B: VoxelTileEditor>(&mut self, builder: B) -> &mut Self {
		add_tile_builder(self, builder, true).register_diff_applier::<TileComponentData<B::Output>, VoxelTileNode<B>>()
	}

	fn register_tile_class_component<Class: Component, C: Component + Clone>(&mut self) -> &mut Self {
		self.world_mut().register_component::<Class>();
		self.world_mut().register_component::<C>();
		self.register_tile_class::<Class>()
			.add_systems(BileTilesSchedule, desire_class_component::<Class, C>.in_set(BileTilesSystemSet::UpdateWantedComponents))
	}
}

fn add_tile_builder<B: VoxelTileBuilder>(app: &mut App, builder: B, applies_edits: bool) -> &mut App {
	let newly_registered = app.world_mut()
		.get_resource_or_init::<RegisteredTileBuilders>()
		.0
		.insert(TypeId::of::<B::Output>());
	assert!(newly_registered, "tile builder already registered for {}", std::any::type_name::<B::Output>());
	app.world_mut().spawn((Name::new(std::any::type_name::<B>()), VoxelTileNode::new(builder, applies_edits)));
	app.add_component_data::<TileComponentData<B::Output>>()
		.register_producer_simple::<TileComponentData<B::Output>, VoxelTileNode<B>>()
		.register_consumer_streaming::<VoxelData, VoxelTileNode<B>>()
		.add_systems(Update, update_voxel_tiles::<B>.before(VoxelSourceNodeSet))
}

fn desire_class_component<Class: Component, C: Component + Clone>(
	mut commands: Commands,
	class: ComponentIdFor<Class>,
	tiles: Query<(Entity, &TileKey, &ChildOf), Added<Class>>,
) {
	for (entity, &key, child_of) in &tiles {
		commands.entity(entity).add_desired_component::<TileComponentData<C>>(class.get(), (child_of.parent(), key));
	}
}
