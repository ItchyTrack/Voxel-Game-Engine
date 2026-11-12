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
use bevy_trait_query::RegisterExt;
use bile_components::{AddComponentData, ComponentData};
use bile_data::{DataConsumingNode, DataDirtier, DataPasser, DataProducingNode, DataType, DirtyState, Request};
use bile_tiles::{BileTilesSchedule, BileTilesSystemSet, DesireForClass, RegisterTileClass, TileKey};
use tile_data::VoxelRegionResult;
use voxel_data::grid::GridId;
use voxel_sources::{VoxelData, VoxelRegions, VoxelRequest, VoxelRequester, VoxelSourceNodeSet};

/// The grid and the tile within it.
pub type TileParams = (GridId, TileKey);

/// Bile data for a tile component built from voxels. Requested through the tile's class.
pub struct TileComponentData<C>(PhantomData<C>);

impl<C: Component + Clone> DataType for TileComponentData<C> {
	type Params = TileParams;
	type Data = Option<C>;
}

impl<C: Component + Clone> ComponentData for TileComponentData<C> {
	type Component = C;
}

/// Builds a tile component from the voxels of a single [`VoxelRequest`].
pub trait VoxelTileBuilder: Send + Sync + 'static {
	type Output: Component + Clone;
	/// When false, finished tiles keep their component when their voxels are edited.
	const REBUILD_ON_EDIT: bool = true;

	fn voxels(&self, grid: GridId, key: TileKey) -> VoxelRequest;
	fn build(&self, grid: GridId, key: TileKey, voxels: &[VoxelRegionResult]) -> Option<Self::Output>;
}

enum TileNodeEvent {
	Request(TileParams),
	Drop(TileParams),
	Voxels(VoxelRequest, VoxelRegions),
	DirtyVoxels(VoxelRequest),
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
	dirtied: Vec<TileParams>,
}

impl<B: VoxelTileBuilder> VoxelTileNode<B> {
	fn new(builder: B) -> Self {
		Self {
			builder: Arc::new(builder),
			events: Vec::new(),
			tiles: HashMap::new(),
			tiles_by_voxels: HashMap::new(),
			dirtied: Vec::new(),
		}
	}

	fn start_build(&mut self, tile: TileParams, voxels: VoxelRegions) {
		let Some(state) = self.tiles.get_mut(&tile) else { return };
		state.waiting = false;
		let builder = self.builder.clone();
		state.build = Some(AsyncComputeTaskPool::get().spawn(async move { builder.build(tile.0, tile.1, &voxels) }));
	}

	fn request_voxels(&mut self, tile: TileParams, entity: Entity, requester: &mut VoxelRequester) {
		let Some(state) = self.tiles.get_mut(&tile) else { return };
		state.build = None;
		state.waiting = true;
		if let Some(voxels) = requester.request(state.voxels, entity) {
			self.start_build(tile, voxels);
		}
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

impl<B: VoxelTileBuilder> DataConsumingNode<VoxelData> for VoxelTileNode<B> {
	fn pass_data(&mut self, params: &VoxelRequest, _tick: u64, data: VoxelRegions) {
		self.events.push(TileNodeEvent::Voxels(*params, data));
	}

	fn dirty_data(&mut self, params: &VoxelRequest, _last_clean_tick: u64) {
		self.events.push(TileNodeEvent::DirtyVoxels(*params));
	}
}

fn update_voxel_tiles<B: VoxelTileBuilder>(
	node: Single<(Entity, &mut VoxelTileNode<B>)>,
	mut requester: VoxelRequester,
	mut passer: DataPasser<TileComponentData<B::Output>>,
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
			TileNodeEvent::DirtyVoxels(voxels) => {
				let Some(tiles) = node.tiles_by_voxels.get(&voxels) else { continue };
				let stale: Vec<_> = tiles.iter().copied().filter(|tile| B::REBUILD_ON_EDIT || !node.tiles[tile].passed).collect();
				for tile in stale {
					if node.tiles[&tile].passed {
						node.dirtied.push(tile);
					}
					node.request_voxels(tile, entity, &mut requester);
				}
			}
		}
	}

	for (&tile, state) in &mut node.tiles {
		let Some(build) = &mut state.build else { continue };
		let Some(output) = check_ready(build) else { continue };
		state.build = None;
		state.passed = true;
		passer.pass_data(entity, &tile, DirtyState::UpToDate, output);
	}
}

fn dirty_voxel_tiles<B: VoxelTileBuilder>(
	node: Single<&mut VoxelTileNode<B>>,
	mut dirtier: DataDirtier<TileComponentData<B::Output>>,
) {
	let mut node = node.into_inner();
	if node.dirtied.is_empty() { return; }
	for tile in std::mem::take(&mut node.dirtied) {
		dirtier.mark_dirty(&tile);
	}
}

#[derive(Resource, Default)]
struct RegisteredTileBuilders(HashSet<TypeId>);

pub trait TileBuilderAppExt {
	fn register_tile_builder<B: VoxelTileBuilder>(&mut self, builder: B) -> &mut Self;
	fn register_tile_class_component<Class: Component, C: Component + Clone>(&mut self) -> &mut Self;
}

impl TileBuilderAppExt for App {
	fn register_tile_builder<B: VoxelTileBuilder>(&mut self, builder: B) -> &mut Self {
		let newly_registered = self.world_mut()
			.get_resource_or_init::<RegisteredTileBuilders>()
			.0
			.insert(TypeId::of::<B::Output>());
		assert!(newly_registered, "tile builder already registered for {}", std::any::type_name::<B::Output>());
		self.world_mut().spawn((Name::new(std::any::type_name::<B>()), VoxelTileNode::new(builder)));
		self.add_component_data::<TileComponentData<B::Output>>()
			.register_component_as::<dyn DataProducingNode<TileComponentData<B::Output>>, VoxelTileNode<B>>()
			.register_component_as::<dyn DataConsumingNode<VoxelData>, VoxelTileNode<B>>()
			.add_systems(Update, (dirty_voxel_tiles::<B>, update_voxel_tiles::<B>).chain().before(VoxelSourceNodeSet))
	}

	fn register_tile_class_component<Class: Component, C: Component + Clone>(&mut self) -> &mut Self {
		self.world_mut().register_component::<Class>();
		self.world_mut().register_component::<C>();
		self.register_tile_class::<Class>()
			.add_systems(BileTilesSchedule, desire_class_component::<Class, C>.in_set(BileTilesSystemSet::UpdateWantedComponents))
	}
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
