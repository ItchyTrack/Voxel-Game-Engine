use std::collections::HashSet;

use bevy::ecs::component::ComponentIdFor;
use bevy::ecs::query::QueryFilter;
use bevy::ecs::system::SystemParam;
use bevy::math::{IVec3, Quat, UVec3, Vec3};
use bevy::prelude::*;
use bile_tiles::{BileTilesSchedule, DesiredComponents, TileInstances, TileKey};
use tile_data::{TileBuilder, TileBuildingSession, chunk_aligned};
use voxel_data::bvh::BVH;
use voxel_data::grid::{Grid, GridId};
use voxel_sources::edit::GridEditMessage;
use voxel_streaming::{TileBuilderAppExt, VoxelTileSourceSet};
use voxel_trees::grid_tree::{GridTree64, U16Cell};
use voxel_trees::views::GridTreeView;

pub type OccupancyTree = GridTree64<U16Cell>;

#[derive(Component, Default)]
pub struct OccupancyTileClass;

#[derive(Component, Debug, Default)]
pub struct OccupancyTileData {
	pub tree: OccupancyTree,
}

#[derive(Default)]
pub struct OccupancyTileBuilder;

#[tile_data::async_trait]
impl TileBuilder for OccupancyTileBuilder {
	type Output = OccupancyTileData;
	const REBUILD_ON_EDIT: bool = false;

	async fn build(&self, mut session: TileBuildingSession) -> Option<OccupancyTileData> {
		let tile_region = session.key.region;
		assert_eq!(session.key.lod, 0, "occupancy tiles must use LOD 0");
		session.request_voxels(tile_region, 0, None);

		let mut areas = Vec::new();
		while let Some(input) = session.receive_voxels().await {
			assert_eq!(input.lod, 0, "occupancy tile source returned a nonzero LOD");
			let offset = input.area.min() - tile_region.min();
			for (origin, size, _) in input.voxels.grid_tree() {
				let origin = offset + origin.as_ivec3();
				assert!(origin.cmpge(IVec3::ZERO).all(), "occupancy source data starts before its requested tile");
				areas.push((origin.as_uvec3(), UVec3::splat(size), 0u16));
			}
		}

		if areas.is_empty() { return None; }
		let mut tree = OccupancyTree::default();
		tree.add_areas(&areas);
		Some(OccupancyTileData { tree })
	}
}

fn apply_edits_to_occupancy(
	mut commands: Commands,
	mut edits: MessageReader<GridEditMessage>,
	class: ComponentIdFor<OccupancyTileClass>,
	component: ComponentIdFor<OccupancyTileData>,
	grids: Query<&TileInstances>,
	mut tiles: Query<(&TileKey, &DesiredComponents, Option<&mut OccupancyTileData>)>,
) {
	for edit in edits.read() {
		let Ok(instances) = grids.get(edit.grid_id()) else { continue };
		let mut keys = Vec::new();
		instances.for_each_overlapping(chunk_aligned(edit.edit().affected_region()), |key| keys.push(key));
		for key in keys {
			let Some(entity) = instances.get(key) else { continue };
			let Ok((key, desired, occupancy)) = tiles.get_mut(entity) else { continue };
			if !desired.is_component_desired(component.get()) { continue; }
			match occupancy {
				Some(mut occupancy) => edit.edit().apply_to_occupancy(key.region, &mut occupancy.tree),
				None if desired.is_class_ready(class.get()) => {
					let mut occupancy = OccupancyTileData::default();
					edit.edit().apply_to_occupancy(key.region, &mut occupancy.tree);
					commands.entity(entity).insert(occupancy);
				}
				None => {}
			}
		}
	}
}

#[derive(Default)]
pub struct VoxelQueryPlugin;

impl Plugin for VoxelQueryPlugin {
	fn build(&self, app: &mut App) {
		app.register_tile_class_component::<OccupancyTileClass, OccupancyTileData>()
			.register_tile_builder(OccupancyTileBuilder)
			.add_systems(BileTilesSchedule, apply_edits_to_occupancy.in_set(VoxelTileSourceSet::Receive));
	}
}

#[derive(Debug, Clone, Copy)]
pub struct VoxelWorldRaycastHit {
	pub grid: GridId,
	pub voxel_pos: IVec3,
	pub normal: IVec3,
	pub world_position: Vec3,
	pub distance: f32,
}

#[derive(SystemParam)]
pub struct VoxelWorldQueryParam<'w, 's, GridFilter = ()>
where
	GridFilter: QueryFilter + 'static,
{
	grids: Query<'w, 's, (Entity, &'static Grid), GridFilter>,
	tiles: Query<'w, 's, (&'static TileKey, &'static ChildOf, &'static OccupancyTileData, &'static GlobalTransform)>,
}

struct RaycastCandidate<'a> {
	key: TileKey,
	grid: GridId,
	occupancy: &'a OccupancyTileData,
	transform: Transform,
}

impl<'w, 's, GridFilter> VoxelWorldQueryParam<'w, 's, GridFilter>
where
	GridFilter: QueryFilter + 'static,
{
	pub fn raycast(&self, origin: Vec3, direction: Vec3, max_distance: Option<f32>) -> Option<VoxelWorldRaycastHit> {
		let direction = direction.normalize_or_zero();
		if direction == Vec3::ZERO { return None; }

		let selected_grids: HashSet<_> = self.grids.iter().map(|(entity, _)| entity).collect();
		let candidates: Vec<_> = self.tiles.iter().filter_map(|(&key, child_of, occupancy, global)| {
			let grid = child_of.parent();
			if !selected_grids.contains(&grid) { return None; }
			let transform = global.compute_transform();
			if !transform.scale.abs_diff_eq(Vec3::ONE, 1e-5) { return None; }
			Some(RaycastCandidate { key, grid, occupancy, transform })
		}).collect();

		let bounds = candidates.iter().enumerate().filter_map(|(index, candidate)| {
			let bounds = candidate.occupancy.tree.occupied_bounds()?;
			let lo = bounds.min().as_vec3();
			let hi = bounds.end().as_vec3();
			Some((index, voxel_data::aabb::aabb_of_transformed_aabb(&candidate.transform, lo, hi)))
		}).collect();
		let bvh = BVH::new(bounds);
		let ray = Transform {
			translation: origin,
			rotation: Quat::from_rotation_arc(Vec3::Z, direction),
			scale: Vec3::ONE,
		};
		let mut best: Option<VoxelWorldRaycastHit> = None;

		for (index, bounds_distance) in bvh.raycast(&ray, max_distance) {
			if best.is_some_and(|hit| bounds_distance > hit.distance) { break; }
			let candidate = &candidates[index];
			let inverse_rotation = candidate.transform.rotation.inverse();
			let local_origin = inverse_rotation * (origin - candidate.transform.translation);
			let local_direction = inverse_rotation * direction;
			let local_ray = Transform {
				translation: local_origin,
				rotation: Quat::from_rotation_arc(Vec3::Z, local_direction),
				scale: Vec3::ONE,
			};
			let Some((tile_voxel, normal, distance)) = candidate.occupancy.tree.raycast(&local_ray, max_distance) else { continue };
			if best.is_some_and(|hit| hit.distance <= distance) { continue; }
			let tile_origin = candidate.key.region.min();
			best = Some(VoxelWorldRaycastHit {
				grid: candidate.grid,
				voxel_pos: tile_origin + tile_voxel.as_ivec3(),
				normal: normal.as_ivec3(),
				world_position: origin + direction * distance,
				distance,
			});
		}

		best
	}
}
