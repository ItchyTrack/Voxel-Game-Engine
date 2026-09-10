#[cfg(test)]
mod tests;

use std::any::Any;
use std::collections::HashSet;

use bevy::ecs::query::QueryFilter;
use bevy::ecs::system::SystemParam;
use bevy::math::{IVec3, UVec3};
use voxel_math::{Fixed, FixedVec3, Ray};
use voxel_transform::{Transform, TransformQuery};
use bevy::prelude::*;
use tile_data::{
	CHUNK_SIZE, DynamicTileData, LoadedTile, TileAppExt, TileBuilder, TileBuildingSession,
	TileClassId, TileData,
};
use voxel_transform::bvh::BVH;
use voxel_data::grid::{Grid, GridId};
use voxel_trees::grid_tree::{GridTree64, U16Cell};
use voxel_trees::views::GridTreeView;

pub type OccupancyTree = GridTree64<U16Cell>;

#[derive(Debug)]
pub struct OccupancyTileData {
	pub tree: OccupancyTree,
}

impl TileData for OccupancyTileData {
	fn as_any(&self) -> &dyn Any { self }
}

#[derive(Resource, Clone, Copy, Debug, PartialEq, Eq)]
pub struct OccupancyTileClass(pub TileClassId);

#[derive(Default)]
pub struct OccupancyTileBuilder;

#[tile_data::async_trait]
impl TileBuilder for OccupancyTileBuilder {
	async fn build(&self, mut session: TileBuildingSession) -> Option<Box<dyn TileData>> {
		let tile_region = session.key.region;
		assert_eq!(session.key.lod, 0, "occupancy tiles must use LOD 0");
		session.request_voxels(tile_region, 0, None);

		let mut areas = Vec::new();
		while let Some(input) = session.receive_voxels().await {
			assert_eq!(input.lod, 0, "occupancy tile source returned a nonzero LOD");
			let offset = (input.area.min() - tile_region.min()) * CHUNK_SIZE as i32;
			for (origin, size, _) in input.voxels.grid_tree() {
				let origin = offset + origin.as_ivec3();
				assert!(origin.cmpge(IVec3::ZERO).all(), "occupancy source data starts before its requested tile");
				areas.push((origin.as_uvec3(), UVec3::splat(size), 0u16));
			}
		}

		if areas.is_empty() { return None; }
		let mut tree = OccupancyTree::new();
		tree.add_areas(&areas);
		Some(Box::new(OccupancyTileData { tree }))
	}
}

#[derive(Default)]
pub struct VoxelQueryPlugin;

impl Plugin for VoxelQueryPlugin {
	fn build(&self, app: &mut App) {
		if !app.is_plugin_added::<tile_data::TileDataPlugin>() {
			app.add_plugins(tile_data::TileDataPlugin);
		}
		let class = app.register_tile_class();
		app.register_tile_builder(class, OccupancyTileBuilder)
			.insert_resource(OccupancyTileClass(class));
	}
}

#[derive(Debug, Clone, Copy)]
pub struct VoxelWorldRaycastHit {
	pub grid: GridId,
	pub voxel_pos: IVec3,
	pub normal: IVec3,
	pub world_position: FixedVec3,
	pub distance: Fixed,
}

#[derive(SystemParam)]
pub struct VoxelWorldQueryParam<'w, 's, GridFilter = ()>
where
	GridFilter: QueryFilter + 'static,
{
	class: Res<'w, OccupancyTileClass>,
	grids: Query<'w, 's, Entity, (With<Grid>, GridFilter)>,
	tiles: Query<'w, 's, (Entity, &'static LoadedTile, &'static DynamicTileData)>,
	transforms: TransformQuery<'w, 's>,
}

struct RaycastCandidate<'a> {
	loaded: &'a LoadedTile,
	occupancy: &'a OccupancyTileData,
	transform: Transform,
}

impl<'w, 's, GridFilter> VoxelWorldQueryParam<'w, 's, GridFilter>
where
	GridFilter: QueryFilter + 'static,
{
	pub fn raycast(&self, origin: FixedVec3, direction: FixedVec3, max_distance: Option<Fixed>) -> Option<VoxelWorldRaycastHit> {
		let direction = direction.normalize_or_zero();
		if direction == FixedVec3::ZERO || max_distance.is_some_and(|distance| distance < Fixed::ZERO) { return None; }

		let selected_grids: HashSet<_> = self.grids.iter().collect();
		let candidates: Vec<_> = self.tiles.iter().filter_map(|(entity, loaded, data)| {
			if loaded.key.class != self.class.0 || !selected_grids.contains(&loaded.grid) { return None; }
			let occupancy = data.downcast_ref::<OccupancyTileData>()?;
			let transform = self.transforms.get_world(entity)?;
			Some(RaycastCandidate { loaded, occupancy, transform })
		}).collect();

		let bounds = candidates.iter().enumerate().filter_map(|(index, candidate)| {
			let bounds = candidate.occupancy.tree.occupied_bounds()?;
			let lo = FixedVec3::from(bounds.min());
			let hi = FixedVec3::from(bounds.end());
			let world_bounds = voxel_transform::aabb::aabb_of_transformed_aabb(&candidate.transform, lo, hi);
			Some((index, world_bounds))
		}).collect();
		let bvh = BVH::new(bounds);
		let ray = Ray { origin, direction };
		let mut best: Option<VoxelWorldRaycastHit> = None;

		for (index, bounds_distance) in bvh.raycast(&ray, max_distance) {
			if best.is_some_and(|hit| bounds_distance > hit.distance) { break; }
			let candidate = &candidates[index];
			let local_origin = candidate.transform.inverse_transform_point(origin);
			// Keep the direction unnormalized so the ray parameter remains world distance.
			let local_direction = candidate.transform.inverse_transform_vector(direction);
			let local_ray = Ray { origin: local_origin, direction: local_direction };
			let Some((tile_voxel, normal, distance)) = candidate.occupancy.tree.raycast(&local_ray, max_distance) else { continue };
			if best.is_some_and(|hit| hit.distance <= distance) { continue; }
			let tile_origin = candidate.loaded.key.region.min() * CHUNK_SIZE as i32;
			best = Some(VoxelWorldRaycastHit {
				grid: candidate.loaded.grid,
				voxel_pos: tile_origin + tile_voxel.as_ivec3(),
				normal: normal.as_ivec3(),
				world_position: origin + direction * distance,
				distance,
			});
		}

		best
	}
}
