use super::*;
use bevy::ecs::system::SystemState;
use tile_data::{NonZeroChunkRegion, TileKey};
use voxel_data::{grid::GridLocalTransform, voxels::{VoxelTypeId, VoxelTypeInfo}};
use voxel_transform::{Body, Scale};

#[test]
fn raycasts_keep_subvoxel_precision_without_propagation_at_large_positions() {
	for base in [0, 1i64 << 32, -(1i64 << 32)] {
		for scale in [Scale::ONE, Scale::from_num(2), Scale::from_num(0.5)] {
			let mut world = World::new();
			let class = TileClassId(0);
			world.insert_resource(OccupancyTileClass(class));
			let root_transform = Transform::from_xyz(base, base, base).with_scale(scale);
			let body = world.spawn((Body, root_transform)).id();
			let grid = world.spawn((
				Grid::new_with_type(VoxelTypeInfo { id: VoxelTypeId(1), size_bytes: 1 }),
				Transform::from_xyz(0.25, 0, 0), ChildOf(body),
			)).id();
			let mut tree = OccupancyTree::new();
			tree.add_single_voxels(&[(UVec3::ZERO, 0)]);
			let tile = world.spawn((
				LoadedTile { grid, key: TileKey::new(NonZeroChunkRegion::from_single(IVec3::X), 0, class) },
				DynamicTileData::new(Box::new(OccupancyTileData { tree })),
				GridLocalTransform(IVec3::X * CHUNK_SIZE as i32), ChildOf(grid),
			)).id();
			assert!(world.get::<GlobalTransform>(tile).is_none());
			let min = root_transform * Transform::from_translation(
				FixedVec3::X * (Fixed::from_num(CHUNK_SIZE) + Fixed::from_num(0.25)),
			);
			let origin = min.translation + FixedVec3::Y * scale.to_fixed() / Fixed::from_num(2) - FixedVec3::X;
			let mut query = SystemState::<VoxelWorldQueryParam>::new(&mut world);
			let hit = query.get(&world).unwrap().raycast(origin, FixedVec3::X, Some(Fixed::ONE)).unwrap();
			assert_eq!(hit.grid, grid);
			assert_eq!(hit.voxel_pos, IVec3::X * CHUNK_SIZE as i32);
			assert_eq!(hit.distance, Fixed::ONE);
			assert_eq!(hit.world_position, origin + FixedVec3::X);
			assert_eq!(hit.normal, IVec3::NEG_X);
			assert!(query.get(&world).unwrap().raycast(origin, FixedVec3::X, Some(Fixed::ONE - Fixed::EPSILON)).is_none());
			world.get_mut::<Transform>(body).unwrap().translation += FixedVec3::X;
			let hit = query.get(&world).unwrap().raycast(origin, FixedVec3::X, None).unwrap();
			assert_eq!(hit.distance, Fixed::from_num(2));
		}
	}
}
