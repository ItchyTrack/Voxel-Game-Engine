use bevy::ecs::message::MessageWriter;
use bevy::input::ButtonInput;
use bevy::math::{IVec3, Quat, Vec3};
use bevy::prelude::*;
use bevy::transform::components::{GlobalTransform, Transform};

use voxel_data::grid::{Grid, SubGrid};
use voxel_data::voxels::Voxel;

use crate::audio_plugin::PlaySfx;

pub struct WorldInteractionPlugin;

impl Plugin for WorldInteractionPlugin {
	fn build(&self, app: &mut App) {
		app.add_systems(Update, voxel_place_break_system);
	}
}

const PLACE_VOXEL: Voxel = Voxel { color: [180, 180, 180, 255], mass: 100 };

fn voxel_place_break_system(
	keys: Res<ButtonInput<KeyCode>>,
	cameras: Query<(&Camera, &GlobalTransform), With<Camera3d>>,
	mut grids: Query<(Entity, Option<&GlobalTransform>, &mut Grid)>,
	mut sfx: MessageWriter<PlaySfx>,
) {
	let place = keys.just_pressed(KeyCode::Space) || keys.pressed(KeyCode::KeyC);
	let destroy = keys.just_pressed(KeyCode::KeyX) || keys.pressed(KeyCode::KeyZ);
	if !place && !destroy { return; }

	let Some((_, camera_gt)) = cameras.iter().find(|(c, _)| c.is_active) else { return };
	let camera_transform = camera_gt.compute_transform();
	let origin = camera_transform.translation;
	let dir = camera_transform.forward().as_vec3();

	let hit = grids.iter()
		.filter_map(|(entity, grid_gt, grid)| {
			let grid_world = grid_world_transform(grid_gt, &grid);
			raycast_grid(entity, &grid_world, &grid, origin, dir)
		})
		.min_by(|a, b| a.distance.total_cmp(&b.distance));

	let Some(hit) = hit else { return };

	let Ok((_, grid_gt, mut grid)) = grids.get_mut(hit.grid_entity) else { return };
	let grid_world = grid_world_transform(grid_gt, &grid);

	if place {
		let pos = hit.voxel_pos + hit.normal;
		grid.add_voxel(&pos, &PLACE_VOXEL);
		sfx.write(PlaySfx::block_place(grid_world.transform_point(pos.as_vec3() + Vec3::splat(0.5))));
	} else {
		grid.remove_voxel(&hit.voxel_pos);
		sfx.write(PlaySfx::block_break(grid_world.transform_point(hit.voxel_pos.as_vec3() + Vec3::splat(0.5))));
	}
}

struct RaycastHit {
	grid_entity: Entity,
	voxel_pos: IVec3,
	normal: IVec3,
	distance: f32,
}

fn grid_world_transform(grid_gt: Option<&GlobalTransform>, grid: &Grid) -> Transform {
	grid_gt
		.map(|gt| gt.compute_transform() * *grid.transform())
		.unwrap_or(*grid.transform())
}

fn raycast_grid(
	entity: Entity,
	grid_world: &Transform,
	grid: &Grid,
	world_origin: Vec3,
	world_dir: Vec3,
) -> Option<RaycastHit> {
	let inv = grid_world.to_matrix().inverse();
	let grid_origin = inv.transform_point3(world_origin);
	let grid_dir = inv.transform_vector3(world_dir).normalize();

	grid.sub_grids().iter()
		.filter_map(|(_, sub_grid)| {
			let sub_origin = sub_grid.sub_grid_pos().as_vec3();
			raycast_sub_grid(sub_grid, grid_origin - sub_origin, grid_dir)
				.map(|(hit_local, normal_local, distance)| RaycastHit {
					grid_entity: entity,
					voxel_pos: sub_grid.sub_grid_pos() + hit_local.as_ivec3(),
					normal: normal_local.as_ivec3(),
					distance,
				})
		})
		.min_by(|a, b| a.distance.total_cmp(&b.distance))
}

fn raycast_sub_grid(
	sub_grid: &SubGrid,
	origin: Vec3,
	dir: Vec3,
) -> Option<(bevy::math::I16Vec3, bevy::math::I8Vec3, f32)> {
	// GridTree::raycast takes a Transform whose rotation maps +Z to the ray dir.
	let transform = Transform {
		translation: origin,
		rotation: Quat::from_rotation_arc(Vec3::Z, dir),
		scale: Vec3::ONE,
	};
	sub_grid.get_voxels().get_voxels().raycast(&transform, None)
}
