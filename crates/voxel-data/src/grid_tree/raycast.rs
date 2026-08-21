use bevy::math::{I8Vec3, UVec3, Vec3};
use bevy::transform::components::Transform;

use super::{CellKind, GridTreeView, GridType, child_size, get_child_contents_index, size};

fn ray_aabb_intersection(start: &Vec3, direction: &Vec3, aabb: &(Vec3, Vec3)) -> Option<f32> {
	let (min, max) = aabb;
	if start.cmpge(*min).all() && start.cmple(*max).all() { return Some(0.0); }
	let inv = Vec3::ONE / *direction;
	let t1 = (*min - *start) * inv;
	let t2 = (*max - *start) * inv;
	let tmin = t1.min(t2).max_element();
	let tmax = t1.max(t2).min_element();
	if tmax < 0.0 || tmin > tmax { return None; }
	Some(tmin)
}

pub fn raycast<G: GridType>(view: GridTreeView<'_, G>, transform: &Transform, max_length: Option<f32>) -> Option<(UVec3, I8Vec3, f32)> {
	let max_length = max_length.unwrap_or(f32::MAX);
	let raw = view.raw();
	let root_pos = view.root_pos();
	let root_depth = view.root_depth();

	let origin = transform.translation;
	let dir = transform.rotation * Vec3::Z;
	let root_min = root_pos.as_vec3();
	let root_max = root_min + Vec3::splat(size(root_depth) as f32);
	let distance_to_aabb = ray_aabb_intersection(&origin, &dir, &(root_min, root_max))?;
	let post_aabb_origin_pre_shift = origin + dir * distance_to_aabb;
	let post_aabb_origin = post_aabb_origin_pre_shift.min(root_pos.as_vec3() + Vec3::splat((size(root_depth) as f32) - 0.00001)).max(root_pos.as_vec3());
	let post_aabb_origin = post_aabb_origin.move_towards(post_aabb_origin.floor() + 0.5, 0.001);
	let root_relative_post_aabb_origin = post_aabb_origin - root_pos.as_vec3();
	let delta = dir.recip().abs();
	let step = dir.signum().as_i8vec3();
	let mut axis_distances = dir.recip()
		* Vec3::new(
			if step.x > 0 { root_relative_post_aabb_origin.x.ceil() } else { root_relative_post_aabb_origin.x.floor() } - root_relative_post_aabb_origin.x,
			if step.y > 0 { root_relative_post_aabb_origin.y.ceil() } else { root_relative_post_aabb_origin.y.floor() } - root_relative_post_aabb_origin.y,
			if step.z > 0 { root_relative_post_aabb_origin.z.ceil() } else { root_relative_post_aabb_origin.z.floor() } - root_relative_post_aabb_origin.z,
		)
		+ distance_to_aabb;
	let mut root_relative_grid_pos = root_relative_post_aabb_origin.as_uvec3();
	let mut last_step_axis = (post_aabb_origin_pre_shift - post_aabb_origin).abs().max_position() as u8;
	let mut current_node_index = 0u32;
	let mut current_depth = root_depth;
	let mut last_distance = distance_to_aabb;
	if max_length < last_distance { return None; }
	loop {
		let node_relative_grid_pos = root_relative_grid_pos % size(current_depth);
		let contents_pos = (node_relative_grid_pos / child_size(current_depth)).as_u8vec3();
		let contents_index = get_child_contents_index(contents_pos);
		match raw.cell_kind(current_node_index, contents_index) {
			CellKind::Empty => {
				if child_size(current_depth) != 1 {
					let node_cell_relative_grid_pos = node_relative_grid_pos % child_size(current_depth);
					let mut step_amount = UVec3::select(
						step.cmpgt(I8Vec3::ZERO),
						UVec3::splat(child_size(current_depth) - 1) - node_cell_relative_grid_pos,
						node_cell_relative_grid_pos,
					);
					let distance_to_edge_of_cell = axis_distances + step_amount.as_vec3() * delta;
					match distance_to_edge_of_cell.min_position() {
						0 => { step_amount.y = ((distance_to_edge_of_cell.x - axis_distances.y + delta.y) / delta.y).abs() as u32; step_amount.z = ((distance_to_edge_of_cell.x - axis_distances.z + delta.z) / delta.z).abs() as u32; }
						1 => { step_amount.x = ((distance_to_edge_of_cell.y - axis_distances.x + delta.x) / delta.x).abs() as u32; step_amount.z = ((distance_to_edge_of_cell.y - axis_distances.z + delta.z) / delta.z).abs() as u32; }
						2 => { step_amount.x = ((distance_to_edge_of_cell.z - axis_distances.x + delta.x) / delta.x).abs() as u32; step_amount.y = ((distance_to_edge_of_cell.z - axis_distances.y + delta.y) / delta.y).abs() as u32; }
						_ => unreachable!(),
					}
					axis_distances += delta * step_amount.as_vec3();
					root_relative_grid_pos = (root_relative_grid_pos.as_ivec3() + step_amount.as_ivec3() * step.as_ivec3()).as_uvec3();
				}
				match axis_distances.min_position() {
					0 => {
						if max_length < axis_distances.x { return None; }
						let next = root_relative_grid_pos.x as i64 + step.x as i64;
						if next < 0 || next >= size(root_depth) as i64 { return None; }
						let next = next as u32;
						loop {
							if root_relative_grid_pos.x / size(current_depth) == next / size(current_depth) { break; }
							let parent_offset = raw.parent_offset(current_node_index);
							if parent_offset == 0 { return None; }
							current_depth += 1;
							current_node_index -= parent_offset;
						}
						root_relative_grid_pos.x = next; last_distance = axis_distances.x; axis_distances.x += delta.x; last_step_axis = 0;
					}
					1 => {
						if max_length < axis_distances.y { return None; }
						let next = root_relative_grid_pos.y as i64 + step.y as i64;
						if next < 0 || next >= size(root_depth) as i64 { return None; }
						let next = next as u32;
						loop {
							if root_relative_grid_pos.y / size(current_depth) == next / size(current_depth) { break; }
							let parent_offset = raw.parent_offset(current_node_index);
							if parent_offset == 0 { return None; }
							current_depth += 1;
							current_node_index -= parent_offset;
						}
						root_relative_grid_pos.y = next; last_distance = axis_distances.y; axis_distances.y += delta.y; last_step_axis = 1;
					}
					2 => {
						if max_length < axis_distances.z { return None; }
						let next = root_relative_grid_pos.z as i64 + step.z as i64;
						if next < 0 || next >= size(root_depth) as i64 { return None; }
						let next = next as u32;
						loop {
							if root_relative_grid_pos.z / size(current_depth) == next / size(current_depth) { break; }
							let parent_offset = raw.parent_offset(current_node_index);
							if parent_offset == 0 { return None; }
							current_depth += 1;
							current_node_index -= parent_offset;
						}
						root_relative_grid_pos.z = next; last_distance = axis_distances.z; axis_distances.z += delta.z; last_step_axis = 2;
					}
					_ => unreachable!(),
				}
			}
			CellKind::Data => return Some((root_relative_grid_pos + root_pos, -step.to_array()[last_step_axis as usize] * I8Vec3::AXES[last_step_axis as usize], last_distance)),
			CellKind::Node => { current_depth -= 1; current_node_index = raw.child_index(current_node_index, contents_index); }
		}
	}
}
