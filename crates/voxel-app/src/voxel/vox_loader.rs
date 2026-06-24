use std::path::Path;

use bevy::math::{IVec3, Quat, Vec3};

use voxel_data::voxels::Voxel;

use crate::voxel::streaming_test::StreamingVoxels;

/// Parses a `.vox` byte buffer and adds its voxels into `grid`, offset by
/// `offset` (in voxels). Returns `false` if the file can't be parsed.
pub fn load_vox_bytes(grid: &mut StreamingVoxels, bytes: &[u8], offset: Vec3) -> bool {
	let Ok(dot_vox_data) = dot_vox::load_bytes(bytes) else { return false };
	grid.reserve(dot_vox_data.models.iter().map(|model| model.voxels.len()).sum());

	#[derive(Clone, Copy)]
	struct Frame {
		translation: Vec3,
		rotation: Quat,
		flip: IVec3,
	}

	let mut stack: Vec<(u32, Frame)> = vec![(0, Frame {
		translation: offset,
		rotation: Quat::IDENTITY,
		flip: IVec3::new(1, 1, -1),
	})];
	while let Some((scene_id, pose)) = stack.pop() {
		let Some(node) = dot_vox_data.scenes.get(scene_id as usize) else { continue };
		match node {
			dot_vox::SceneNode::Transform { frames, child, .. } => {
				let Some(frame) = frames.first() else { continue };
				let pos = frame.position().unwrap_or(dot_vox::Position { x: 0, y: 0, z: 0 });
				let (rot, flip_vec) = frame.orientation().and_then(|q| {
					let (qarr, varr) = q.to_quat_scale();
					let q = Quat::from_array(qarr);
					let q = Quat::from_xyzw(q.x, q.z, -q.y, q.w);
					Some((q, Vec3::from_array(varr).as_ivec3()))
				}).unwrap_or((Quat::IDENTITY, IVec3::ONE));
				stack.push((*child, Frame {
					translation: pose.translation + pose.rotation * Vec3::new(pos.x as f32, pos.z as f32, -pos.y as f32),
					rotation: pose.rotation * rot,
					flip: pose.flip * IVec3::new(flip_vec.x, flip_vec.z, flip_vec.y),
				}));
			}
			dot_vox::SceneNode::Group { children, .. } => {
				for child in children {
					stack.push((*child, pose));
				}
			}
			dot_vox::SceneNode::Shape { models, .. } => {
				for shape_model in models {
					let Some(model) = dot_vox_data.models.get(shape_model.model_id as usize) else { continue };
					let half = Vec3::new(
						model.size.x as f32 / 2.0,
						model.size.z as f32 / 2.0,
						model.size.y as f32 / 2.0,
					).floor();
					let base = pose.translation - pose.rotation * (half * pose.flip.as_vec3());
					let flip_min = pose.flip.min(IVec3::ZERO);
					for voxel in &model.voxels {
						let local = IVec3::new(voxel.x as i32, voxel.z as i32, voxel.y as i32) * pose.flip + flip_min;
						let world_pos = (base + pose.rotation * local.as_vec3()).as_ivec3();
						let palette_entry = dot_vox_data.palette[voxel.i as usize];
						grid.add_voxel(&world_pos, &Voxel {
							color: [palette_entry.r, palette_entry.g, palette_entry.b, palette_entry.a],
							mass: 100,
						});
					}
				}
			}
		}
	}
	true
}

/// Reads a `.vox` file from `path` and adds its voxels into `grid`, offset by
/// `offset` (in voxels). Returns `false` if the file can't be read or parsed.
#[cfg(not(target_arch = "wasm32"))]
pub fn load_vox(grid: &mut StreamingVoxels, path: &Path, offset: Vec3) -> bool {
	let Ok(bytes) = std::fs::read(path) else { return false };
	load_vox_bytes(grid, &bytes, offset)
}
