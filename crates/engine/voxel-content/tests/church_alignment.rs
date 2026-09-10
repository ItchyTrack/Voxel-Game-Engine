use std::collections::HashSet;
use std::path::PathBuf;

use bevy::math::{IVec3, Quat, UVec3};
use voxel_math::{Fixed, FixedVec3};
use tile_data::chunk_of;
use tile_data::CHUNK_SIZE;

#[derive(Clone, Copy)]
struct Frame {
	translation: FixedVec3,
	rotation: Quat,
	flip: IVec3,
}

fn church_path() -> PathBuf {
	PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../../res/Church_Of_St_Sophia.vox")
}

fn load_chunks(round_positions: bool) -> HashSet<IVec3> {
	let bytes = std::fs::read(church_path()).expect("read church vox");
	let data = dot_vox::load_bytes(&bytes).expect("parse church vox");
	let mut chunks = HashSet::new();
	let mut stack = vec![(0u32, Frame { translation: FixedVec3::ZERO, rotation: Quat::IDENTITY, flip: IVec3::new(1, 1, -1) })];
	while let Some((scene_id, pose)) = stack.pop() {
		let Some(node) = data.scenes.get(scene_id as usize) else { continue };
		match node {
			dot_vox::SceneNode::Transform { frames, child, .. } => {
				let Some(frame) = frames.first() else { continue };
				let pos = frame.position().unwrap_or(dot_vox::Position { x: 0, y: 0, z: 0 });
				let (rot, flip_vec) = frame.orientation().map(|q| {
					let (qarr, varr) = q.to_quat_scale();
					let q = Quat::from_array(qarr);
					(Quat::from_xyzw(q.x, q.z, -q.y, q.w), IVec3::new(varr[0] as i32, varr[1] as i32, varr[2] as i32))
				}).unwrap_or((Quat::IDENTITY, IVec3::ONE));
				stack.push((*child, Frame {
					translation: pose.translation + pose.rotation * FixedVec3::new(Fixed::from_num(pos.x), Fixed::from_num(pos.z), -Fixed::from_num(pos.y)),
					rotation: pose.rotation * rot,
					flip: pose.flip * IVec3::new(flip_vec.x, flip_vec.z, flip_vec.y),
				}));
			}
			dot_vox::SceneNode::Group { children, .. } => {
				for child in children { stack.push((*child, pose)); }
			}
			dot_vox::SceneNode::Shape { models, .. } => {
				for shape_model in models {
					let Some(model) = data.models.get(shape_model.model_id as usize) else { continue };
					let half = FixedVec3::from(UVec3::new(model.size.x / 2, model.size.z / 2, model.size.y / 2));
					let base = pose.translation - pose.rotation * (half * FixedVec3::from(pose.flip));
					let flip_min = pose.flip.min(IVec3::ZERO);
					for voxel in &model.voxels {
						let local = IVec3::new(voxel.x as i32, voxel.z as i32, voxel.y as i32) * pose.flip + flip_min;
						let p = base + pose.rotation * FixedVec3::from(local);
						let world = if round_positions { p.round().as_ivec3() } else { p.as_ivec3() };
						chunks.insert(chunk_of(world));
					}
				}
			}
		}
	}
	chunks
}

#[test]
fn church_chunk_set_changes_if_transform_positions_are_rounded_instead_of_truncated() {
	let truncated = load_chunks(false);
	let rounded = load_chunks(true);
	assert_eq!(truncated, rounded, "church chunk coverage differs: only_trunc={}, only_round={}, chunk_size={CHUNK_SIZE}", truncated.difference(&rounded).count(), rounded.difference(&truncated).count());
}

#[test]
fn church_vox_source_chunk_set_matches_scene_trace() {
	let bytes = std::fs::read(church_path()).expect("read church vox");
	let data = dot_vox::load_bytes(&bytes).expect("parse church vox");
	let mut chunk_points = HashSet::new();
	let mut stack = vec![(0u32, Frame { translation: FixedVec3::ZERO, rotation: Quat::IDENTITY, flip: IVec3::new(1, 1, -1) })];
	while let Some((scene_id, pose)) = stack.pop() {
		let Some(node) = data.scenes.get(scene_id as usize) else { continue };
		match node {
			dot_vox::SceneNode::Transform { frames, child, .. } => {
				let Some(frame) = frames.first() else { continue };
				let pos = frame.position().unwrap_or(dot_vox::Position { x: 0, y: 0, z: 0 });
				let (rot, flip_vec) = frame.orientation().map(|q| {
					let (qarr, varr) = q.to_quat_scale();
					let q = Quat::from_array(qarr);
					(Quat::from_xyzw(q.x, q.z, -q.y, q.w), IVec3::new(varr[0] as i32, varr[1] as i32, varr[2] as i32))
				}).unwrap_or((Quat::IDENTITY, IVec3::ONE));
				stack.push((*child, Frame {
					translation: pose.translation + pose.rotation * FixedVec3::new(Fixed::from_num(pos.x), Fixed::from_num(pos.z), -Fixed::from_num(pos.y)),
					rotation: pose.rotation * rot,
					flip: pose.flip * IVec3::new(flip_vec.x, flip_vec.z, flip_vec.y),
				}));
			}
			dot_vox::SceneNode::Group { children, .. } => {
				for child in children { stack.push((*child, pose)); }
			}
			dot_vox::SceneNode::Shape { models, .. } => {
				for shape_model in models {
					let Some(model) = data.models.get(shape_model.model_id as usize) else { continue };
					let half = FixedVec3::from(UVec3::new(model.size.x / 2, model.size.z / 2, model.size.y / 2));
					let base = pose.translation - pose.rotation * (half * FixedVec3::from(pose.flip));
					let flip_min = pose.flip.min(IVec3::ZERO);
					for voxel in &model.voxels {
						let local = IVec3::new(voxel.x as i32, voxel.z as i32, voxel.y as i32) * pose.flip + flip_min;
						let source_pos = (base + pose.rotation * FixedVec3::from(local)).as_ivec3();
						chunk_points.insert(chunk_of(source_pos));
					}
				}
			}
		}
	}

	let traced = load_chunks(false);
	assert_eq!(chunk_points, traced);
}
