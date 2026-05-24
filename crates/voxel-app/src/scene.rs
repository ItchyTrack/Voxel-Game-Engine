use bevy::math::{IVec3, Quat, Vec3};
use bevy::prelude::*;

use voxel_data::grid::Grid;
use voxel_data::voxels::Voxel;
use voxel_physics::{AngularVelocity, IsStatic, Mass, RigidBody, Velocity};

pub struct ScenePlugin;

impl Plugin for ScenePlugin {
	fn build(&self, app: &mut App) {
		app.add_systems(Startup, setup_scene);
	}
}

fn setup_scene(mut commands: Commands) {
	let mut ball = Grid::new(&Transform::IDENTITY);
	make_ball(&mut ball, 16);
	commands
		.spawn((
			RigidBody,
			Velocity::default(),
			AngularVelocity::default(),
			Mass(1000.0),
			Transform::from_translation(Vec3::new(0.0, 50.0, 0.0)),
		))
		.with_child((Transform::IDENTITY, ball));

	let candidate_paths = [
		std::path::PathBuf::from("res/Church_Of_St_Sophia.vox"),
		std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
			.join("../../res/Church_Of_St_Sophia.vox"),
	];
	let Some(bytes) = candidate_paths.iter().find_map(|p| std::fs::read(p).ok()) else {
		log::warn!("voxel scene: could not find res/Church_Of_St_Sophia.vox");
		return;
	};
	let dot_vox_data = match dot_vox::load_bytes(&bytes) {
		Ok(d) => d,
		Err(err) => { log::error!("voxel scene: dot_vox parse error: {err}"); return; }
	};

	let mut grid = Grid::new(&Transform::IDENTITY);

	#[derive(Clone, Copy)]
	struct Frame { translation: Vec3, rotation: Quat, flip: IVec3 }
	let mut stack: Vec<(u32, Frame)> = vec![(0, Frame {
		translation: Vec3::ZERO,
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
				for child in children { stack.push((*child, pose)); }
			}
			dot_vox::SceneNode::Shape { models, .. } => {
				for shape_model in models {
					let Some(model) = dot_vox_data.models.get(shape_model.model_id as usize) else { continue };
					let size = Vec3::new(model.size.x as f32, model.size.z as f32, model.size.y as f32);
					let half = (size / 2.0).floor();
					let pose_transform = Transform { translation: pose.translation, rotation: pose.rotation, scale: Vec3::ONE };
					let half_offset = Transform::from_translation(-half * pose.flip.as_vec3());
					for voxel in &model.voxels {
						let local = IVec3::new(voxel.x as i32, voxel.z as i32, voxel.y as i32) * pose.flip + pose.flip.min(IVec3::ZERO);
						let world_pos = (pose_transform * half_offset).transform_point(local.as_vec3()).as_ivec3();
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

	log::info!("voxel scene: loaded church with {} sub-grids", grid.sub_grids().iter().count());
	commands
		.spawn((
			RigidBody,
			Velocity::default(),
			AngularVelocity::default(),
			Mass(0.0),
			IsStatic,
			Transform::from_translation(Vec3::new(0.0, -350.0, 0.0)),
		))
		.with_child((Transform::IDENTITY, grid));
}

fn make_ball(grid: &mut Grid, radius: i32) {
	let rsq = (radius as f32 - 0.5).powi(2);
	let shade = |v: i32| -> u8 {
		let t = ((v + radius) as f32 / (2 * radius).max(1) as f32 * 5.0) as u8;
		40 + t * 40
	};
	for x in -radius..=radius {
		for y in -radius..=radius {
			for z in -radius..=radius {
				let p = IVec3::new(x, y, z);
				if p.as_vec3().length_squared() > rsq { continue; }
				grid.add_voxel(&p, &Voxel { color: [shade(x), shade(y), shade(z), 255], mass: 100 });
			}
		}
	}
}
