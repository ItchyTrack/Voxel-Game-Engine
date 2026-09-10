use std::time::{Duration, Instant};
use bevy::prelude::*;
use voxel_engine::VoxelEngineMode;
use voxel_math::{Fixed, FixedVec3};
use voxel_physics::{FreezePhysics, IsStatic, RigidBody, Velocity};
use voxel_transform::Transform;

#[derive(Resource)]
struct SmokeState {
	start: Instant,
	moved: bool,
	frames: u32,
}

fn main() {
	let runtime = tokio::runtime::Builder::new_multi_thread().enable_time().build().unwrap();
	let _guard = runtime.enter();
	let mut app = voxel_app::build_app_with_mode(Window::default(), VoxelEngineMode::Host);
	app.insert_resource(FreezePhysics(false))
		.insert_resource(SmokeState { start: Instant::now(), moved: false, frames: 0 })
		.add_systems(Update, finish_smoke);
	if std::env::args().any(|arg| arg == "--far") {
		app.add_systems(PostStartup, shift_world);
	}
	app.run();
}

fn shift_world(mut roots: Query<&mut Transform, Without<ChildOf>>) {
	let offset = FixedVec3::new(Fixed::from_num(1i64 << 32), Fixed::from_num(-(1i64 << 32)), Fixed::from_num(1i64 << 31));
	for mut transform in &mut roots { transform.translation += offset; }
}

fn finish_smoke(
	mut state: ResMut<SmokeState>,
	bodies: Query<&Velocity, (With<RigidBody>, Without<IsStatic>)>,
	mut exit: MessageWriter<AppExit>,
) {
	state.frames += 1;
	state.moved |= bodies.iter().any(|velocity| velocity.0 != FixedVec3::ZERO);
	if state.start.elapsed() >= Duration::from_secs(30) {
		assert!(state.frames > 10, "app did not advance frames");
		assert!(state.moved, "dynamic physics never advanced");
		info!(frames = state.frames, "fixed transform smoke passed with active physics");
		exit.write(AppExit::Success);
	}
}
