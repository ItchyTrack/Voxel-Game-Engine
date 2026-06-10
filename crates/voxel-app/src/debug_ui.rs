use bevy::diagnostic::{DiagnosticsStore, FrameTimeDiagnosticsPlugin};
use bevy::prelude::*;
use bevy_egui::{EguiContexts, EguiPlugin, EguiPrimaryContextPass, egui};

use camera_lods::FreezeCameraLods;
use gpu_voxel_data::world_gpu_data::WorldGpuData;
use voxel_data::task_queue::AsyncTaskPriorityQueueResource;
use voxel_physics::{
	CenterOfMass, FreezePhysics, IsStatic, Mass, RigidBody, RotationalInertia,
};
use voxel_renderer::graphics_settings::GraphicsSettings;
use voxel_renderer::hit_count_feedback::RenderStats;
use voxel_streaming::{ChunkLoaderChannel, ChunkRequestChannel, ChunkState, GridStreaming, LodLoaderChannel, LodRequestChannel, CHUNK_SIZE};

#[derive(Resource, Default, Debug, Clone, Copy)]
pub struct InertiaBoxes(pub bool);

#[derive(Resource, Default, Debug, Clone, Copy)]
pub struct ChunkPresenceBoxes(pub bool);

#[derive(Default, Reflect, GizmoConfigGroup)]
struct ChunkGizmos;

pub struct DebugUiPlugin;

impl Plugin for DebugUiPlugin {
	fn build(&self, app: &mut App) {
		if !app.is_plugin_added::<EguiPlugin>() {
			app.add_plugins(EguiPlugin::default());
		}
		if !app.is_plugin_added::<FrameTimeDiagnosticsPlugin>() {
			app.add_plugins(FrameTimeDiagnosticsPlugin::default());
		}
		app.init_resource::<InertiaBoxes>()
			.init_resource::<ChunkPresenceBoxes>()
			.init_gizmo_group::<ChunkGizmos>()
			.add_systems(Startup, |mut store: ResMut<GizmoConfigStore>| {
				store.config_mut::<ChunkGizmos>().0.line.width = 4.0;
			})
			.add_systems(EguiPrimaryContextPass, debug_window)
			.add_systems(Update, draw_inertia_boxes.run_if(|b: Res<InertiaBoxes>| b.0))
			.add_systems(Update, draw_chunk_presence.run_if(|b: Res<ChunkPresenceBoxes>| b.0));
	}
}

fn debug_window(
	mut contexts: EguiContexts,
	diagnostics: Res<DiagnosticsStore>,
	world_gpu_data: Option<Res<WorldGpuData>>,
	render_stats: Res<RenderStats>,
	mut graphics_settings: ResMut<GraphicsSettings>,
	mut freeze_camera_lods: ResMut<FreezeCameraLods>,
	mut freeze_physics: ResMut<FreezePhysics>,
	mut inertia_boxes: ResMut<InertiaBoxes>,
	mut chunk_presence_boxes: ResMut<ChunkPresenceBoxes>,
	chunk_requests: Res<ChunkRequestChannel>,
	chunk_results: Res<ChunkLoaderChannel>,
	lod_requests: Res<LodRequestChannel>,
	lod_results: Res<LodLoaderChannel>,
	async_task_priority_queue: Res<AsyncTaskPriorityQueueResource>,
) -> Result {
	let ctx = contexts.ctx_mut()?;

	let fps = diagnostics
		.get(&FrameTimeDiagnosticsPlugin::FPS)
		.and_then(|d| d.smoothed())
		.unwrap_or(0.0);

	let (tree_kb, voxel_kb) = world_gpu_data
		.as_ref()
		.map(|w| (
			w.packed_64_tree_dynamic_buffer.held_bytes() / 1000,
			w.packed_voxel_data_dynamic_buffer.held_bytes() / 1000,
		))
		.unwrap_or((0, 0));

	let (bvh_kb, bvh_leaf_kb) = render_stats
		.inner
		.lock()
		.map(|s| (s.bvh_bytes / 1000, s.bvh_leaf_bytes / 1000))
		.unwrap_or((0, 0));

	let chunk_sent = chunk_requests.sent_count();
	let chunk_received = chunk_results.received_count();
	let lod_sent = lod_requests.sent_count();
	let lod_received = lod_results.received_count();
	let async_queue_len = async_task_priority_queue.len();

	egui::Window::new("Debug")
		.default_pos([0.0, 0.0])
		.default_size([175.0, 260.0])
		.show(ctx, |ui| {
			ui.label(format!("FPS: {:.2}", fps));
			ui.separator();
			ui.label(format!("64 tree bytes: {}KB", tree_kb));
			ui.label(format!("Voxel bytes: {}KB", voxel_kb));
			ui.label(format!("BVH bytes: {}KB", bvh_kb));
			ui.label(format!("BVH leaf bytes: {}KB", bvh_leaf_kb));
			ui.separator();
			ui.label("Streaming");
			ui.label(format!("Chunks sent/received: {}/{}", chunk_sent, chunk_received));
			ui.label(format!("Chunks active: {}", chunk_sent.saturating_sub(chunk_received)));
			ui.label(format!("LODs sent/received: {}/{}", lod_sent, lod_received));
			ui.label(format!("LODs active: {}", lod_sent.saturating_sub(lod_received)));
			ui.label(format!("Async priority queue: {}", async_queue_len));
			ui.separator();
			ui.label("Graphics");
			ui.checkbox(&mut graphics_settings.shadows, "shadows");
			ui.separator();
			ui.label("Debug");
			ui.checkbox(&mut freeze_camera_lods.0, "freeze camera LODs");
			ui.checkbox(&mut freeze_physics.0, "freeze physics");
			ui.checkbox(&mut inertia_boxes.0, "inertia boxes");
			ui.checkbox(&mut chunk_presence_boxes.0, "chunk presence");
		});

	Ok(())
}

fn draw_inertia_boxes(
	mut gizmos: Gizmos,
	bodies: Query<
		(&GlobalTransform, &Mass, &CenterOfMass, &RotationalInertia),
		(With<RigidBody>, Without<IsStatic>),
	>,
) {
	use nalgebra::{Matrix3, SymmetricEigen};

	let color = Color::srgba(1.0, 0.2, 0.2, 0.6);

	for (gt, mass, com, inertia) in bodies.iter() {
		if mass.0 <= 0.0 { continue; }

		let body_t = gt.compute_transform();
		let world_inertia = inertia.0.get_rotated(body_t.rotation.as_dquat());
		let m = world_inertia.mat;

		// Build a column-major Matrix3 from the (symmetric) inertia tensor.
		let nm = Matrix3::new(
			m.x_axis.x as f32, m.x_axis.y as f32, m.x_axis.z as f32,
			m.y_axis.x as f32, m.y_axis.y as f32, m.y_axis.z as f32,
			m.z_axis.x as f32, m.z_axis.y as f32, m.z_axis.z as f32,
		);
		let eigen = SymmetricEigen::new(nm);
		let (e0, e1, e2) = (eigen.eigenvalues.x, eigen.eigenvalues.y, eigen.eigenvalues.z);
		let v0 = Vec3::new(eigen.eigenvectors[(0, 0)], eigen.eigenvectors[(1, 0)], eigen.eigenvectors[(2, 0)]);
		let v1 = Vec3::new(eigen.eigenvectors[(0, 1)], eigen.eigenvectors[(1, 1)], eigen.eigenvectors[(2, 1)]);
		let v2 = Vec3::new(eigen.eigenvectors[(0, 2)], eigen.eigenvectors[(1, 2)], eigen.eigenvectors[(2, 2)]);

		let s0 = v0.normalize_or_zero() * ((6.0 / mass.0) * (e1 + e2 - e0)).max(0.0).sqrt();
		let s1 = v1.normalize_or_zero() * ((6.0 / mass.0) * (e0 + e2 - e1)).max(0.0).sqrt();
		let s2 = v2.normalize_or_zero() * ((6.0 / mass.0) * (e0 + e1 - e2)).max(0.0).sqrt();

		// Corner of the box (matches main: pos + (s0+s1+s2)/-2).
		let world_com = body_t.transform_point(com.0);
		let origin = world_com - (s0 + s1 + s2) * 0.5;

		let p000 = origin;
		let p100 = origin + s0;
		let p010 = origin + s1;
		let p001 = origin + s2;
		let p110 = origin + s0 + s1;
		let p101 = origin + s0 + s2;
		let p011 = origin + s1 + s2;
		let p111 = origin + s0 + s1 + s2;

		// 12 edges along the three basis directions.
		for (a, b) in [
			(p000, p100), (p010, p110), (p001, p101), (p011, p111),
			(p000, p010), (p100, p110), (p001, p011), (p101, p111),
			(p000, p001), (p100, p101), (p010, p011), (p110, p111),
		] {
			gizmos.line(a, b, color);
		}
	}
}

fn chunk_state_color(state: ChunkState) -> Color {
	match state {
		ChunkState::Available => Color::srgb(0.2, 0.2, 0.2),
		ChunkState::InFlight => Color::srgb(0.9, 0.9, 0.1),
		ChunkState::Loaded => Color::srgb(0.1, 0.8, 0.1),
		ChunkState::InternalDirty => Color::srgb(0.9, 0.9, 0.9),
		ChunkState::ExternalDirty => Color::srgb(0.9, 0.1, 0.1),
		ChunkState::ExternalDirtyInFlight => Color::srgb(0.9, 0.1, 0.9),
	}
}

fn draw_chunk_presence(
	mut gizmos: Gizmos<ChunkGizmos>,
	grids: Query<(&GlobalTransform, &GridStreaming)>,
) {
	const INSET: f32 = 0.75;
	for (gt, streaming) in grids.iter() {
		for (origin, size, state) in streaming.presence().iter_states() {
			let lo = (origin * CHUNK_SIZE).as_vec3() + Vec3::splat(INSET);
			let hi = ((origin + IVec3::splat(size as i32)) * CHUNK_SIZE).as_vec3() - Vec3::splat(INSET);
			let color = chunk_state_color(state);

			let corner = |x: f32, y: f32, z: f32| gt.transform_point(Vec3::new(x, y, z));
			let c000 = corner(lo.x, lo.y, lo.z);
			let c100 = corner(hi.x, lo.y, lo.z);
			let c010 = corner(lo.x, hi.y, lo.z);
			let c001 = corner(lo.x, lo.y, hi.z);
			let c110 = corner(hi.x, hi.y, lo.z);
			let c101 = corner(hi.x, lo.y, hi.z);
			let c011 = corner(lo.x, hi.y, hi.z);
			let c111 = corner(hi.x, hi.y, hi.z);

			for (a, b) in [
				(c000, c100), (c010, c110), (c001, c101), (c011, c111),
				(c000, c010), (c100, c110), (c001, c011), (c101, c111),
				(c000, c001), (c100, c101), (c010, c011), (c110, c111),
			] {
				gizmos.line(a, b, color);
			}
		}
	}
}
