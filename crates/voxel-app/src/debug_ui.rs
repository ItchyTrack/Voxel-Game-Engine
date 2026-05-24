use bevy::diagnostic::{DiagnosticsStore, FrameTimeDiagnosticsPlugin};
use bevy::prelude::*;
use bevy_egui::input::EguiWantsInput;
use bevy_egui::{EguiContexts, EguiPlugin, EguiPrimaryContextPass, egui};

use voxel_data::world_gpu_data::WorldGpuData;
use voxel_physics::{
	CenterOfMass, FreezePhysics, IsStatic, Mass, RigidBody, RotationalInertia,
};
use voxel_renderer::graphics_settings::GraphicsSettings;
use voxel_renderer::hit_count_feedback::RenderStats;
use voxel_renderer::scene::FreezeUploads;

#[derive(Resource, Default, Debug, Clone, Copy)]
pub struct InertiaBoxes(pub bool);

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
			.add_systems(EguiPrimaryContextPass, debug_window)
			.add_systems(Update, draw_inertia_boxes.run_if(|b: Res<InertiaBoxes>| b.0));
	}
}

fn debug_window(
	mut contexts: EguiContexts,
	diagnostics: Res<DiagnosticsStore>,
	world_gpu_data: Option<Res<WorldGpuData>>,
	render_stats: Res<RenderStats>,
	mut graphics_settings: ResMut<GraphicsSettings>,
	mut freeze_uploads: ResMut<FreezeUploads>,
	mut freeze_physics: ResMut<FreezePhysics>,
	mut inertia_boxes: ResMut<InertiaBoxes>,
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
			ui.label("Graphics");
			ui.checkbox(&mut graphics_settings.shadows, "shadows");
			ui.separator();
			ui.label("Debug");
			ui.checkbox(&mut freeze_uploads.0, "freeze upload");
			ui.checkbox(&mut freeze_physics.0, "freeze physics");
			ui.checkbox(&mut inertia_boxes.0, "inertia boxes");
		});

	Ok(())
}

/// Run conditions for game input systems so egui captures the click/keypress
/// when the cursor is over (or focused on) the debug window.
pub fn egui_blocks_pointer(egui_wants: Res<EguiWantsInput>) -> bool {
	egui_wants.wants_any_pointer_input()
}

pub fn egui_blocks_keyboard(egui_wants: Res<EguiWantsInput>) -> bool {
	egui_wants.wants_any_keyboard_input()
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
