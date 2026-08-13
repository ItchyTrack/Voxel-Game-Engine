use bevy::diagnostic::{DiagnosticsStore, FrameTimeDiagnosticsPlugin};
use bevy::prelude::*;
use bevy_egui::{EguiContexts, EguiPlugin, EguiPrimaryContextPass, egui};

use camera_voxel_loader::{CameraVoxelLoader, CoverageDebugState, FreezeCameraVoxelLoader};
use voxel_tasks::AsyncTaskPriorityQueueResource;
use voxel_physics::{
	BallJoint, CenterOfMass, FreezePhysics, IsStatic, Mass, RigidBody, RotationalInertia,
};
use crate::voxel::rendering::VoxelRenderMode;
use voxel_ray_renderer::{gpu_data::RayWorldGpuData, graphics_settings::GraphicsSettings};
use voxel_raster_renderer::gpu_data::RasterWorldGpuData;
use voxel_ray_renderer::direction_feedback::RenderStats;
use tile_data::CHUNK_SIZE;
use voxel_streaming::{ChunkState, GridStreaming};

#[derive(Resource, Default, Debug, Clone, Copy)]
pub struct InertiaBoxes(pub bool);

#[derive(Resource, Default, Debug, Clone, Copy)]
pub struct ChunkPresenceBoxes(pub bool);

#[derive(Resource, Default, Debug, Clone, Copy)]
pub struct CoverageBoxes(pub bool);

#[derive(Resource, Default, Debug, Clone, Copy)]
pub struct ConstraintDebugRender(pub bool);


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
			.init_resource::<CoverageBoxes>()
			.init_resource::<ConstraintDebugRender>()
			.init_gizmo_group::<ChunkGizmos>()
			.add_systems(Startup, |mut store: ResMut<GizmoConfigStore>| {
				store.config_mut::<ChunkGizmos>().0.line.width = 4.0;
			})
			.add_systems(EguiPrimaryContextPass, debug_window)
			.add_systems(Update, draw_inertia_boxes.run_if(|b: Res<InertiaBoxes>| b.0))
			.add_systems(Update, draw_ball_joint_constraints.run_if(|b: Res<ConstraintDebugRender>| b.0))
			.add_systems(Update, draw_chunk_presence.run_if(|b: Res<ChunkPresenceBoxes>| b.0))
			.add_systems(Update, draw_coverage.run_if(|b: Res<CoverageBoxes>| b.0));
	}
}

fn debug_window(
	mut contexts: EguiContexts,
	diagnostics: Res<DiagnosticsStore>,
	ray_gpu_data: Option<Res<RayWorldGpuData>>,
	raster_gpu_data: Option<Res<RasterWorldGpuData>>,
	render_stats: Res<RenderStats>,
	mut graphics_settings: ResMut<GraphicsSettings>,
	mut freeze_camera_voxel_loader: ResMut<FreezeCameraVoxelLoader>,
	mut freeze_physics: ResMut<FreezePhysics>,
	mut inertia_boxes: ResMut<InertiaBoxes>,
	mut constraint_debug_render: ResMut<ConstraintDebugRender>,
	mut chunk_presence_boxes: ResMut<ChunkPresenceBoxes>,
	mut coverage_boxes: ResMut<CoverageBoxes>,
	mut voxel_render_mode: ResMut<VoxelRenderMode>,
	async_task_priority_queue: Res<AsyncTaskPriorityQueueResource>,
) -> Result {
	let ctx = contexts.ctx_mut()?;

	let fps = diagnostics
		.get(&FrameTimeDiagnosticsPlugin::FPS)
		.and_then(|d| d.smoothed())
		.unwrap_or(0.0);

	let (tree_kb, voxel_kb) = ray_gpu_data
		.as_ref()
		.map(|gpu| { let gpu = gpu.lock(); (gpu.trees.held_bytes() / 1000, gpu.voxels.held_bytes() / 1000) })
		.unwrap_or_default();
	let raster_kb = raster_gpu_data.as_ref().map(|gpu| gpu.lock().faces.held_bytes() / 1000).unwrap_or_default();

	let (bvh_kb, bvh_leaf_kb) = render_stats
		.inner
		.lock()
		.map(|s| (s.bvh_bytes / 1000, s.bvh_leaf_bytes / 1000))
		.unwrap_or((0, 0));

	let async_queue_len = async_task_priority_queue.len();
	let mut render_mode = *voxel_render_mode;

	egui::Window::new("Debug")
		.default_pos([0.0, 0.0])
		.default_size([175.0, 260.0])
		.show(ctx, |ui| {
			ui.label(format!("FPS: {:.2}", fps));
			ui.separator();
			ui.label(format!("64 tree bytes: {}KB", tree_kb));
			ui.label(format!("Voxel bytes: {}KB", voxel_kb));
			ui.label(format!("Raster face bytes: {}KB", raster_kb));
			ui.label(format!("BVH bytes: {}KB", bvh_kb));
			ui.label(format!("BVH leaf bytes: {}KB", bvh_leaf_kb));
			ui.separator();
			ui.label("Streaming");
			ui.label(format!("Async priority queue: {}", async_queue_len));
			ui.separator();
			ui.label("Graphics");
			ui.checkbox(&mut graphics_settings.shadows, "shadows");
			ui.horizontal(|ui| {
				ui.label("grid renderer");
				ui.selectable_value(&mut render_mode, VoxelRenderMode::Ray, "ray");
				ui.selectable_value(&mut render_mode, VoxelRenderMode::Raster, "raster");
			});
			ui.separator();
			ui.label("Debug");
			ui.checkbox(&mut freeze_camera_voxel_loader.0, "freeze camera voxel loader");
			ui.checkbox(&mut freeze_physics.0, "freeze physics");
			ui.checkbox(&mut inertia_boxes.0, "inertia boxes");
			ui.checkbox(&mut constraint_debug_render.0, "constraints");
			ui.checkbox(&mut chunk_presence_boxes.0, "chunk presence");
			ui.checkbox(&mut coverage_boxes.0, "camera coverage");
		});

	if *voxel_render_mode != render_mode { *voxel_render_mode = render_mode; }

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

fn draw_ball_joint_constraints(
	mut gizmos: Gizmos,
	joints: Query<&BallJoint>,
	bodies: Query<&GlobalTransform>,
) {
	const HALF_EXTENT: f32 = 0.75;
	let color = Color::srgba(0.2, 0.8, 1.0, 0.9);

	for joint in joints.iter() {
		let Ok(body_1_gt) = bodies.get(joint.body_1) else { continue; };
		let Ok(body_2_gt) = bodies.get(joint.body_2) else { continue; };

		let attachment_1 = body_1_gt.transform_point(joint.body_1_attachment.translation);
		let attachment_2 = body_2_gt.transform_point(joint.body_2_attachment.translation);
		let center = (attachment_1 + attachment_2) * 0.5;

		draw_axis_aligned_box_edges(&mut gizmos, center - Vec3::splat(HALF_EXTENT), center + Vec3::splat(HALF_EXTENT), color);
	}
}

fn draw_axis_aligned_box_edges<C: GizmoConfigGroup>(gizmos: &mut Gizmos<C>, lo: Vec3, hi: Vec3, color: Color) {
	let c000 = Vec3::new(lo.x, lo.y, lo.z);
	let c100 = Vec3::new(hi.x, lo.y, lo.z);
	let c010 = Vec3::new(lo.x, hi.y, lo.z);
	let c001 = Vec3::new(lo.x, lo.y, hi.z);
	let c110 = Vec3::new(hi.x, hi.y, lo.z);
	let c101 = Vec3::new(hi.x, lo.y, hi.z);
	let c011 = Vec3::new(lo.x, hi.y, hi.z);
	let c111 = Vec3::new(hi.x, hi.y, hi.z);

	for (a, b) in [
		(c000, c100), (c010, c110), (c001, c101), (c011, c111),
		(c000, c010), (c100, c110), (c001, c011), (c101, c111),
		(c000, c001), (c100, c101), (c010, c011), (c110, c111),
	] {
		gizmos.line(a, b, color);
	}
}

fn draw_box_edges<C: GizmoConfigGroup>(gizmos: &mut Gizmos<C>, gt: &GlobalTransform, lo: Vec3, hi: Vec3, color: Color) {
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

fn chunk_state_color(state: ChunkState) -> Color {
	match state {
		ChunkState::Available => Color::srgb(0.2, 0.2, 0.2),
		ChunkState::InFlight => Color::srgb(0.9, 0.9, 0.1),
		ChunkState::Loaded => Color::srgb(0.1, 0.8, 0.1),
		ChunkState::InternalDirty => Color::srgb(0.9, 0.9, 0.9),
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
			draw_box_edges(&mut gizmos, gt, lo, hi, chunk_state_color(state));
		}
	}
}

fn draw_coverage(
	mut gizmos: Gizmos<ChunkGizmos>,
	loaders: Query<&CameraVoxelLoader>,
	transforms: Query<&GlobalTransform>,
) {
	const INSET: f32 = 1.5;
	for loader in &loaders {
		for tile in loader.coverage_debug_tiles() {
			let Ok(gt) = transforms.get(tile.grid) else { continue };
			let lo = (tile.min * CHUNK_SIZE).as_vec3() + Vec3::splat(INSET);
			let hi = ((tile.min + tile.size) * CHUNK_SIZE).as_vec3() - Vec3::splat(INSET);
			let color = match tile.state {
				CoverageDebugState::Pending => chunk_state_color(ChunkState::InFlight),
				CoverageDebugState::Loaded => chunk_state_color(ChunkState::Loaded),
				CoverageDebugState::Empty => chunk_state_color(ChunkState::Available),
				CoverageDebugState::Waiting => Color::srgb(0.9, 0.1, 0.1),
			};
			draw_box_edges(&mut gizmos, gt, lo, hi, color);
		}
	}
}
