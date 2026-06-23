use std::sync::{Arc, OnceLock};

use bevy::math::{I16Vec3, IVec3, Vec3};
use bevy::prelude::*;

use voxel_data::grid::Grid;
use voxel_data::voxels::{Voxel, Voxels};
use voxel_edit::GridEdits;
use voxel_physics::{IsStatic, RigidBody};
use voxel_physics::components::VoxelCollider;
use voxel_data::grid::GridId;
use voxel_sources::{ChunkSource, SourceHandle, VoxelSourcesAppExt};
use voxel_streaming::{chunk_origin, GridStreaming, CHUNK_SIZE};
use voxel_lightyear::ReplicateVoxels;

use crate::voxel::streaming_test::WorldStore;

const RADIUS: i32 = 2_000;
const COST: u32 = 5;

/// Grid-local origin sits at the sphere centre, placed a full radius below the
/// church base (~y = -350) so the church ends up sitting on the sphere's top.
const CENTER: Vec3 = Vec3::new(0.0, -350.0 - RADIUS as f32, 0.0);

fn radius2() -> i64 {
	let r = RADIUS as i64;
	r * r
}

fn isqrt_u64(n: u64) -> u64 {
	if n < 2 {
		return n;
	}
	let mut x = n;
	let mut y = (x + n / x) / 2;
	while y < x {
		x = y;
		y = (x + n / x) / 2;
	}
	x
}

fn div_floor(a: i64, b: i64) -> i64 {
	debug_assert!(b > 0);
	let q = a / b;
	let r = a % b;
	if r < 0 { q - 1 } else { q }
}

fn div_ceil(a: i64, b: i64) -> i64 {
	-div_floor(-a, b)
}

fn sample_y_span(origin_y: i32, extent_y: i32, step: i32, sample_offset: i32, max_abs_y: i64) -> Option<(i32, i32)> {
	let step = step as i64;
	let start = div_ceil(-max_abs_y - origin_y as i64 - sample_offset as i64, step).max(0);
	let end = div_floor(max_abs_y - origin_y as i64 - sample_offset as i64, step).min(extent_y as i64 - 1);
	(start <= end).then_some((start as i32, end as i32))
}

fn region_intersects(min: Vec3, max: Vec3) -> bool {
	Vec3::ZERO.clamp(min, max).length_squared() <= (RADIUS as f32) * (RADIUS as f32)
}

fn sphere_voxel_unchecked(world: IVec3, mass: u32) -> Voxel {
	let normal = world.as_vec3().normalize_or_zero();
	let color = [
		((normal.x * 0.5 + 0.5) * 255.0) as u8,
		((normal.y * 0.5 + 0.5) * 255.0) as u8,
		((normal.z * 0.5 + 0.5) * 255.0) as u8,
		255,
	];
	Voxel { color, mass }
}

fn build_chunk(chunk: IVec3) -> Option<Voxels> {
	let _zone = tracy_client::span!("sphere chunk columns");
	let origin = chunk_origin(chunk);
	let r2 = radius2();
	let mut points = Vec::with_capacity((CHUNK_SIZE * CHUNK_SIZE * CHUNK_SIZE / 2) as usize);

	for z in 0..CHUNK_SIZE {
		let world_z = origin.z + z;
		let z2 = world_z as i64 * world_z as i64;
		for x in 0..CHUNK_SIZE {
			let world_x = origin.x + x;
			let rem = r2 - world_x as i64 * world_x as i64 - z2;
			if rem < 0 {
				continue;
			}
			let max_y = isqrt_u64(rem as u64) as i64;
			let Some((y0, y1)) = sample_y_span(origin.y, CHUNK_SIZE, 1, 0, max_y) else { continue };
			for y in y0..=y1 {
				points.push((I16Vec3::new(x as i16, y as i16, z as i16), sphere_voxel_unchecked(origin + IVec3::new(x, y, z), 100)));
			}
		}
	}

	if points.is_empty() {
		None
	} else {
		let mut voxels = Voxels::new();
		voxels.add_voxels(&points);
		Some(voxels)
	}
}

fn quantized_lod_voxel(voxel: Voxel) -> Voxel {
	Voxel {
		color: [
			quantize_channel(voxel.color[0], 6),
			quantize_channel(voxel.color[1], 6),
			quantize_channel(voxel.color[2], 6),
			255,
		],
		mass: 0,
	}
}

fn quantize_channel(value: u8, levels: u8) -> u8 {
	let max_level = (levels - 1) as f32;
	let level = ((value as f32 / 255.0) * max_level).round();
	((level / max_level) * 255.0).round() as u8
}

fn build_lod_region(min: IVec3, size: IVec3, lod: f32) -> Option<Voxels> {
	let _zone = tracy_client::span!("sphere direct LOD region");
	let step = 1i32 << lod.max(0.0).floor() as u32;
	let sample_offset = step / 2;
	let extent = (size * CHUNK_SIZE) / step;
	let origin = chunk_origin(min);
	let max_source = size * CHUNK_SIZE - IVec3::ONE;
	let r2 = radius2();
	let mut areas = Vec::new();

	for z in 0..extent.z {
		let sample_z = (z * step + sample_offset).min(max_source.z);
		let world_z = origin.z + sample_z;
		let z2 = world_z as i64 * world_z as i64;
		for x in 0..extent.x {
			let sample_x = (x * step + sample_offset).min(max_source.x);
			let world_x = origin.x + sample_x;
			let rem = r2 - world_x as i64 * world_x as i64 - z2;
			if rem < 0 {
				continue;
			}
			let max_y = isqrt_u64(rem as u64) as i64;
			let Some((y0, y1)) = sample_y_span(origin.y, extent.y, step, sample_offset, max_y) else { continue };

			let mid_y = ((y0 + y1) / 2 * step + sample_offset).min(max_source.y);
			let voxel = quantized_lod_voxel(sphere_voxel_unchecked(origin + IVec3::new(sample_x, mid_y, sample_z), 0));
			areas.push((I16Vec3::new(x as i16, y0 as i16, z as i16), I16Vec3::new(1, (y1 + 1 - y0) as i16, 1), voxel));
		}
	}

	if areas.is_empty() {
		None
	} else {
		let mut voxels = Voxels::new();
		voxels.add_areas(&areas);
		Some(voxels)
	}
}

struct SphereSource {
	grid: Arc<OnceLock<GridId>>,
	handle: OnceLock<SourceHandle>,
}

impl SphereSource {
	fn is_mine(&self, grid: GridId) -> bool {
		self.grid.get() == Some(&grid)
	}
}

impl ChunkSource for SphereSource {
	fn init(&self, handle: SourceHandle) {
		let _ = self.handle.set(handle);
	}

	fn cost(&self, grid: GridId, chunk: IVec3) -> Option<u32> {
		if !self.is_mine(grid) {
			return None;
		}
		let min = chunk_origin(chunk).as_vec3();
		region_intersects(min, min + Vec3::splat(CHUNK_SIZE as f32)).then_some(COST)
	}

	fn request_load(&self, grid: GridId, chunk: IVec3, generation: u64) {
		let voxels = build_chunk(chunk);
		if let Some(handle) = self.handle.get() {
			handle.loaded(grid, chunk, generation, voxels);
		}
	}

	fn cost_lod(&self, grid: GridId, min: IVec3, size: IVec3, _lod: f32) -> Option<u32> {
		if !self.is_mine(grid) {
			return None;
		}
		let lo = chunk_origin(min).as_vec3();
		let hi = chunk_origin(min + size).as_vec3();
		region_intersects(lo, hi).then_some(COST)
	}

	fn request_load_lod(&self, grid: GridId, min: IVec3, size: IVec3, lod: f32, generation: u64) {
		let voxels = build_lod_region(min, size, lod);
		if let Some(handle) = self.handle.get() {
			handle.loaded_lod(grid, min, size, lod, generation, voxels);
		}
	}
}

#[derive(Resource, Clone)]
struct SphereGrid(Arc<OnceLock<GridId>>);

pub struct SphereSourcePlugin;

impl Plugin for SphereSourcePlugin {
	fn build(&self, app: &mut App) {
		let grid = Arc::new(OnceLock::new());
		app.register_source(SphereSource { grid: grid.clone(), handle: OnceLock::new() });
		app.insert_resource(SphereGrid(grid));
		app.add_systems(Startup, spawn_sphere_grid);
	}
}

fn spawn_sphere_grid(mut commands: Commands, _store: Res<WorldStore>, grid: Res<SphereGrid>) {

	let radius_chunks = RADIUS.div_euclid(CHUNK_SIZE) + 1;
	let min = IVec3::splat(-radius_chunks);
	let size = IVec3::splat(radius_chunks * 2 + 1);

	let mut streaming = GridStreaming::default();
	streaming.presence_mut().mark_present_area(min, size);

	let body = commands
		.spawn((
			RigidBody,
			IsStatic,
			Transform::from_translation(CENTER),
		))
		.id();

	let grid_entity = commands
		.spawn((
			Transform::IDENTITY,
			Grid::new(),
			ReplicateVoxels,
			VoxelCollider,
			GridEdits::default(),
			streaming,
		))
		.id();
	let _ = grid.0.set(grid_entity);

	commands.entity(body).add_child(grid_entity);
}
