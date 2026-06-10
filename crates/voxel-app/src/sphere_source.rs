use std::sync::{Arc, OnceLock};

use bevy::math::{I16Vec3, IVec3, Vec3};
use bevy::prelude::*;

use voxel_data::grid::Grid;
use voxel_data::voxels::{Voxel, Voxels};
use voxel_edit::GridEdits;
use voxel_physics::{IsStatic, RigidBody};
use voxel_physics::components::VoxelCollider;
use voxel_sources::{ChunkSource, GridKey, SourceHandle, VoxelSourcesAppExt};
use voxel_streaming::{chunk_origin, GridStreaming, CHUNK_SIZE};

use crate::streaming_test::WorldStore;

const RADIUS: i32 = 2_000;
const COST: u32 = 5;

/// Grid-local origin sits at the sphere centre, placed a full radius below the
/// church base (~y = -350) so the church ends up sitting on the sphere's top.
const CENTER: Vec3 = Vec3::new(0.0, -350.0 - RADIUS as f32, 0.0);

fn inside(point: IVec3) -> bool {
	point.as_vec3().length_squared() <= (RADIUS as f32) * (RADIUS as f32)
}

fn region_intersects(min: Vec3, max: Vec3) -> bool {
	Vec3::ZERO.clamp(min, max).length_squared() <= (RADIUS as f32) * (RADIUS as f32)
}

fn sphere_voxel(world: IVec3) -> Option<Voxel> {
	if !inside(world) {
		return None;
	}
	let normal = world.as_vec3().normalize_or_zero();
	let color = [
		((normal.x * 0.5 + 0.5) * 255.0) as u8,
		((normal.y * 0.5 + 0.5) * 255.0) as u8,
		((normal.z * 0.5 + 0.5) * 255.0) as u8,
		255,
	];
	Some(Voxel { color, mass: 100 })
}

fn build_chunk(chunk: IVec3) -> Option<Voxels> {
	let origin = chunk_origin(chunk);
	let mut voxels = Voxels::new();
	for x in 0..CHUNK_SIZE {
		for y in 0..CHUNK_SIZE {
			for z in 0..CHUNK_SIZE {
				let local = IVec3::new(x, y, z);
				let Some(voxel) = sphere_voxel(origin + local) else { continue };
				voxels.add_voxel(I16Vec3::new(x as i16, y as i16, z as i16), voxel);
			}
		}
	}
	(!voxels.is_empty()).then_some(voxels)
}

fn build_lod_region(min: IVec3, size: IVec3, lod: f32) -> Option<Voxels> {
	let _zone = tracy_client::span!("sphere direct LOD region");
	let step = 1i32 << lod.max(0.0).floor() as u32;
	let half_step = step / 2;
	let extent = (size * CHUNK_SIZE) / step;
	let origin = chunk_origin(min);
	let max_source = size * CHUNK_SIZE - IVec3::ONE;
	let mut voxels = Voxels::new();

	for z in 0..extent.z {
		for y in 0..extent.y {
			for x in 0..extent.x {
				let coarse = IVec3::new(x, y, z);
				let sample = (coarse * step + IVec3::splat(half_step)).min(max_source);
				let Some(voxel) = sphere_voxel(origin + sample) else { continue };
				voxels.add_voxel(I16Vec3::new(x as i16, y as i16, z as i16), voxel);
			}
		}
	}

	(!voxels.is_empty()).then_some(voxels)
}

struct SphereSource {
	grid: Arc<OnceLock<GridKey>>,
	handle: OnceLock<SourceHandle>,
}

impl SphereSource {
	fn is_mine(&self, grid: GridKey) -> bool {
		self.grid.get() == Some(&grid)
	}
}

impl ChunkSource for SphereSource {
	fn init(&self, handle: SourceHandle) {
		let _ = self.handle.set(handle);
	}

	fn cost(&self, grid: GridKey, chunk: IVec3) -> Option<u32> {
		if !self.is_mine(grid) {
			return None;
		}
		let min = chunk_origin(chunk).as_vec3();
		region_intersects(min, min + Vec3::splat(CHUNK_SIZE as f32)).then_some(COST)
	}

	fn request_load(&self, grid: GridKey, chunk: IVec3) {
		let voxels = build_chunk(chunk);
		if let Some(handle) = self.handle.get() {
			handle.loaded(grid, chunk, voxels);
		}
	}

	fn cost_lod(&self, grid: GridKey, min: IVec3, size: IVec3, _lod: f32) -> Option<u32> {
		if !self.is_mine(grid) {
			return None;
		}
		let lo = chunk_origin(min).as_vec3();
		let hi = chunk_origin(min + size).as_vec3();
		region_intersects(lo, hi).then_some(COST)
	}

	fn request_load_lod(&self, grid: GridKey, min: IVec3, size: IVec3, lod: f32) {
		let voxels = build_lod_region(min, size, lod);
		if let Some(handle) = self.handle.get() {
			handle.loaded_lod(grid, min, size, lod, voxels);
		}
	}
}

#[derive(Resource, Clone)]
struct SphereGrid(Arc<OnceLock<GridKey>>);

pub struct SphereSourcePlugin;

impl Plugin for SphereSourcePlugin {
	fn build(&self, app: &mut App) {
		let grid = Arc::new(OnceLock::new());
		app.register_source(SphereSource { grid: grid.clone(), handle: OnceLock::new() });
		app.insert_resource(SphereGrid(grid));
		app.add_systems(Startup, spawn_sphere_grid);
	}
}

fn spawn_sphere_grid(mut commands: Commands, store: Res<WorldStore>, grid: Res<SphereGrid>) {
	let key = store.alloc_key();
	let _ = grid.0.set(key);

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
			VoxelCollider,
			GridEdits::default(),
			key,
			streaming,
		))
		.id();

	commands.entity(body).add_child(grid_entity);
}
