use std::sync::{Arc, OnceLock};

use bevy::math::{IVec3, Vec3};
use bevy::prelude::*;

use voxel_data::grid::Grid;
use voxel_data::voxels::{VoxelType, VoxelTypeId, Voxels};
use voxel_sources::edit::GridEditIdManager;
use voxel_physics::{IsStatic, RigidBody};
use voxel_physics::components::VoxelCollider;
use voxel_lightyear::ReplicateVoxels;
use voxel_data::grid::GridId;
use voxel_sources::{ForgottenChunks, ChunkSource, RequestId, SourceCoverage, SourceHandle, VoxelSourcesAppExt, edit::GridGeneration};
use voxel_tasks::{AsyncPriorityTaskPool, CancellationToken};
use tile_data::{CHUNK_SIZE, NonZeroChunkRegion, chunk_origin};
use voxel_streaming::GridStreaming;
use basic_voxel::{BasicVoxel, LodVoxel};

const RADIUS: i32 = 2_000;

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

fn chunk_owned(grid: GridId, chunk: IVec3, forgotten: &ForgottenChunks) -> bool {
	let origin = chunk_origin(chunk).as_vec3();
	!forgotten.contains(grid, chunk) && region_intersects(origin, origin + Vec3::splat(CHUNK_SIZE as f32))
}

fn region_chunks(region: NonZeroChunkRegion) -> impl Iterator<Item = IVec3> {
	(region.min().z..region.end().z).flat_map(move |z| {
		(region.min().y..region.end().y).flat_map(move |y| {
			(region.min().x..region.end().x).map(move |x| IVec3::new(x, y, z))
		})
	})
}

fn sphere_chunk_region() -> NonZeroChunkRegion {
	let radius_chunks = RADIUS.div_euclid(CHUNK_SIZE as i32) + 1;
	NonZeroChunkRegion::from_min_size(
		IVec3::splat(-radius_chunks),
		UVec3::splat(radius_chunks as u32 * 2 + 1),
	).unwrap()
}

fn sphere_color(world: IVec3) -> [u8; 4] {
	let normal = world.as_vec3().normalize_or_zero();
	[
		((normal.x * 0.5 + 0.5) * 255.0) as u8,
		((normal.y * 0.5 + 0.5) * 255.0) as u8,
		((normal.z * 0.5 + 0.5) * 255.0) as u8,
		255,
	]
}

fn sphere_voxel_unchecked(world: IVec3, mass: u32) -> BasicVoxel {
	BasicVoxel { color: sphere_color(world), mass }
}

fn sphere_lod_voxel_unchecked(world: IVec3) -> LodVoxel {
	LodVoxel::solid(sphere_color(world))
}

fn quantize_channel(value: u8, levels: u8) -> u8 {
	let max_level = (levels - 1) as f32;
	let level = ((value as f32 / 255.0) * max_level).round();
	((level / max_level) * 255.0).round() as u8
}

fn build_region(
	region: NonZeroChunkRegion,
	chunks: &[IVec3],
	lod: u8,
	use_raw: bool,
	cancellation: &CancellationToken,
) -> Option<Voxels> {
	let _zone = tracy_client::span!("sphere region");

	let step = if use_raw { 1 } else { 1i32 << lod as u32 };
	let sample_offset = if use_raw { 0 } else { step / 2 };
	let chunk_extent = CHUNK_SIZE as i32 / step;
	let max_source = IVec3::splat(CHUNK_SIZE as i32 - 1);
	let r2 = radius2();

	// Only one of these ends up populated, depending on `use_raw`.
	let mut points = Vec::new();
	let mut areas = Vec::new();

	for &chunk in chunks {
		if cancellation.is_cancelled() { return None; }
		let origin = chunk_origin(chunk);
		let chunk_offset = (chunk - region.min()) * chunk_extent;

		for z in 0..chunk_extent {
			let sample_z = (z * step + sample_offset).min(max_source.z);
			let world_z = origin.z + sample_z;
			let z2 = world_z as i64 * world_z as i64;
			for x in 0..chunk_extent {
				let sample_x = (x * step + sample_offset).min(max_source.x);
				let world_x = origin.x + sample_x;
				let rem = r2 - world_x as i64 * world_x as i64 - z2;
				if rem < 0 {
					continue;
				}
				let max_y = isqrt_u64(rem as u64) as i64;
				let Some((y0, y1)) = sample_y_span(origin.y, chunk_extent, step, sample_offset, max_y) else { continue };

				if use_raw {
					for y in y0..=y1 {
						let world = origin + IVec3::new(sample_x, y, sample_z);
						let pos = chunk_offset + IVec3::new(x, y, z);
						points.push((pos.as_uvec3(), sphere_voxel_unchecked(world, 100)));
					}
				} else {
					let sample_y = ((y0 + y1) / 2 * step + sample_offset).min(max_source.y);
					let world = origin + IVec3::new(sample_x, sample_y, sample_z);
					let pos = chunk_offset + IVec3::new(x, y0, z);
					areas.push((pos.as_uvec3(), UVec3::new(1, (y1 + 1 - y0) as u32, 1), sphere_lod_voxel_unchecked(world)));
				}
			}
		}
	}

	if use_raw {
		if points.is_empty() {
			return None;
		}
		let voxel_refs: Vec<_> = points.iter().map(|(pos, voxel)| (*pos, voxel.get_ref())).collect();
		let mut voxels = Voxels::new::<BasicVoxel>();
		voxels.add_voxels(&voxel_refs);
		Some(voxels)
	} else {
		if areas.is_empty() {
			return None;
		}
		let area_refs: Vec<_> = areas.iter().map(|(pos, size, voxel)| (*pos, *size, voxel.get_ref())).collect();
		let mut voxels = Voxels::new::<LodVoxel>();
		voxels.add_areas(&area_refs);
		Some(voxels)
	}
}

struct SphereSource {
	grid: Arc<OnceLock<GridId>>,
	handle: OnceLock<SourceHandle>,
	forgotten: ForgottenChunks,
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

	fn request_voxels(
		&self,
		request_id: RequestId,
		cancellation: &CancellationToken,
		grid: GridId,
		region: NonZeroChunkRegion,
		lod: u8,
		voxel_type: Option<VoxelTypeId>,
		generation: GridGeneration,
	) -> SourceCoverage {
		if cancellation.is_cancelled() || !self.is_mine(grid) {
			return SourceCoverage::None;
		}

		let owned_chunks: Vec<_> = region_chunks(region)
			.filter(|&chunk| chunk_owned(grid, chunk, &self.forgotten))
			.collect();
		let coverage = if owned_chunks.is_empty() {
			return SourceCoverage::None;
		} else if owned_chunks.len() == region.area() as usize {
			SourceCoverage::All
		} else {
			SourceCoverage::Some
		};

		let use_raw = match voxel_type {
			Some(id) if id == BasicVoxel::TYPE_INFO.id && lod == 0 => true,
			Some(id) if id == LodVoxel::TYPE_INFO.id => false,
			_ => lod == 0,
		};

		let handle = self.handle.get().expect("sphere source was not initialized").clone();
		let cancellation = cancellation.clone();
		AsyncPriorityTaskPool::get().spawn(1.0, async move {
			let _span = bevy::log::info_span!("SphereSource build").entered();
			let voxels = build_region(region, &owned_chunks, lod, use_raw, &cancellation);
			if !cancellation.is_cancelled() && let Some(voxels) = voxels {
				handle.voxels(request_id, grid, region, lod, generation, voxels);
			}
			handle.voxels_loaded(request_id, generation);
		});
		coverage
	}

	fn request_presence(&self, request_id: RequestId, _cancellation: CancellationToken, grid: GridId) {
		let handle = self.handle.get().expect("sphere source was not initialized");
		if self.is_mine(grid) {
			handle.presence(request_id, grid, sphere_chunk_region());
		}
		handle.presence_loaded(request_id);
	}

	fn acquire_ownership(&self, grid: GridId, region: NonZeroChunkRegion) {
		if !self.is_mine(grid) { return; }
		self.forgotten.remember_area(grid, region);
	}

	fn relinquish_ownership(&self, grid: GridId, region: NonZeroChunkRegion) {
		if !self.is_mine(grid) { return; }
		self.forgotten.forget_area(grid, region);
	}
}

#[derive(Resource, Clone)]
struct SphereGrid(Arc<OnceLock<GridId>>);

pub struct SphereSourcePlugin;

impl Plugin for SphereSourcePlugin {
	fn build(&self, app: &mut App) {
		let grid = Arc::new(OnceLock::new());
		app.register_voxel_source(SphereSource {
			grid: grid.clone(),
			handle: OnceLock::new(),
			forgotten: ForgottenChunks::default(),
		});
		app.insert_resource(SphereGrid(grid));
		app.add_systems(Startup, spawn_sphere_grid);
	}
}

fn spawn_sphere_grid(mut commands: Commands, grid: Res<SphereGrid>) {
	let mut streaming = GridStreaming::default();
	streaming.mark_present_area(sphere_chunk_region());

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
			Grid::new::<BasicVoxel>(),
			VoxelCollider,
			GridEditIdManager::default(),
			ReplicateVoxels,
			streaming,
		))
		.id();
	let _ = grid.0.set(grid_entity);

	commands.entity(body).add_child(grid_entity);
}
