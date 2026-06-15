use std::time::Duration;

use bevy::math::{I16Vec3, IVec3};
use bevy::tasks::{ComputeTaskPool, TaskPoolBuilder};
use criterion::{black_box, criterion_group, criterion_main, BatchSize, Criterion};
use voxel_data::grid::Grid;
use voxel_data::splat::{splat_voxels_blocking, GridSplat};
use voxel_data::voxels::{Voxel, Voxels};

const CHUNK: i32 = 64;

fn voxel(c: u8) -> Voxel {
	Voxel { color: [c, c, c, 255], mass: 1 }
}

fn gradient_voxel(pos: IVec3) -> Voxel {
	let c = ((pos.x * 17 + pos.y * 29 + pos.z * 43).rem_euclid(251)) as u8;
	Voxel { color: [c, c.wrapping_mul(3), c.wrapping_mul(7), 255], mass: 1 }
}

fn uniform_chunk() -> Voxels {
	let mut voxels = Voxels::new();
	voxels.add_area(I16Vec3::ZERO, I16Vec3::splat(CHUNK as i16), voxel(7));
	voxels
}

fn gradient_chunk() -> Voxels {
	let mut voxels = Voxels::new();
	for z in 0..CHUNK {
		for y in 0..CHUNK {
			for x in 0..CHUNK {
				let pos = IVec3::new(x, y, z);
				voxels.add_voxel(pos.as_i16vec3(), gradient_voxel(pos));
			}
		}
	}
	voxels
}

fn sphere_surface_chunk() -> Voxels {
	let mut voxels = Voxels::new();
	let center = IVec3::splat(CHUNK / 2);
	let radius_outer = 30 * 30;
	let radius_inner = 24 * 24;
	for z in 0..CHUNK {
		for y in 0..CHUNK {
			for x in 0..CHUNK {
				let pos = IVec3::new(x, y, z);
				let d = pos - center;
				let dist2 = d.dot(d);
				if dist2 <= radius_outer && dist2 >= radius_inner {
					voxels.add_voxel(pos.as_i16vec3(), gradient_voxel(pos));
				}
			}
		}
	}
	voxels
}

fn bench_grid_splat_voxels(c: &mut Criterion) {
	let uniform = uniform_chunk();
	let gradient = gradient_chunk();
	let sphere = sphere_surface_chunk();
	let mut group = c.benchmark_group("grid/splat_voxels");

	group.bench_function("uniform_64_empty_grid", |b| {
		b.iter_batched(
			Grid::new,
			|mut grid| {
				let touched = grid.splat_voxels(black_box(IVec3::ZERO), black_box(&uniform));
				black_box(touched.len())
			},
			BatchSize::SmallInput,
		)
	});
	group.bench_function("gradient_64_empty_grid", |b| {
		b.iter_batched(
			Grid::new,
			|mut grid| {
				let touched = grid.splat_voxels(black_box(IVec3::ZERO), black_box(&gradient));
				black_box(touched.len())
			},
			BatchSize::SmallInput,
		)
	});
	group.bench_function("sphere_surface_64_empty_grid", |b| {
		b.iter_batched(
			Grid::new,
			|mut grid| {
				let touched = grid.splat_voxels(black_box(IVec3::ZERO), black_box(&sphere));
				black_box(touched.len())
			},
			BatchSize::SmallInput,
		)
	});
	group.bench_function("gradient_over_existing_uniform", |b| {
		b.iter_batched(
			|| {
				let mut grid = Grid::new();
				grid.splat_voxels(IVec3::ZERO, &uniform);
				grid
			},
			|mut grid| {
				let touched = grid.splat_voxels(black_box(IVec3::ZERO), black_box(&gradient));
				black_box(touched.len())
			},
			BatchSize::SmallInput,
		)
	});
	group.finish();
}

fn bench_splat_voxels_blocking(c: &mut Criterion) {
	ComputeTaskPool::get_or_init(|| TaskPoolBuilder::new().build());

	let uniform = uniform_chunk();
	let gradient = gradient_chunk();
	let sphere = sphere_surface_chunk();
	let mut group = c.benchmark_group("grid/splat_voxels_blocking");

	group.bench_function("uniform_64_empty_grid", |b| {
		b.iter_batched(
			Grid::new,
			|mut grid| {
				let splats = [GridSplat { grid: 0, base: IVec3::ZERO, voxels: &uniform }];
				let touched = splat_voxels_blocking(std::slice::from_mut(&mut grid), black_box(&splats));
				black_box(touched.len())
			},
			BatchSize::SmallInput,
		)
	});
	group.bench_function("gradient_64_empty_grid", |b| {
		b.iter_batched(
			Grid::new,
			|mut grid| {
				let splats = [GridSplat { grid: 0, base: IVec3::ZERO, voxels: &gradient }];
				let touched = splat_voxels_blocking(std::slice::from_mut(&mut grid), black_box(&splats));
				black_box(touched.len())
			},
			BatchSize::SmallInput,
		)
	});
	group.bench_function("sphere_surface_64_empty_grid", |b| {
		b.iter_batched(
			Grid::new,
			|mut grid| {
				let splats = [GridSplat { grid: 0, base: IVec3::ZERO, voxels: &sphere }];
				let touched = splat_voxels_blocking(std::slice::from_mut(&mut grid), black_box(&splats));
				black_box(touched.len())
			},
			BatchSize::SmallInput,
		)
	});
	group.bench_function("four_overlapping_uniform_splats", |b| {
		b.iter_batched(
			Grid::new,
			|mut grid| {
				let splats = [
					GridSplat { grid: 0, base: IVec3::new(0, 0, 0), voxels: &uniform },
					GridSplat { grid: 0, base: IVec3::new(32, 0, 0), voxels: &uniform },
					GridSplat { grid: 0, base: IVec3::new(0, 32, 0), voxels: &uniform },
					GridSplat { grid: 0, base: IVec3::new(0, 0, 32), voxels: &uniform },
				];
				let touched = splat_voxels_blocking(std::slice::from_mut(&mut grid), black_box(&splats));
				black_box(touched.len())
			},
			BatchSize::SmallInput,
		)
	});
	group.bench_function("gradient_over_existing_uniform", |b| {
		b.iter_batched(
			|| {
				let mut grid = Grid::new();
				let splats = [GridSplat { grid: 0, base: IVec3::ZERO, voxels: &uniform }];
				splat_voxels_blocking(std::slice::from_mut(&mut grid), &splats);
				grid
			},
			|mut grid| {
				let splats = [GridSplat { grid: 0, base: IVec3::ZERO, voxels: &gradient }];
				let touched = splat_voxels_blocking(std::slice::from_mut(&mut grid), black_box(&splats));
				black_box(touched.len())
			},
			BatchSize::SmallInput,
		)
	});
	group.finish();
}

fn criterion_config() -> Criterion {
	Criterion::default().sample_size(40).warm_up_time(Duration::from_millis(300)).measurement_time(Duration::from_secs(2))
}

criterion_group! {
	name = benches;
	config = criterion_config();
	targets = bench_grid_splat_voxels, bench_splat_voxels_blocking
}
criterion_main!(benches);
