use std::time::Duration;

use bevy::math::{I16Vec3, IVec3, Vec3};
use bevy::transform::components::Transform;
use criterion::{black_box, criterion_group, criterion_main, BatchSize, Criterion, Throughput};
use voxel_trees::grid_tree::NonZeroVoxelRegion;
use voxel_data::voxel_grid_tree::VoxelGridTree;

const CHUNK: i32 = 64;
const SMALL: i32 = 16;
const QUERIES: usize = 4096;

fn p(x: i32, y: i32, z: i32) -> I16Vec3 {
	I16Vec3::new(x as i16, y as i16, z as i16)
}

fn data(x: i32, y: i32, z: i32) -> u16 {
	((x * 17 + y * 29 + z * 43).rem_euclid(251) + 1) as u16
}

fn single_voxel_areas(size: i32) -> Vec<(I16Vec3, IVec3, u16)> {
	let mut out = Vec::with_capacity((size * size * size) as usize);
	for z in 0..size {
		for y in 0..size {
			for x in 0..size {
				out.push((p(x, y, z), IVec3::ONE, data(x, y, z)));
			}
		}
	}
	out
}

fn mixed_boxes() -> Vec<(I16Vec3, IVec3, u16)> {
	let mut out = Vec::new();
	for z in (0..CHUNK).step_by(8) {
		for y in (0..CHUNK).step_by(8) {
			for x in (0..CHUNK).step_by(8) {
				out.push((p(x, y, z), IVec3::new(6, 5, 7), data(x, y, z)));
			}
		}
	}
	out
}

fn single_voxels(size: i32) -> Vec<(I16Vec3, u16)> {
	let mut out = Vec::with_capacity((size * size * size) as usize);
	for z in 0..size {
		for y in 0..size {
			for x in 0..size {
				out.push((p(x, y, z), data(x, y, z)));
			}
		}
	}
	out
}

fn query_points(count: usize, span: i32) -> Vec<I16Vec3> {
	let mut state = 0x9e37_79b9_7f4a_7c15u64;
	let mut out = Vec::with_capacity(count);
	for _ in 0..count {
		state = state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
		let x = ((state >> 32) as i32).rem_euclid(span);
		state = state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
		let y = ((state >> 32) as i32).rem_euclid(span);
		state = state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
		let z = ((state >> 32) as i32).rem_euclid(span);
		out.push(p(x, y, z));
	}
	out
}

fn uniform_tree() -> VoxelGridTree {
	let mut tree = VoxelGridTree::new();
	tree.add_area(&p(0, 0, 0), IVec3::splat(CHUNK), 7);
	tree
}

fn gradient_tree(size: i32) -> VoxelGridTree {
	let mut tree = VoxelGridTree::new();
	tree.add_areas(&single_voxel_areas(size));
	tree
}

fn mixed_tree() -> VoxelGridTree {
	let mut tree = VoxelGridTree::new();
	tree.add_areas(&mixed_boxes());
	tree
}

fn bench_build(c: &mut Criterion) {
	let mut group = c.benchmark_group("grid_tree/build");
	group.throughput(Throughput::Elements((CHUNK * CHUNK * CHUNK) as u64));
	group.bench_function("add_area_uniform_64", |b| {
		b.iter(|| {
			let mut tree = VoxelGridTree::new();
			tree.add_area(black_box(&p(0, 0, 0)), black_box(IVec3::splat(CHUNK)), black_box(7));
			black_box(tree.len())
		})
	});

	let areas = single_voxel_areas(SMALL);
	group.throughput(Throughput::Elements((SMALL * SMALL * SMALL) as u64));
	group.bench_function("add_areas_single_voxel_16", |b| {
		b.iter_batched(
			|| areas.clone(),
			|areas| {
				let mut tree = VoxelGridTree::new();
				tree.add_areas(black_box(&areas));
				black_box(tree.len())
			},
			BatchSize::SmallInput,
		)
	});

	let voxels = single_voxels(SMALL);
	group.bench_function("add_single_voxels_16", |b| {
		b.iter_batched(
			|| voxels.clone(),
			|voxels| {
				let mut tree = VoxelGridTree::new();
				tree.add_single_voxels(black_box(&voxels));
				black_box(tree.len())
			},
			BatchSize::SmallInput,
		)
	});

	let boxes = mixed_boxes();
	group.throughput(Throughput::Elements(boxes.len() as u64));
	group.bench_function("add_areas_mixed_boxes_64", |b| {
		b.iter_batched(
			|| boxes.clone(),
			|areas| {
				let mut tree = VoxelGridTree::new();
				tree.add_areas(black_box(&areas));
				black_box(tree.len())
			},
			BatchSize::SmallInput,
		)
	});
	group.finish();
}

fn bench_query_mutate(c: &mut Criterion) {
	let mut group = c.benchmark_group("grid_tree/query_mutate");
	let points = query_points(QUERIES, SMALL);
	let gradient = gradient_tree(SMALL);
	let uniform = uniform_tree();
	let mixed = mixed_tree();

	group.throughput(Throughput::Elements(points.len() as u64));
	group.bench_function("get_existing_4096", |b| {
		b.iter(|| {
			let mut sum = 0u64;
			for pos in &points {
				sum += gradient.get(black_box(pos)).unwrap_or(0) as u64;
			}
			black_box(sum)
		})
	});

	group.bench_function("insert_4096", |b| {
		b.iter(|| {
			let mut tree = VoxelGridTree::new();
			for (i, pos) in points.iter().enumerate() {
				tree.insert(black_box(pos), black_box((i % 251 + 1) as u16));
			}
			black_box(tree.len())
		})
	});

	group.bench_function("remove_existing_4096", |b| {
		b.iter_batched(
			|| gradient_tree(SMALL),
			|mut tree| {
				let mut sum = 0u64;
				for pos in &points {
					sum += tree.remove(black_box(pos)).unwrap_or(0) as u64;
				}
				black_box(sum)
			},
			BatchSize::SmallInput,
		)
	});

	group.bench_function("remove_area_center_32", |b| {
		b.iter_batched(
			|| uniform.clone(),
			|mut tree| {
				tree.remove_area(black_box(&p(16, 16, 16)), black_box(IVec3::splat(32)));
				black_box(tree.len())
			},
			BatchSize::SmallInput,
		)
	});

	group.bench_function("is_area_filled_32", |b| {
		b.iter(|| black_box(uniform.is_area_filled(black_box(&p(8, 8, 8)), black_box(IVec3::splat(32)))))
	});

	group.bench_function("for_each_in_region_mixed_32", |b| {
		b.iter(|| {
			let mut sum = 0u64;
			mixed.for_each_in_region(black_box(NonZeroVoxelRegion::from_min_size(IVec3::splat(16), IVec3::splat(32)).unwrap()), |_, size, value| {
				sum = sum.wrapping_add(size as u64).wrapping_add(value as u64);
			});
			black_box(sum)
		})
	});
	group.finish();
}

fn sparse_destination_tree() -> VoxelGridTree {
	let mut tree = VoxelGridTree::new();
	for z in (0..CHUNK).step_by(16) {
		for y in (0..CHUNK).step_by(16) {
			for x in (0..CHUNK).step_by(16) {
				tree.add_area(&p(x + 4, y + 4, z + 4), IVec3::splat(4), 3);
			}
		}
	}
	tree
}

fn hollow_source_tree() -> VoxelGridTree {
	let mut tree = VoxelGridTree::new();
	tree.add_area(&p(0, 0, 0), IVec3::splat(CHUNK), 9);
	tree.remove_area(&p(20, 20, 20), IVec3::splat(12));
	tree
}

fn bench_region_transfer(c: &mut Criterion) {
	let mut group = c.benchmark_group("grid_tree/region_transfer");
	group.throughput(Throughput::Elements((CHUNK * CHUNK * CHUNK) as u64));

	let uniform = uniform_tree();
	group.bench_function("merge_tree_uniform_64_into_empty", |b| {
		b.iter_batched(
			VoxelGridTree::new,
			|mut dest| {
				dest.merge_tree(black_box(&uniform), black_box(IVec3::ZERO));
				black_box(dest.len())
			},
			BatchSize::SmallInput,
		)
	});

	group.bench_function("merge_tree_uniform_64_offset_into_empty", |b| {
		b.iter_batched(
			VoxelGridTree::new,
			|mut dest| {
				dest.merge_tree(black_box(&uniform), black_box(IVec3::new(5, -3, 7)));
				black_box(dest.len())
			},
			BatchSize::SmallInput,
		)
	});

	let region = NonZeroVoxelRegion::from_min_size(IVec3::splat(8), IVec3::splat(32)).unwrap();
	group.throughput(Throughput::Elements(32 * 32 * 32));
	group.bench_function("merge_region_uniform_32_into_empty", |b| {
		b.iter_batched(
			VoxelGridTree::new,
			|mut dest| {
				dest.merge_region_from(black_box(&uniform), black_box(region), black_box(IVec3::ZERO));
				black_box(dest.len())
			},
			BatchSize::SmallInput,
		)
	});

	group.bench_function("split_region_uniform_32", |b| {
		b.iter_batched(
			|| uniform.clone(),
			|mut tree| {
				let moved = tree.split_region(black_box(region));
				black_box((tree.len(), moved.len()))
			},
			BatchSize::SmallInput,
		)
	});

	let sparse_dest = sparse_destination_tree();
	let hollow_source = hollow_source_tree();
	group.throughput(Throughput::Elements((CHUNK * CHUNK * CHUNK) as u64));
	group.bench_function("merge_large_hollow_source_over_sparse_dest", |b| {
		b.iter_batched(
			|| sparse_dest.clone(),
			|mut dest| {
				dest.merge_tree(black_box(&hollow_source), black_box(IVec3::ZERO));
				black_box(dest.len())
			},
			BatchSize::SmallInput,
		)
	});

	let gradient = gradient_tree(SMALL);
	let gradient_region = NonZeroVoxelRegion::from_min_size(IVec3::ZERO, IVec3::splat(SMALL)).unwrap();
	group.throughput(Throughput::Elements((SMALL * SMALL * SMALL) as u64));
	group.bench_function("merge_region_mapped_gradient_16_into_existing", |b| {
		b.iter_batched(
			|| uniform_tree(),
			|mut dest| {
				dest.merge_region_from_mapped(black_box(&gradient), black_box(gradient_region), black_box(IVec3::new(24, 24, 24)), |id| id + 1);
				black_box(dest.len())
			},
			BatchSize::SmallInput,
		)
	});
	group.finish();
}

fn bench_traversal(c: &mut Criterion) {
	let mut group = c.benchmark_group("grid_tree/traversal");
	let uniform = uniform_tree();
	let mixed = mixed_tree();
	let gradient = gradient_tree(SMALL);

	group.bench_function("iter_uniform_64", |b| {
		b.iter(|| {
			let mut sum = 0u64;
			for (_, size, value) in &uniform {
				sum = sum.wrapping_add(size as u64).wrapping_add(value as u64);
			}
			black_box(sum)
		})
	});
	group.bench_function("iter_mixed_boxes_64", |b| {
		b.iter(|| {
			let mut sum = 0u64;
			for (_, size, value) in &mixed {
				sum = sum.wrapping_add(size as u64).wrapping_add(value as u64);
			}
			black_box(sum)
		})
	});
	group.bench_function("iter_gradient_16", |b| {
		b.iter(|| {
			let mut sum = 0u64;
			for (_, size, value) in &gradient {
				sum = sum.wrapping_add(size as u64).wrapping_add(value as u64);
			}
			black_box(sum)
		})
	});

	let transform = Transform::from_translation(Vec3::new(8.0, 8.0, -8.0));
	group.bench_function("raycast_gradient_16_z", |b| b.iter(|| black_box(gradient.raycast(black_box(&transform), black_box(Some(64.0))))));
	group.finish();
}

fn criterion_config() -> Criterion {
	Criterion::default().sample_size(40).warm_up_time(Duration::from_millis(300)).measurement_time(Duration::from_secs(2))
}

criterion_group! {
	name = benches;
	config = criterion_config();
	targets = bench_build, bench_query_mutate, bench_region_transfer, bench_traversal
}
criterion_main!(benches);
