use std::time::Duration;

use bevy::math::{IVec3, UVec3};
use criterion::{black_box, criterion_group, criterion_main, Criterion, Throughput};
use voxel_data::{
	grid_tree::{reduce_grid_trees, GridReducer, NonZeroVoxelRegion, SourceOverlaps, SourceTree},
	voxel_grid_tree::{PackedCell, PackedGridTree},
};

const SOURCE_SIZE: u32 = 16;
const OUTPUT_SIZE: u32 = 16;

#[derive(Clone, Copy)]
struct SumReducer;

impl GridReducer<PackedCell> for SumReducer {
	type Output = u16;

	fn output_grid_type(&self) -> PackedCell { PackedCell }

	fn reduce<'overlaps, 'a>(
		&mut self,
		_region: NonZeroVoxelRegion,
		overlaps: SourceOverlaps<'overlaps, 'a, PackedCell>,
	) -> Option<Self::Output> {
		let mut seen = false;
		let mut sum = 0u16;
		for overlap in overlaps {
			seen = true;
			sum = sum.wrapping_add(overlap.data);
		}
		seen.then_some(sum)
	}
}

fn dense_source() -> PackedGridTree {
	let mut voxels = Vec::with_capacity(SOURCE_SIZE as usize * SOURCE_SIZE as usize * SOURCE_SIZE as usize);
	for z in 0..SOURCE_SIZE {
		for y in 0..SOURCE_SIZE {
			for x in 0..SOURCE_SIZE {
				let value = ((x as u32 * 17 + y as u32 * 29 + z as u32 * 43) % 31 + 1) as u16;
				voxels.push((UVec3::new(x, y, z), value));
			}
		}
	}
	let mut tree = PackedGridTree::new();
	tree.add_single_voxels(&voxels);
	tree
}

fn sparse_source() -> PackedGridTree {
	let mut voxels = Vec::new();
	for z in 0..SOURCE_SIZE {
		for y in 0..SOURCE_SIZE {
			for x in 0..SOURCE_SIZE {
				let dx = x as i32 - 8;
				let dy = y as i32 - 8;
				let dz = z as i32 - 8;
				if dx * dx + dy * dy + dz * dz > 36 { continue; }
				let value = ((x as u32 * 17 + y as u32 * 29 + z as u32 * 43) % 31 + 1) as u16;
				voxels.push((UVec3::new(x, y, z), value));
			}
		}
	}
	let mut tree = PackedGridTree::new();
	tree.add_single_voxels(&voxels);
	tree
}

fn bench_case(group: &mut criterion::BenchmarkGroup<'_, criterion::measurement::WallTime>, name: &str, trees: &[PackedGridTree]) {
	let sources: Vec<_> = trees
		.iter()
		.enumerate()
		.map(|(index, tree)| {
			let offset = IVec3::new((index & 1) as i32, ((index >> 1) & 1) as i32, ((index >> 2) & 1) as i32) * (OUTPUT_SIZE / 2) as i32;
			SourceTree { tree, scale_down: 1, output_offset: offset }
		})
		.collect();
	let output_region = NonZeroVoxelRegion::from_min_size(IVec3::ZERO, UVec3::splat(OUTPUT_SIZE)).expect("benchmark output region");
	group.bench_function(name, |b| {
		b.iter(|| {
			let output = reduce_grid_trees(black_box(output_region), black_box(&sources), SumReducer);
			black_box(output.map(|tree| tree.len()))
		})
	});
}

fn bench_reduce(c: &mut Criterion) {
	let dense_trees: Vec<_> = (0..8).map(|_| dense_source()).collect();
	let sparse_trees: Vec<_> = (0..8).map(|_| sparse_source()).collect();
	let mut group = c.benchmark_group("grid_tree/reduce");
	group.throughput(Throughput::Elements((OUTPUT_SIZE * OUTPUT_SIZE * OUTPUT_SIZE) as u64));
	bench_case(&mut group, "eight_dense_sources_16_downsample_2x", &dense_trees);
	bench_case(&mut group, "eight_sparse_sources_16_downsample_2x", &sparse_trees);
	group.finish();
}

fn criterion_config() -> Criterion {
	Criterion::default().sample_size(40).warm_up_time(Duration::from_millis(300)).measurement_time(Duration::from_secs(2))
}

criterion_group! {
	name = benches;
	config = criterion_config();
	targets = bench_reduce
}
criterion_main!(benches);
