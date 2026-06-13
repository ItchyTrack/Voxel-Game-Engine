use std::hint::black_box;
use std::time::{Duration, Instant};

use bevy::math::{I16Vec3, IVec3};
use voxel_data::grid::{Grid, SUB_GRID_SIZE};
use voxel_data::voxel_grid_tree::VoxelGridTree;
use voxel_data::voxels::{Voxel, Voxels};

const CHUNK_SIZE: i32 = 64;
const ITERS: usize = 30;

fn voxel(c: u8) -> Voxel {
	Voxel { color: [c, c, c, 255], mass: 1 }
}

fn gradient_voxel(pos: IVec3) -> Voxel {
	// Keep the palette deliberately small enough for PackedCell's 15-bit data id
	// while still defeating large same-color run merging.
	let c = ((pos.x * 17 + pos.y * 29 + pos.z * 43).rem_euclid(251)) as u8;
	Voxel { color: [c, c.wrapping_mul(3), c.wrapping_mul(7), 255], mass: 1 }
}

fn build_uniform_chunk() -> Voxels {
	let mut voxels = Voxels::new();
	voxels.add_area(I16Vec3::ZERO, I16Vec3::splat(CHUNK_SIZE as i16), voxel(7));
	voxels
}

fn build_gradient_chunk() -> Voxels {
	let mut voxels = Voxels::new();
	for z in 0..CHUNK_SIZE {
		for y in 0..CHUNK_SIZE {
			for x in 0..CHUNK_SIZE {
				let pos = IVec3::new(x, y, z);
				voxels.add_voxel(pos.as_i16vec3(), gradient_voxel(pos));
			}
		}
	}
	voxels
}

fn build_sphere_surface_chunk() -> Voxels {
	let mut voxels = Voxels::new();
	let center = IVec3::splat(CHUNK_SIZE / 2);
	let radius_outer = 30 * 30;
	let radius_inner = 24 * 24;
	for z in 0..CHUNK_SIZE {
		for y in 0..CHUNK_SIZE {
			for x in 0..CHUNK_SIZE {
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

fn build_sphere_octant_chunk() -> Voxels {
	let mut voxels = Voxels::new();
	let center = IVec3::ZERO;
	let radius2 = (CHUNK_SIZE - 1) * (CHUNK_SIZE - 1);
	for z in 0..CHUNK_SIZE {
		for y in 0..CHUNK_SIZE {
			for x in 0..CHUNK_SIZE {
				let pos = IVec3::new(x, y, z);
				let d = pos - center;
				if d.dot(d) <= radius2 {
					voxels.add_voxel(pos.as_i16vec3(), gradient_voxel(pos));
				}
			}
		}
	}
	voxels
}

fn build_grid_tree_single_voxel_batch() -> Vec<(I16Vec3, IVec3, u16)> {
	let mut areas = Vec::with_capacity((CHUNK_SIZE * CHUNK_SIZE * CHUNK_SIZE) as usize);
	for z in 0..CHUNK_SIZE {
		for y in 0..CHUNK_SIZE {
			for x in 0..CHUNK_SIZE {
				areas.push((I16Vec3::new(x as i16, y as i16, z as i16), IVec3::ONE, ((x + y + z) % 251) as u16));
			}
		}
	}
	areas
}

fn measure(name: &str, iters: usize, mut f: impl FnMut() -> usize) {
	let mut times = Vec::with_capacity(iters);
	for _ in 0..iters {
		let start = Instant::now();
		let value = f();
		black_box(value);
		times.push(start.elapsed());
	}
	times.sort_unstable();
	let min = times[0];
	let median = times[times.len() / 2];
	let max = times[times.len() - 1];
	let mean = times.iter().copied().sum::<Duration>() / times.len() as u32;
	println!("{name:<36} min={:>8.3}ms median={:>8.3}ms mean={:>8.3}ms max={:>8.3}ms", ms(min), ms(median), ms(mean), ms(max));
}

fn ms(duration: Duration) -> f64 {
	duration.as_secs_f64() * 1000.0
}

fn bench_splat(name: &str, source: &Voxels, base: IVec3) {
	measure(name, ITERS, || {
		let mut grid = Grid::new();
		let touched = grid.splat_voxels(base, source);
		touched.len()
	});
}

fn make_splat_batches(source: &Voxels, base: IVec3) -> Vec<(IVec3, Vec<(I16Vec3, I16Vec3, Voxel)>)> {
	let mut batches: Vec<(IVec3, Vec<(I16Vec3, I16Vec3, Voxel)>)> = Vec::new();
	let palette = source.palette();
	let sub = IVec3::splat(SUB_GRID_SIZE);
	for (pos, size, palette_id) in source.grid_tree().iter() {
		let Some(voxel) = palette.voxel(palette_id) else { continue };
		let voxel = *voxel;
		let lo = base + pos.as_ivec3();
		if size == 1 {
			let sub_origin = lo.div_euclid(sub) * SUB_GRID_SIZE;
			let local = (lo - sub_origin).as_i16vec3();
			if let Some((_, batch)) = batches.iter_mut().find(|(origin, _)| *origin == sub_origin) {
				batch.push((local, I16Vec3::ONE, voxel));
			} else {
				batches.push((sub_origin, vec![(local, I16Vec3::ONE, voxel)]));
			}
			continue;
		}
		let hi = lo + IVec3::splat(size as i32);
		let sg_lo = lo.div_euclid(sub);
		let sg_hi = (hi - IVec3::ONE).div_euclid(sub);
		for sx in sg_lo.x..=sg_hi.x {
			for sy in sg_lo.y..=sg_hi.y {
				for sz in sg_lo.z..=sg_hi.z {
					let sub_origin = IVec3::new(sx, sy, sz) * SUB_GRID_SIZE;
					let cell_lo = lo.max(sub_origin);
					let cell_hi = hi.min(sub_origin + sub);
					let local = (cell_lo - sub_origin).as_i16vec3();
					let extent = (cell_hi - cell_lo).as_i16vec3();
					if let Some((_, batch)) = batches.iter_mut().find(|(origin, _)| *origin == sub_origin) {
						batch.push((local, extent, voxel));
					} else {
						batches.push((sub_origin, vec![(local, extent, voxel)]));
					}
				}
			}
		}
	}
	batches
}

fn bench_breakdown(name: &str, source: &Voxels, base: IVec3) {
	let batches = make_splat_batches(source, base);
	let flat_areas: Vec<_> = batches.iter().flat_map(|(_, batch)| batch.iter().copied()).collect();
	let tree_areas: Vec<_> = flat_areas.iter().map(|(pos, size, i)| (*pos, size.as_ivec3(), i.color[0] as u16)).collect();
	println!();
	println!("breakdown for {name}: leaves={}, subgrid_batches={}, areas={}", source.grid_tree().iter().count(), batches.len(), flat_areas.len());

	measure("breakdown: source tree iter only", ITERS, || source.grid_tree().iter().count());
	measure("breakdown: make splat batches", ITERS, || make_splat_batches(source, base).iter().map(|(_, b)| b.len()).sum());
	measure("breakdown: apply Voxels batches", ITERS, || {
		let mut total = 0usize;
		for (_, batch) in &batches {
			let mut voxels = Voxels::new();
			voxels.add_areas(batch);
			total += voxels.grid_tree().len() as usize;
		}
		total
	});
	measure("breakdown: Voxels::add_areas flat", ITERS, || {
		let mut voxels = Voxels::new();
		voxels.add_areas(&flat_areas);
		voxels.grid_tree().len() as usize
	});
	measure("breakdown: GridTree::add_areas flat", ITERS, || {
		let mut tree = VoxelGridTree::new();
		tree.add_areas(&tree_areas);
		tree.len() as usize
	});
}

fn main() {
	println!("voxel-data splat/add_areas microbench ({ITERS} iterations, release recommended)");
	println!();

	let uniform = build_uniform_chunk();
	let gradient = build_gradient_chunk();
	let sphere_surface = build_sphere_surface_chunk();
	let sphere_octant = build_sphere_octant_chunk();
	let single_voxel_batch = build_grid_tree_single_voxel_batch();

	bench_splat("splat uniform 64^3 @ origin", &uniform, IVec3::ZERO);
	bench_splat("splat uniform 64^3 unaligned", &uniform, IVec3::new(-17, 5, -70));
	bench_splat("splat gradient 64^3 @ origin", &gradient, IVec3::ZERO);
	bench_splat("splat gradient 64^3 unaligned", &gradient, IVec3::new(-17, 5, -70));
	bench_splat("splat sphere shell unaligned", &sphere_surface, IVec3::new(-17, 5, -70));
	bench_splat("splat sphere octant @ origin", &sphere_octant, IVec3::ZERO);
	bench_splat("splat sphere octant unaligned", &sphere_octant, IVec3::new(-17, 5, -70));

	measure("GridTree::add_areas 64^3 singles", 10, || {
		let mut tree = VoxelGridTree::new();
		tree.add_areas(&single_voxel_batch);
		tree.len() as usize
	});
	measure("GridTree::add_area 57^3 single", ITERS, || {
		let mut tree = VoxelGridTree::new();
		tree.add_area(&I16Vec3::ZERO, IVec3::splat(57), 7);
		tree.len() as usize
	});
	measure("Voxels::add_area 57^3 single", ITERS, || {
		let mut voxels = Voxels::new();
		voxels.add_area(I16Vec3::ZERO, I16Vec3::splat(57), voxel(7));
		voxels.grid_tree().len() as usize
	});

	bench_breakdown("gradient 64^3 unaligned", &gradient, IVec3::new(-17, 5, -70));
	bench_breakdown("sphere octant unaligned", &sphere_octant, IVec3::new(-17, 5, -70));
}
