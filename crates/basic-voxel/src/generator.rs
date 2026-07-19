use std::collections::HashMap;
use bevy::math::IVec3;
use voxel_data::grid_tree::{child_size, size as node_span, CellKind, CellRef, GridCell, GridCoord, GridTreeView, NodeRef};
use voxel_data::voxels::{VoxelRef, VoxelType, VoxelTypeId, Voxels};
use voxel_sources::VoxelLodGenerator;
use voxel_streaming::CHUNK_SIZE;

use crate::BasicVoxel;

#[derive(Clone, Copy, Debug, Default)]
pub struct BasicVoxelLodGenerator;

impl VoxelLodGenerator for BasicVoxelLodGenerator {
	fn input_type_id(&self) -> VoxelTypeId {
		BasicVoxel::TYPE_INFO.id
	}

	fn generate(&self, min: IVec3, size: IVec3, lod: f32, fetch: &dyn Fn(IVec3) -> Option<Voxels>) -> Option<Voxels> {
		downsample_region(min, size, lod, fetch)
	}
}

#[derive(Clone, Copy, Default)]
struct Accum {
	color: [u64; 3],
	weight: u64,
}

impl Accum {
	fn add(&mut self, other: Accum) {
		for c in 0..3 { self.color[c] += other.color[c]; }
		self.weight += other.weight;
	}
	fn from_leaf(voxel: &BasicVoxel, volume: u64) -> Self {
		Self {
			color: [voxel.color[0] as u64 * volume, voxel.color[1] as u64 * volume, voxel.color[2] as u64 * volume],
			weight: volume,
		}
	}
	fn average(&self) -> [u8; 3] {
		std::array::from_fn(|c| round_channel(self.color[c], self.weight))
	}
}

/// One finished coarse cell, pending final color assignment.
struct CoarseCell {
	pos: IVec3,
	color: [u8; 3],
}

const MAX_PALETTE_COLORS: usize = 254; // leave 1 slot of headroom under the u8 index cap

pub fn downsample_region(min: IVec3, size: IVec3, lod: f32, fetch: impl Fn(IVec3) -> Option<Voxels>) -> Option<Voxels> {
	let step = 1i32 << lod.max(0.0).floor() as u32;

	// --- Pass 1: exact accumulation, collect finished cells instead of writing directly ---
	let mut cells: Vec<CoarseCell> = Vec::new();
	for chunk_z in 0..size.z {
		for chunk_y in 0..size.y {
			for chunk_x in 0..size.x {
				let local = IVec3::new(chunk_x, chunk_y, chunk_z);
				let Some(src) = fetch(min + local) else { continue };
				let chunk_origin = local * CHUNK_SIZE;
				let view = src.grid_tree().view();
				let root = view.root();

				let mut emit = |local_origin: IVec3, accum: Accum| {
					if accum.weight == 0 { return; }
					cells.push(CoarseCell { pos: (chunk_origin + local_origin) / step, color: accum.average() });
				};

				if node_span(root.depth) as i32 <= step {
					let accum = sum_node(view, &src, root);
					if accum.weight > 0 {
						emit(root.origin, accum);
					}
				} else {
					process_node(view, &src, root, step, &mut emit);
				}
			}
		}
	}

	if cells.is_empty() {
		return None;
	}

	// --- Build weighted histogram of distinct colors ---
	let mut histogram: HashMap<[u8; 3], u64> = HashMap::new();
	for cell in &cells {
		*histogram.entry(cell.color).or_default() += 1;
	}

	// --- Pass 2: quantize only if the region genuinely exceeds the palette budget ---
	let palette: Vec<[u8; 3]> = if histogram.len() <= MAX_PALETTE_COLORS {
		histogram.keys().copied().collect()
	} else {
		weighted_median_cut(histogram, MAX_PALETTE_COLORS)
	};

	let mut out = Voxels::new::<BasicVoxel>();
	for cell in &cells {
		let color = nearest_palette_color(cell.color, &palette);
		let voxel = BasicVoxel { color: [color[0], color[1], color[2], 255], mass: 0 };
		let mut bytes = [0u8; BasicVoxel::TYPE_INFO.size_bytes as usize];
		voxel.into_bytes(&mut bytes);
		out.add_voxel(cell.pos.as_u16vec3(), VoxelRef::new(BasicVoxel::TYPE_INFO.id, &bytes));
	}

	if out.is_empty() { None } else { Some(out) }
}

fn process_node<C, Co>(
	view: GridTreeView<'_, C, Co>,
	src: &Voxels,
	node: NodeRef,
	step: i32,
	emit: &mut impl FnMut(IVec3, Accum),
) where
	C: GridCell<Data = u16>,
	Co: GridCoord,
{
	let child_sz = child_size(node.depth) as i32;

	if child_sz > step {
		for child in view.occupied_children(node) {
			process_cell(view, src, child, step, emit);
		}
	} else if child_sz == step {
		for child in view.occupied_children(node) {
			let accum = sum_cell(view, src, child);
			if accum.weight > 0 {
				emit(child.origin, accum);
			}
		}
	} else {
		debug_assert_eq!(child_sz * 2, step);
		let mut blocks = [Accum::default(); 8];
		let mut occupied = [false; 8];
		for child in view.occupied_children(node) {
			let local = (child.origin - node.origin) / child_sz;
			let block = ((local.x / 2) + (local.y / 2) * 2 + (local.z / 2) * 4) as usize;
			blocks[block].add(sum_cell(view, src, child));
			occupied[block] = true;
		}
		for b in 0..8 {
			if !occupied[b] { continue; }
			let offset = IVec3::new((b & 1) as i32, ((b >> 1) & 1) as i32, ((b >> 2) & 1) as i32);
			emit(node.origin + offset * step, blocks[b]);
		}
	}
}

fn process_cell<C, Co>(
	view: GridTreeView<'_, C, Co>,
	src: &Voxels,
	cell: CellRef<C>,
	step: i32,
	emit: &mut impl FnMut(IVec3, Accum),
) where
	C: GridCell<Data = u16>,
	Co: GridCoord,
{
	match cell.kind() {
		CellKind::Empty => {}
		CellKind::Data => tile_uniform(src, cell, step, emit),
		CellKind::Node => {
			let node = view.child_node(cell).expect("Node kind implies child_node");
			process_node(view, src, node, step, emit);
		}
	}
}

fn sum_cell<C, Co>(view: GridTreeView<'_, C, Co>, src: &Voxels, cell: CellRef<C>) -> Accum
where
	C: GridCell<Data = u16>,
	Co: GridCoord,
{
	match cell.kind() {
		CellKind::Empty => Accum::default(),
		CellKind::Data => {
			let Some(voxel) = src.voxel_for_palette_id(cell.data_value()).map(|v| BasicVoxel::from_voxel_ref(&v)) else {
				return Accum::default();
			};
			Accum::from_leaf(&voxel, (cell.size as u64).pow(3))
		}
		CellKind::Node => {
			let node = view.child_node(cell).expect("Node kind implies child_node");
			sum_node(view, src, node)
		}
	}
}

fn sum_node<C, Co>(view: GridTreeView<'_, C, Co>, src: &Voxels, node: NodeRef) -> Accum
where
	C: GridCell<Data = u16>,
	Co: GridCoord,
{
	let mut total = Accum::default();
	for child in view.occupied_children(node) {
		total.add(sum_cell(view, src, child));
	}
	total
}

fn tile_uniform<C: GridCell<Data = u16>>(src: &Voxels, cell: CellRef<C>, step: i32, emit: &mut impl FnMut(IVec3, Accum)) {
	let Some(voxel) = src.voxel_for_palette_id(cell.data_value()).map(|v| BasicVoxel::from_voxel_ref(&v)) else { return };
	let accum = Accum::from_leaf(&voxel, (step as u64).pow(3));
	let n = cell.size as i32 / step;
	for z in 0..n { for y in 0..n { for x in 0..n {
		emit(cell.origin + IVec3::new(x, y, z) * step, accum);
	}}}
}

fn round_channel(sum: u64, weight: u64) -> u8 {
	((sum + weight / 2) / weight).min(255) as u8
}

/// Weighted median-cut color quantization: repeatedly split the bucket with the
/// largest weighted spread along its widest channel, until `target` buckets exist,
/// then average each bucket (weighted) into one representative color.
fn weighted_median_cut(histogram: HashMap<[u8; 3], u64>, target: usize) -> Vec<[u8; 3]> {
	struct Bucket {
		colors: Vec<([u8; 3], u64)>,
	}
	impl Bucket {
		fn channel_range(&self, c: usize) -> u8 {
			let (mut lo, mut hi) = (255u8, 0u8);
			for (color, _) in &self.colors {
				lo = lo.min(color[c]);
				hi = hi.max(color[c]);
			}
			hi - lo
		}
		fn widest_channel(&self) -> usize {
			(0..3).max_by_key(|&c| self.channel_range(c)).unwrap()
		}
		fn total_weight(&self) -> u64 {
			self.colors.iter().map(|(_, w)| w).sum()
		}
		fn weighted_average(&self) -> [u8; 3] {
			let total = self.total_weight().max(1);
			std::array::from_fn(|c| {
				let sum: u64 = self.colors.iter().map(|(color, w)| color[c] as u64 * w).sum();
				((sum + total / 2) / total).min(255) as u8
			})
		}
	}

	let mut buckets = vec![Bucket { colors: histogram.into_iter().collect() }];

	while buckets.len() < target {
		// Split the bucket with the largest total weight (most impactful to refine).
		let Some((idx, _)) = buckets.iter().enumerate()
			.filter(|(_, b)| b.colors.len() > 1)
			.max_by_key(|(_, b)| b.total_weight())
		else { break }; // nothing left worth splitting

		let mut bucket = buckets.swap_remove(idx);
		let channel = bucket.widest_channel();
		bucket.colors.sort_by_key(|(color, _)| color[channel]);

		let total: u64 = bucket.total_weight();
		let mut running = 0u64;
		let mut split_at = bucket.colors.len() / 2; // fallback if weights are degenerate
		for (i, (_, w)) in bucket.colors.iter().enumerate() {
			running += w;
			if running * 2 >= total {
				split_at = (i + 1).clamp(1, bucket.colors.len() - 1);
				break;
			}
		}

		let second_half = bucket.colors.split_off(split_at);
		buckets.push(bucket);
		buckets.push(Bucket { colors: second_half });
	}

	buckets.iter().map(Bucket::weighted_average).collect()
}

fn nearest_palette_color(color: [u8; 3], palette: &[[u8; 3]]) -> [u8; 3] {
	palette.iter().copied().min_by_key(|p| {
		let dr = p[0] as i32 - color[0] as i32;
		let dg = p[1] as i32 - color[1] as i32;
		let db = p[2] as i32 - color[2] as i32;
		dr * dr + dg * dg + db * db
	}).unwrap_or(color)
}
