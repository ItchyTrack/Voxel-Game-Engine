use bevy::math::{IVec3, U16Vec3};
use voxel_data::grid_tree::{child_size, size as node_span, CellKind, CellRef, GridTreeView, NodeRef, U16Coord};
use voxel_data::voxel_grid_tree::VoxelGridType;
use voxel_data::voxels::{Voxel, VoxelType, VoxelTypeId, Voxels};
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

pub fn downsample_region(min: IVec3, size: IVec3, lod: f32, fetch: impl Fn(IVec3) -> Option<Voxels>) -> Option<Voxels> {
	let step = 1i32 << lod.max(0.0).floor() as u32;

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
					let accum = sum_node(view, root);
					if accum.weight > 0 {
						emit(root.origin, accum);
					}
				} else {
					process_node(view, root, step, &mut emit);
				}
			}
		}
	}

	if cells.is_empty() {
		return None;
	}

	let points = cells.iter().map(|cell| {
		let voxel = BasicVoxel { color: [cell.color[0], cell.color[1], cell.color[2], 255], mass: 0 }.into_voxel();
		(cell.pos.as_u16vec3(), voxel)
	}).collect::<Vec<(U16Vec3, Voxel)>>();
	let voxel_refs: Vec<_> = points.iter().map(|(pos, voxel)| (*pos, voxel.get_ref())).collect();

	let mut out = Voxels::new::<BasicVoxel>();
	out.add_voxels(&voxel_refs);

	if out.is_empty() { None } else { Some(out) }
}

fn process_node(
	view: GridTreeView<'_, VoxelGridType, U16Coord>,
	node: NodeRef,
	step: i32,
	emit: &mut impl FnMut(IVec3, Accum),
) {
	let child_sz = child_size(node.depth) as i32;

	if child_sz > step {
		for child in view.occupied_children(node) {
			process_cell(view, child, step, emit);
		}
	} else if child_sz == step {
		for child in view.occupied_children(node) {
			let accum = sum_cell(view, child);
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
			blocks[block].add(sum_cell(view, child));
			occupied[block] = true;
		}
		for b in 0..8 {
			if !occupied[b] { continue; }
			let offset = IVec3::new((b & 1) as i32, ((b >> 1) & 1) as i32, ((b >> 2) & 1) as i32);
			emit(node.origin + offset * step, blocks[b]);
		}
	}
}

fn process_cell(
	view: GridTreeView<'_, VoxelGridType, U16Coord>,
	cell: CellRef<'_, VoxelGridType>,
	step: i32,
	emit: &mut impl FnMut(IVec3, Accum),
) {
	match cell.kind() {
		CellKind::Empty => {}
		CellKind::Data => tile_uniform(cell, step, emit),
		CellKind::Node => {
			let node = view.child_node(cell).expect("Node kind implies child_node");
			process_node(view, node, step, emit);
		}
	}
}

fn sum_cell(view: GridTreeView<'_, VoxelGridType, U16Coord>, cell: CellRef<'_, VoxelGridType>) -> Accum {
	match cell.kind() {
		CellKind::Empty => Accum::default(),
		CellKind::Data => {
			let voxel = BasicVoxel::from_voxel_ref(&cell.data_value());
			Accum::from_leaf(&voxel, (cell.size as u64).pow(3))
		}
		CellKind::Node => {
			let node = view.child_node(cell).expect("Node kind implies child_node");
			sum_node(view, node)
		}
	}
}

fn sum_node(view: GridTreeView<'_, VoxelGridType, U16Coord>, node: NodeRef) -> Accum {
	let mut total = Accum::default();
	for child in view.occupied_children(node) {
		total.add(sum_cell(view, child));
	}
	total
}

fn tile_uniform(cell: CellRef<'_, VoxelGridType>, step: i32, emit: &mut impl FnMut(IVec3, Accum)) {
	let voxel = BasicVoxel::from_voxel_ref(&cell.data_value());
	let accum = Accum::from_leaf(&voxel, (step as u64).pow(3));
	let n = cell.size as i32 / step;
	for z in 0..n { for y in 0..n { for x in 0..n {
		emit(cell.origin + IVec3::new(x, y, z) * step, accum);
	}}}
}

fn round_channel(sum: u64, weight: u64) -> u8 {
	((sum + weight / 2) / weight).min(255) as u8
}

