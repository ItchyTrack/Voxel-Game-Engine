use basic_voxel::MarchingVoxel;
use bevy::math::{IVec3, Vec3};
use earcut::int::EarcutI32;
use rustc_hash::FxHashMap;
use voxel_data::{grid_tree::GridRegion, voxels::{VoxelType, Voxels}};

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, bytemuck::Pod, bytemuck::Zeroable)]
pub struct MarchingTriangle {
	pub packed_origin_xy: u32,
	pub packed_origin_z_edges: u32,
	pub color: u32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct CellSamples {
	colors: [u32; 8],
	occupied: u8,
}

const CORNERS: [IVec3; 8] = [
	IVec3::new(0, 0, 0), IVec3::new(1, 0, 0),
	IVec3::new(1, 1, 0), IVec3::new(0, 1, 0),
	IVec3::new(0, 0, 1), IVec3::new(1, 0, 1),
	IVec3::new(1, 1, 1), IVec3::new(0, 1, 1),
];

const EDGES: [(usize, usize); 12] = [
	(0, 1), (1, 2), (2, 3), (3, 0),
	(4, 5), (5, 6), (6, 7), (7, 4),
	(0, 4), (1, 5), (2, 6), (3, 7),
];

// Edge midpoints in exact half-voxel units.
const EDGE_MIDPOINTS: [IVec3; 12] = [
	IVec3::new(1, 0, 0), IVec3::new(2, 1, 0),
	IVec3::new(1, 2, 0), IVec3::new(0, 1, 0),
	IVec3::new(1, 0, 2), IVec3::new(2, 1, 2),
	IVec3::new(1, 2, 2), IVec3::new(0, 1, 2),
	IVec3::new(0, 0, 1), IVec3::new(2, 0, 1),
	IVec3::new(2, 2, 1), IVec3::new(0, 2, 1),
];

// Face corners and the edge following each corner around that face.
const FACES: [([usize; 4], [usize; 4]); 6] = [
	([0, 1, 2, 3], [0, 1, 2, 3]),
	([4, 5, 6, 7], [4, 5, 6, 7]),
	([0, 1, 5, 4], [0, 9, 4, 8]),
	([3, 2, 6, 7], [2, 10, 6, 11]),
	([0, 3, 7, 4], [3, 11, 7, 8]),
	([1, 2, 6, 5], [1, 10, 5, 9]),
];

pub fn make_marching_cubes_mesh(voxels: &Voxels) -> (Vec<MarchingTriangle>, u32, Vec3, Vec3) {
	let Some((occupied_min, occupied_max)) = voxels.bounding_box() else {
		return (Vec::new(), 0, Vec3::ZERO, Vec3::ZERO);
	};
	let cell_min = occupied_min.as_ivec3() - IVec3::ONE;
	let cell_size = occupied_max.as_ivec3() - cell_min + IVec3::ONE;
	make_marching_cubes_mesh_in_region(voxels, cell_min, cell_size)
}

pub fn make_marching_cubes_mesh_in_region(
	voxels: &Voxels,
	cell_min: IVec3,
	cell_size: IVec3,
) -> (Vec<MarchingTriangle>, u32, Vec3, Vec3) {
	if cell_size.cmple(IVec3::ZERO).any() {
		return (Vec::new(), 0, Vec3::ZERO, Vec3::ZERO);
	}
	let cell_end = cell_min + cell_size;
	let cell_region = GridRegion { min: cell_min, end: cell_end };
	let mut cells: Vec<_> = stage_cells(voxels, cell_region).into_iter().collect();
	cells.sort_unstable_by_key(|(origin, _)| (origin.z, origin.y, origin.x));
	let mut triangles = Vec::new();
	let mut earcut = EarcutI32::new();
	let mut triangle_indices = Vec::new();
	for (origin, samples) in cells {
		polygonize_cell(
			origin,
			cell_min,
			samples,
			&mut earcut,
			&mut triangle_indices,
			&mut triangles,
		);
	}

	let vertex_count = (triangles.len() * 3) as u32;
	(triangles, vertex_count, Vec3::ZERO, cell_size.as_vec3())
}

fn stage_cells(voxels: &Voxels, cell_region: GridRegion) -> FxHashMap<IVec3, CellSamples> {
	let sample_min = cell_region.min.max(IVec3::ZERO);
	let sample_end = (cell_region.end + IVec3::ONE).min(IVec3::splat(u16::MAX as i32 + 1));
	let mut cells = FxHashMap::default();
	if let Some(sample_region) = GridRegion::new(sample_min, sample_end) {
		voxels.grid_tree().for_each_in_region(sample_region, |origin, size, voxel| {
			let leaf_min = origin.as_ivec3();
			let leaf_end = leaf_min + IVec3::splat(i32::from(size));
			stage_leaf_boundary(
				&mut cells,
				cell_region,
				leaf_min,
				leaf_end,
				MarchingVoxel::from_voxel_ref(&voxel),
			);
		});
	}
	cells
}

fn stage_leaf_boundary(
	cells: &mut FxHashMap<IVec3, CellSamples>,
	cell_region: GridRegion,
	leaf_min: IVec3,
	leaf_end: IVec3,
	value: MarchingVoxel,
) {
	let shell_min = leaf_min - IVec3::ONE;
	let shell_max = leaf_end - IVec3::ONE;
	let min = shell_min.max(cell_region.min);
	let max = shell_max.min(cell_region.end - IVec3::ONE);
	if min.cmpgt(max).any() { return; }

	for x in [shell_min.x, shell_max.x] {
		if x < min.x || x > max.x { continue; }
		for z in min.z..=max.z {
			for y in min.y..=max.y {
				stage_cell(cells, IVec3::new(x, y, z), leaf_min, leaf_end, value);
			}
		}
	}

	let inner_x_min = min.x.max(shell_min.x + 1);
	let inner_x_max = max.x.min(shell_max.x - 1);
	if inner_x_min <= inner_x_max {
		for y in [shell_min.y, shell_max.y] {
			if y < min.y || y > max.y { continue; }
			for z in min.z..=max.z {
				for x in inner_x_min..=inner_x_max {
					stage_cell(cells, IVec3::new(x, y, z), leaf_min, leaf_end, value);
				}
			}
		}
	}

	let inner_y_min = min.y.max(shell_min.y + 1);
	let inner_y_max = max.y.min(shell_max.y - 1);
	if inner_x_min <= inner_x_max && inner_y_min <= inner_y_max {
		for z in [shell_min.z, shell_max.z] {
			if z < min.z || z > max.z { continue; }
			for y in inner_y_min..=inner_y_max {
				for x in inner_x_min..=inner_x_max {
					stage_cell(cells, IVec3::new(x, y, z), leaf_min, leaf_end, value);
				}
			}
		}
	}
}

fn stage_cell(
	cells: &mut FxHashMap<IVec3, CellSamples>,
	origin: IVec3,
	leaf_min: IVec3,
	leaf_end: IVec3,
	value: MarchingVoxel,
) {
	let samples = cells.entry(origin).or_default();
	for (index, corner) in CORNERS.iter().enumerate() {
		let position = origin + *corner;
		if position.cmpge(leaf_min).all() && position.cmplt(leaf_end).all() {
			samples.colors[index] = u32::from_le_bytes(value.0.color);
			samples.occupied |= 1 << index;
		}
	}
}

fn polygonize_cell(
	origin: IVec3,
	output_offset: IVec3,
	samples: CellSamples,
	earcut: &mut EarcutI32,
	triangle_indices: &mut Vec<u16>,
	out: &mut Vec<MarchingTriangle>,
) {
	let occupied = std::array::from_fn(|index| samples.occupied & (1 << index) != 0);
	let occupied_count = samples.occupied.count_ones();
	if occupied_count == 0 || occupied_count == 8 { return; }

	let crossing = EDGES.map(|(a, b)| occupied[a] != occupied[b]);
	let mut adjacency = [0u16; 12];
	for (corners, edges) in FACES {
		let crossing_count = edges.iter().filter(|edge| crossing[**edge]).count();
		match crossing_count {
			2 => {
				let mut pair = edges.into_iter().filter(|edge| crossing[*edge]);
				connect(&mut adjacency, pair.next().unwrap(), pair.next().unwrap());
			}
			4 => {
				// Resolve checkerboard faces by connecting the solid side.
				for index in 0..4 {
					if !occupied[corners[index]] {
						connect(&mut adjacency, edges[(index + 3) % 4], edges[index]);
					}
				}
			}
			_ => {}
		}
	}

	let color = average_color(&samples);
	let packed_origin = pack_origin(origin - output_offset);
	let mut visited = [false; 12];
	for start in 0..12 {
		if !crossing[start] || visited[start] || adjacency[start] == 0 { continue; }
		let mut polygon = [0usize; 12];
		let mut polygon_len = 0;
		let mut previous = usize::MAX;
		let mut current = start;
		loop {
			if visited[current] { break; }
			visited[current] = true;
			polygon[polygon_len] = current;
			polygon_len += 1;
			let next_mask = if previous < 12 {
				adjacency[current] & !(1u16 << previous)
			} else {
				adjacency[current]
			};
			if next_mask == 0 { break; }
			previous = current;
			current = next_mask.trailing_zeros() as usize;
			if current == start { break; }
		}
		if polygon_len < 3 || current != start { continue; }
		let normal = outward_polygon_normal(&polygon, polygon_len, &occupied);
		let (projection_x, projection_y) = projection_basis(normal);
		earcut.earcut(
			polygon[..polygon_len].iter().map(|&edge| {
				let point = EDGE_MIDPOINTS[edge];
				[point.dot(projection_x), point.dot(projection_y)]
			}),
			&[] as &[u16],
			triangle_indices,
		);
		let reverse_winding = triangle_indices.chunks_exact(3).next().is_some_and(|triangle| {
			let edges = [
				polygon[triangle[0] as usize],
				polygon[triangle[1] as usize],
				polygon[triangle[2] as usize],
			];
			let triangle_normal = (EDGE_MIDPOINTS[edges[1]] - EDGE_MIDPOINTS[edges[0]])
				.cross(EDGE_MIDPOINTS[edges[2]] - EDGE_MIDPOINTS[edges[0]]);
			triangle_normal.dot(normal) < 0
		});
		for triangle in triangle_indices.chunks_exact(3) {
			let mut edges = [
				polygon[triangle[0] as usize],
				polygon[triangle[1] as usize],
				polygon[triangle[2] as usize],
			];
			if reverse_winding { edges.swap(1, 2); }
			push_triangle(out, packed_origin, edges, color);
		}
	}
}

fn connect(adjacency: &mut [u16; 12], a: usize, b: usize) {
	adjacency[a] |= 1u16 << b;
	adjacency[b] |= 1u16 << a;
}

fn average_color(samples: &CellSamples) -> u32 {
	let mut sum = [0u32; 4];
	for (index, color) in samples.colors.iter().enumerate() {
		if samples.occupied & (1 << index) == 0 { continue; }
		for (total, channel) in sum.iter_mut().zip(color.to_le_bytes()) {
			*total += u32::from(channel);
		}
	}
	let count = samples.occupied.count_ones();
	let channels = sum.map(|total| (total / count) as u8);
	u32::from_le_bytes(channels)
}

fn outward_polygon_normal(polygon: &[usize; 12], polygon_len: usize, occupied: &[bool; 8]) -> IVec3 {
	let mut normal = IVec3::ZERO;
	for index in 0..polygon_len {
		let current = EDGE_MIDPOINTS[polygon[index]];
		let next = EDGE_MIDPOINTS[polygon[(index + 1) % polygon_len]];
		normal += current.cross(next);
	}

	let mut orientation: i32 = 0;
	for &edge in &polygon[..polygon_len] {
		let (a, b) = EDGES[edge];
		let toward_empty = if occupied[a] { CORNERS[b] - CORNERS[a] } else { CORNERS[a] - CORNERS[b] };
		let candidate = normal.dot(toward_empty);
		if candidate.abs() > orientation.abs() { orientation = candidate; }
	}
	if orientation < 0 { -normal } else { normal }
}

fn projection_basis(normal: IVec3) -> (IVec3, IVec3) {
	let absolute = normal.abs();
	let helper = if absolute.x <= absolute.y && absolute.x <= absolute.z {
		IVec3::X
	} else if absolute.y <= absolute.z {
		IVec3::Y
	} else {
		IVec3::Z
	};
	let x = normal.cross(helper);
	let y = normal.cross(x);
	(x, y)
}

fn pack_origin(origin: IVec3) -> [u32; 2] {
	let x = u16::try_from(origin.x).expect("marching triangle origin x exceeds u16") as u32;
	let y = u16::try_from(origin.y).expect("marching triangle origin y exceeds u16") as u32;
	let z = u16::try_from(origin.z).expect("marching triangle origin z exceeds u16") as u32;
	[x | (y << 16), z]
}

fn push_triangle(out: &mut Vec<MarchingTriangle>, packed_origin: [u32; 2], edges: [usize; 3], color: u32) {
	let packed_edges = edges[0] as u32 | (edges[1] as u32) << 4 | (edges[2] as u32) << 8;
	out.push(MarchingTriangle {
		packed_origin_xy: packed_origin[0],
		packed_origin_z_edges: packed_origin[1] | (packed_edges << 16),
		color,
	});
}

#[cfg(test)]
mod tests {
	use super::*;
	use std::collections::{HashMap, HashSet};

	use basic_voxel::BasicVoxel;
	use bevy::math::U16Vec3;

	fn triangulate_case(occupied: u8) -> Vec<[usize; 3]> {
		let samples = CellSamples { colors: [u32::MAX; 8], occupied };
		let mut earcut = EarcutI32::new();
		let mut triangle_indices = Vec::new();
		let mut triangles = Vec::new();
		polygonize_cell(
			IVec3::ZERO,
			IVec3::ZERO,
			samples,
			&mut earcut,
			&mut triangle_indices,
			&mut triangles,
		);
		triangles.into_iter().map(|triangle| {
			let packed = triangle.packed_origin_z_edges >> 16;
			[
				(packed & 0xf) as usize,
				((packed >> 4) & 0xf) as usize,
				((packed >> 8) & 0xf) as usize,
			]
		}).collect()
	}

	fn case_adjacency(occupied: u8) -> ([bool; 12], [u16; 12]) {
		let corners: [bool; 8] = std::array::from_fn(|index| occupied & (1 << index) != 0);
		let crossing = EDGES.map(|(a, b)| corners[a] != corners[b]);
		let mut adjacency = [0u16; 12];
		for (face_corners, face_edges) in FACES {
			let crossing_count = face_edges.iter().filter(|edge| crossing[**edge]).count();
			match crossing_count {
				2 => {
					let mut pair = face_edges.into_iter().filter(|edge| crossing[*edge]);
					connect(&mut adjacency, pair.next().unwrap(), pair.next().unwrap());
				}
				4 => {
					for index in 0..4 {
						if !corners[face_corners[index]] {
							connect(&mut adjacency, face_edges[(index + 3) % 4], face_edges[index]);
						}
					}
				}
				_ => {}
			}
		}
		(crossing, adjacency)
	}

	fn sorted_pair(a: usize, b: usize) -> (usize, usize) {
		if a < b { (a, b) } else { (b, a) }
	}

	fn directly_sample_cell(voxels: &Voxels, origin: IVec3) -> CellSamples {
		let mut samples = CellSamples::default();
		for (index, corner) in CORNERS.iter().enumerate() {
			let position = origin + *corner;
			if position.is_negative_bitmask() != 0 || position.cmpgt(IVec3::splat(u16::MAX as i32)).any() {
				continue;
			}
			let Some(voxel) = voxels.voxel(&position.as_u16vec3()) else { continue };
			let voxel = MarchingVoxel::from_voxel_ref(&voxel);
			samples.colors[index] = u32::from_le_bytes(voxel.0.color);
			samples.occupied |= 1 << index;
		}
		samples
	}

	#[test]
	fn checkerboard_faces_connect_the_solid_side() {
		let triangles = triangulate_case((1 << 0) | (1 << 2));
		let mut edge_counts = HashMap::new();
		for [a, b, c] in triangles {
			for edge in [sorted_pair(a, b), sorted_pair(b, c), sorted_pair(c, a)] {
				*edge_counts.entry(edge).or_insert(0) += 1;
			}
		}
		assert_eq!(edge_counts.get(&(0, 1)), Some(&1));
		assert_eq!(edge_counts.get(&(2, 3)), Some(&1));
		assert_ne!(edge_counts.get(&(0, 3)), Some(&1));
		assert_ne!(edge_counts.get(&(1, 2)), Some(&1));
	}

	#[test]
	fn staged_tree_leaves_match_direct_voxel_samples() {
		let red = MarchingVoxel(BasicVoxel { color: [255, 0, 0, 255], mass: 1 });
		let green = MarchingVoxel(BasicVoxel { color: [0, 255, 0, 255], mass: 2 });
		let blue = MarchingVoxel(BasicVoxel { color: [0, 0, 255, 255], mass: 3 });
		let mut voxels = Voxels::new::<MarchingVoxel>();
		voxels.add_area(U16Vec3::new(2, 3, 4), U16Vec3::new(7, 5, 6), red.get_ref());
		voxels.add_area(U16Vec3::new(5, 1, 6), U16Vec3::new(4, 8, 3), green.get_ref());
		voxels.remove_area(U16Vec3::new(6, 4, 6), U16Vec3::new(2, 2, 2));
		voxels.add_voxel(U16Vec3::new(0, 0, 0), blue.get_ref());
		voxels.add_voxel(U16Vec3::new(12, 11, 10), blue.get_ref());

		let region = GridRegion { min: IVec3::splat(-1), end: IVec3::splat(15) };
		let staged = stage_cells(&voxels, region);
		for z in region.min.z..region.end.z {
			for y in region.min.y..region.end.y {
				for x in region.min.x..region.end.x {
					let origin = IVec3::new(x, y, z);
					let expected = directly_sample_cell(&voxels, origin);
					if expected.occupied == 0 || expected.occupied == u8::MAX {
						if let Some(actual) = staged.get(&origin) {
							assert_eq!(*actual, expected, "cell {origin} disagrees with direct sampling");
						}
					} else {
						assert_eq!(staged.get(&origin), Some(&expected), "surface cell {origin} disagrees with direct sampling");
					}
				}
			}
		}
	}

	#[test]
	fn sparse_tree_mesh_matches_direct_sampling() {
		let red = MarchingVoxel(BasicVoxel { color: [255, 0, 0, 255], mass: 1 });
		let green = MarchingVoxel(BasicVoxel { color: [0, 255, 0, 255], mass: 2 });
		let mut voxels = Voxels::new::<MarchingVoxel>();
		voxels.add_area(U16Vec3::new(2, 3, 4), U16Vec3::new(7, 5, 6), red.get_ref());
		voxels.add_area(U16Vec3::new(5, 1, 6), U16Vec3::new(4, 8, 3), green.get_ref());
		voxels.remove_area(U16Vec3::new(6, 4, 6), U16Vec3::new(2, 2, 2));

		let cell_min = IVec3::splat(-1);
		let cell_size = IVec3::splat(15);
		let (actual, _, _, _) = make_marching_cubes_mesh_in_region(&voxels, cell_min, cell_size);
		let mut expected = Vec::new();
		let mut earcut = EarcutI32::new();
		let mut triangle_indices = Vec::new();
		let cell_end = cell_min + cell_size;
		for z in cell_min.z..cell_end.z {
			for y in cell_min.y..cell_end.y {
				for x in cell_min.x..cell_end.x {
					let origin = IVec3::new(x, y, z);
					polygonize_cell(
						origin,
						cell_min,
						directly_sample_cell(&voxels, origin),
						&mut earcut,
						&mut triangle_indices,
						&mut expected,
					);
				}
			}
		}
		assert_eq!(actual, expected);
	}

	#[test]
	fn every_marching_case_produces_valid_edge_triangles() {
		for occupied in 1..u8::MAX {
			let triangles = triangulate_case(occupied);
			assert!(!triangles.is_empty(), "case {occupied:#010b} produced no triangles");
			for edges in triangles {
				for &edge in &edges {
					assert!(edge < EDGES.len(), "case {occupied:#010b} emitted invalid edge {edge}");
					let (a, b) = EDGES[edge];
					assert_ne!(occupied & (1 << a), occupied & (1 << b));
				}
				let normal = (EDGE_MIDPOINTS[edges[1]] - EDGE_MIDPOINTS[edges[0]])
					.cross(EDGE_MIDPOINTS[edges[2]] - EDGE_MIDPOINTS[edges[0]]);
				assert_ne!(normal, IVec3::ZERO, "case {occupied:#010b} emitted a degenerate triangle");
			}
		}
	}

	#[test]
	fn every_marching_case_has_consistent_internal_winding() {
		for occupied in 1..u8::MAX {
			let triangles = triangulate_case(occupied);
			let mut directed_edges = HashMap::<_, (usize, i32)>::new();
			for [a, b, c] in triangles {
				for (from, to) in [(a, b), (b, c), (c, a)] {
					let entry = directed_edges.entry(sorted_pair(from, to)).or_default();
					entry.0 += 1;
					entry.1 += if from < to { 1 } else { -1 };
				}
			}
			for (edge, (count, direction)) in directed_edges {
				if count == 2 {
					assert_eq!(direction, 0, "case {occupied:#010b} winds internal edge {edge:?} in one direction");
				}
			}
		}
	}

	#[test]
	fn adjacent_cells_wind_shared_face_edges_oppositely() {
		fn point_key(point: IVec3) -> [i32; 3] { [point.x, point.y, point.z] }
		fn pair_key(a: IVec3, b: IVec3) -> ([i32; 3], [i32; 3], i32) {
			let a = point_key(a);
			let b = point_key(b);
			if a < b { (a, b, 1) } else { (b, a, -1) }
		}

		for samples in 0u16..1 << 12 {
			let occupied = |x: usize, y: usize, z: usize| samples & (1 << (x * 4 + y * 2 + z)) != 0;
			let mut left_mask = 0u8;
			let mut right_mask = 0u8;
			for (corner, position) in CORNERS.iter().enumerate() {
				if occupied(position.x as usize, position.y as usize, position.z as usize) {
					left_mask |= 1 << corner;
				}
				if occupied(position.x as usize + 1, position.y as usize, position.z as usize) {
					right_mask |= 1 << corner;
				}
			}

			let mut shared_edges = HashMap::<_, (usize, i32)>::new();
			for (origin, mask) in [(IVec3::ZERO, left_mask), (IVec3::new(2, 0, 0), right_mask)] {
				for [a, b, c] in triangulate_case(mask) {
					for (from, to) in [(a, b), (b, c), (c, a)] {
						let from = EDGE_MIDPOINTS[from] + origin;
						let to = EDGE_MIDPOINTS[to] + origin;
						if from.x != 2 || to.x != 2 { continue; }
						let (a, b, direction) = pair_key(from, to);
						let entry = shared_edges.entry((a, b)).or_default();
						entry.0 += 1;
						entry.1 += direction;
					}
				}
			}
			for (edge, (count, direction)) in shared_edges {
				assert_eq!(count % 2, 0, "sample pattern {samples:#014b} leaves unmatched shared face edge {edge:?}");
				assert_eq!(direction, 0, "sample pattern {samples:#014b} winds shared face edge {edge:?} in one direction");
			}
		}
	}

	#[test]
	fn every_marching_case_preserves_polygon_boundaries() {
		for occupied in 1..u8::MAX {
			let (crossing, adjacency) = case_adjacency(occupied);
			let expected_boundary: HashSet<_> = adjacency.iter().enumerate().flat_map(|(a, connections)| {
				(0..EDGES.len()).filter_map(move |b| {
					(a < b && connections & (1 << b) != 0).then_some((a, b))
				})
			}).collect();
			let triangles = triangulate_case(occupied);
			let mut edge_counts = HashMap::<_, usize>::new();
			let mut used_vertices = HashSet::new();
			for [a, b, c] in &triangles {
				used_vertices.extend([*a, *b, *c]);
				for pair in [sorted_pair(*a, *b), sorted_pair(*b, *c), sorted_pair(*c, *a)] {
					*edge_counts.entry(pair).or_default() += 1;
				}
			}

			let expected_vertices: HashSet<_> = crossing.iter().enumerate()
				.filter_map(|(edge, crossing)| crossing.then_some(edge))
				.collect();
			assert_eq!(used_vertices, expected_vertices, "case {occupied:#010b} omitted polygon vertices");
			for boundary in &expected_boundary {
				assert_eq!(edge_counts.get(boundary), Some(&1), "case {occupied:#010b} changed boundary edge {boundary:?}");
			}
			for (edge, count) in edge_counts {
				let expected = if expected_boundary.contains(&edge) { 1 } else { 2 };
				assert_eq!(count, expected, "case {occupied:#010b} has non-manifold edge {edge:?}");
			}
		}
	}
}
