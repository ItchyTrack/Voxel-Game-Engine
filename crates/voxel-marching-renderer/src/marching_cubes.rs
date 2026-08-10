use basic_voxel::MarchingVoxel;
use bevy::math::{IVec3, U16Vec3, Vec3};
use voxel_data::voxels::{VoxelType, Voxels};

#[repr(C)]
#[derive(Clone, Copy, Debug, bytemuck::Pod, bytemuck::Zeroable)]
pub struct MarchingVertex {
	pub position: [f32; 3],
	pub _position_padding: f32,
	pub normal: [f32; 3],
	pub color: u32,
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

// Face corners and the edge following each corner around that face.
const FACES: [([usize; 4], [usize; 4]); 6] = [
	([0, 1, 2, 3], [0, 1, 2, 3]),
	([4, 5, 6, 7], [4, 5, 6, 7]),
	([0, 1, 5, 4], [0, 9, 4, 8]),
	([3, 2, 6, 7], [2, 10, 6, 11]),
	([0, 3, 7, 4], [3, 11, 7, 8]),
	([1, 2, 6, 5], [1, 10, 5, 9]),
];

pub fn make_marching_cubes_mesh(voxels: &Voxels) -> (Vec<u8>, u32, Vec3, Vec3) {
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
) -> (Vec<u8>, u32, Vec3, Vec3) {
	if cell_size.cmple(IVec3::ZERO).any() {
		return (Vec::new(), 0, Vec3::ZERO, Vec3::ZERO);
	}
	let cell_end = cell_min + cell_size;
	let mut vertices = Vec::new();
	for z in cell_min.z..cell_end.z {
		for y in cell_min.y..cell_end.y {
			for x in cell_min.x..cell_end.x {
				polygonize_cell(voxels, IVec3::new(x, y, z), cell_min, &mut vertices);
			}
		}
	}

	let vertex_count = vertices.len() as u32;
	(bytemuck::cast_slice(&vertices).to_vec(), vertex_count, Vec3::ZERO, cell_size.as_vec3())
}

fn polygonize_cell(voxels: &Voxels, origin: IVec3, output_offset: IVec3, out: &mut Vec<MarchingVertex>) {
	let samples = CORNERS.map(|corner| sample(voxels, origin + corner));
	let occupied = samples.map(|sample| sample.is_some());
	let occupied_count = occupied.iter().filter(|value| **value).count();
	if occupied_count == 0 || occupied_count == 8 { return; }

	let crossing = EDGES.map(|(a, b)| occupied[a] != occupied[b]);
	let mut adjacency: [Vec<usize>; 12] = std::array::from_fn(|_| Vec::with_capacity(2));
	for (corners, edges) in FACES {
		let crossing_count = edges.iter().filter(|edge| crossing[**edge]).count();
		match crossing_count {
			2 => {
				let mut pair = edges.into_iter().filter(|edge| crossing[*edge]);
				connect(&mut adjacency, pair.next().unwrap(), pair.next().unwrap());
			}
			4 => {
				// Resolve the checkerboard face by tracing around each occupied corner.
				for index in 0..4 {
					if occupied[corners[index]] {
						connect(&mut adjacency, edges[(index + 3) % 4], edges[index]);
					}
				}
			}
			_ => {}
		}
	}

	let color = average_color(&samples);
	let outward = outward_direction(&occupied);
	let edge_points = EDGES.map(|(a, b)| {
		(origin - output_offset).as_vec3() + (CORNERS[a] + CORNERS[b]).as_vec3() * 0.5
	});
	let mut visited = [false; 12];
	for start in 0..12 {
		if !crossing[start] || visited[start] || adjacency[start].is_empty() { continue; }
		let mut polygon = Vec::new();
		let mut previous = usize::MAX;
		let mut current = start;
		loop {
			if visited[current] { break; }
			visited[current] = true;
			polygon.push(edge_points[current]);
			let Some(next) = adjacency[current].iter().copied().find(|next| *next != previous) else { break };
			previous = current;
			current = next;
			if current == start { break; }
		}
		if polygon.len() < 3 || current != start { continue; }
		let center = polygon.iter().copied().sum::<Vec3>() / polygon.len() as f32;
		for index in 0..polygon.len() {
			let mut a = polygon[index];
			let mut b = polygon[(index + 1) % polygon.len()];
			let mut normal = (a - center).cross(b - center).normalize_or_zero();
			if normal.dot(outward) < 0.0 {
				std::mem::swap(&mut a, &mut b);
				normal = (a - center).cross(b - center).normalize_or_zero();
			}
			push_vertex(out, center, normal, color);
			push_vertex(out, a, normal, color);
			push_vertex(out, b, normal, color);
		}
	}
}

fn connect(adjacency: &mut [Vec<usize>; 12], a: usize, b: usize) {
	if !adjacency[a].contains(&b) { adjacency[a].push(b); }
	if !adjacency[b].contains(&a) { adjacency[b].push(a); }
}

fn sample(voxels: &Voxels, position: IVec3) -> Option<MarchingVoxel> {
	if position.cmplt(IVec3::ZERO).any() || position.cmpgt(IVec3::splat(u16::MAX as i32)).any() {
		return None;
	}
	voxels.voxel(&U16Vec3::new(position.x as u16, position.y as u16, position.z as u16))
		.map(|voxel| MarchingVoxel::from_voxel_ref(&voxel))
}

fn average_color(samples: &[Option<MarchingVoxel>; 8]) -> u32 {
	let mut sum = [0u32; 4];
	let mut count = 0;
	for sample in samples.iter().flatten() {
		for (total, channel) in sum.iter_mut().zip(sample.0.color) { *total += u32::from(channel); }
		count += 1;
	}
	let channels = sum.map(|total| (total / count) as u8);
	u32::from_le_bytes(channels)
}

fn outward_direction(occupied: &[bool; 8]) -> Vec3 {
	let mut solid = Vec3::ZERO;
	let mut empty = Vec3::ZERO;
	let mut solid_count = 0.0;
	let mut empty_count = 0.0;
	for (index, corner) in CORNERS.iter().enumerate() {
		if occupied[index] {
			solid += corner.as_vec3();
			solid_count += 1.0;
		} else {
			empty += corner.as_vec3();
			empty_count += 1.0;
		}
	}
	(empty / empty_count - solid / solid_count).normalize_or_zero()
}

fn push_vertex(out: &mut Vec<MarchingVertex>, position: Vec3, normal: Vec3, color: u32) {
	out.push(MarchingVertex {
		position: position.to_array(),
		_position_padding: 0.0,
		normal: normal.to_array(),
		color,
	});
}
