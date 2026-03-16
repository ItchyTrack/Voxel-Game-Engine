use glam::{Vec3, Vec4};
use std::cell::RefCell;

use crate::pose::Pose;

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub struct DebugVertex {
	pub position: [f32; 3],
	pub color: [f32; 4],
}

impl DebugVertex {
	const ATTRIBS: [wgpu::VertexAttribute; 2] = wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x4];
	pub fn desc() -> wgpu::VertexBufferLayout<'static> {
		wgpu::VertexBufferLayout {
			array_stride: std::mem::size_of::<DebugVertex>() as wgpu::BufferAddress,
			step_mode: wgpu::VertexStepMode::Vertex,
			attributes: &Self::ATTRIBS,
		}
	}
}

pub struct DebugBatch {
	pub lines: Vec<DebugVertex>, // pairs of vertices
	pub triangles: Vec<DebugVertex>, // triples of vertices
}

impl DebugBatch {
	fn new() -> Self {
		Self {
			lines: Vec::with_capacity(4096),
			triangles: Vec::with_capacity(2048),
		}
	}

	fn clear(&mut self) {
		self.lines.clear();
		self.triangles.clear();
	}
}

thread_local! {
	static BUFFER: RefCell<DebugBatch> = RefCell::new(DebugBatch::new());
}

pub fn point(pos: Vec3, color: &Vec4, size: f32) {
	let s = size * 0.5;
	line(pos - Vec3::X * s, pos + Vec3::X * s, &color);
	line(pos - Vec3::Y * s, pos + Vec3::Y * s, &color);
	line(pos - Vec3::Z * s, pos + Vec3::Z * s, &color);
}

pub fn line(a: Vec3, b: Vec3, color: &Vec4) {
	BUFFER.with(|buffer| {
		let mut buffer = buffer.borrow_mut();
		buffer.lines.push(DebugVertex { position: a.to_array(), color: color.to_array() });
		buffer.lines.push(DebugVertex { position: b.to_array(), color: color.to_array() });
	});
}

pub fn aabb(min: Vec3, max: Vec3, color: &Vec4) {
	let corners = [
		min,
		Vec3::new(max.x, min.y, min.z),
		Vec3::new(min.x, max.y, min.z),
		Vec3::new(min.x, min.y, max.z),
		Vec3::new(max.x, max.y, min.z),
		Vec3::new(max.x, min.y, max.z),
		Vec3::new(min.x, max.y, max.z),
		max
	];
	let edges = [
		(0, 1), (0, 2), (0, 3),
		(1, 4), (1, 5),
		(2, 4), (2, 6),
		(3, 5), (3, 6),
		(4, 7),
		(5, 7),
		(6, 7)
	];
	for (i, j) in edges {
		line(corners[i], corners[j], color);
	}
}

pub fn quad(a: Vec3, b: Vec3, c: Vec3, d: Vec3, color: &Vec4, filled: bool) {
	if filled {
		BUFFER.with(|buf| {
			let mut buf = buf.borrow_mut();
			for &vertex in &[a, b, c, a, c, d] {
				buf.triangles.push(DebugVertex { position: vertex.to_array(), color: color.to_array() });
			}
		});
	} else {
		line(a, b, color);
		line(b, c, color);
		line(c, d, color);
		line(d, a, color);
	}
}

pub fn rectangular_prism(pose: &Pose, size: Vec3, color: &Vec4, filled: bool) {
	rectangular_prism_from_vec(&pose.translation, &(pose.rotation * Vec3::X * size.x, pose.rotation * Vec3::Y * size.y, pose.rotation * Vec3::Z * size.z), color, filled);
	// quad(
	// 	pose.translation,
	// 	pose * Vec3::X * size.x,
	// 	pose * (Vec3::X * size.x + Vec3::Y * size.y),
	// 	pose * Vec3::Y * size.y,
	// 	color,
	// 	filled
	// );
	// quad(
	// 	pose.translation,
	// 	pose * Vec3::X * size.x,
	// 	pose * (Vec3::X * size.x + Vec3::Z * size.z),
	// 	pose * Vec3::Z * size.z,
	// 	color,
	// 	filled
	// );
	// quad(
	// 	pose.translation,
	// 	pose * Vec3::Y * size.y,
	// 	pose * (Vec3::Y * size.y + Vec3::Z * size.z),
	// 	pose * Vec3::Z * size.z,
	// 	color,
	// 	filled
	// );
	// quad(
	// 	pose * size,
	// 	pose * (Vec3::Z * size.z + Vec3::Y * size.y),
	// 	pose * Vec3::Z * size.z,
	// 	pose * (Vec3::Z * size.z + Vec3::X * size.x),
	// 	color,
	// 	filled
	// );
	// quad(
	// 	pose * size,
	// 	pose * (Vec3::Z * size.z + Vec3::Y * size.y),
	// 	pose * Vec3::Y * size.y,
	// 	pose * (Vec3::X * size.x + Vec3::Y * size.y),
	// 	color,
	// 	filled
	// );
	// quad(
	// 	pose * size,
	// 	pose * (Vec3::Z * size.z + Vec3::X * size.x),
	// 	pose * Vec3::X * size.x,
	// 	pose * (Vec3::X * size.x + Vec3::Y * size.y),
	// 	color,
	// 	filled
	// );
}

pub fn rectangular_prism_from_vec(pos: &Vec3, size: &(Vec3, Vec3, Vec3), color: &Vec4, filled: bool) {
	quad(
		*pos,
		pos + size.0,
		pos + size.0 + size.1,
		pos + size.1,
		color,
		filled
	);
	quad(
		*pos,
		pos + size.0,
		pos + size.0 + size.2,
		pos + size.2,
		color,
		filled
	);
	quad(
		*pos,
		pos + size.1,
		pos + (size.1 + size.2),
		pos + size.2,
		color,
		filled
	);
	quad(
		pos + size.0 + size.1 + size.2,
		pos + size.2 + size.1,
		pos + size.2,
		pos + size.2 + size.0,
		color,
		filled
	);
	quad(
		pos + size.0 + size.1 + size.2,
		pos + size.2 + size.1,
		pos + size.1,
		pos + size.0 + size.1,
		color,
		filled
	);
	quad(
		pos + size.0 + size.1 + size.2,
		pos + size.2 + size.0,
		pos + size.0,
		pos + size.0 + size.1,
		color,
		filled
	);
}

pub fn take_batch() -> DebugBatch {
	BUFFER.with(|buf| {
		let mut buf = buf.borrow_mut();
		let mut taken = DebugBatch::new();
		std::mem::swap(&mut taken, &mut *buf);
		taken
	})
}
