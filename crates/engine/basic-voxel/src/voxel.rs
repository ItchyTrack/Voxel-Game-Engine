use rustc_hash::FxHashMap;

use voxel_content::{VoxMaterial, VoxMaterialVoxel};
use voxel_data::voxels::{VoxelType, VoxelTypeId};
use voxel_gpu::{VoxelGpuBlockEncoder, VoxelGpuData, VoxelGpuNodeEntry};
use voxel_mass::VoxelMassValue;

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, bytemuck::Pod, bytemuck::Zeroable)]
pub struct BasicVoxel {
	pub color: [u8; 4],
	pub mass: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, bytemuck::Pod, bytemuck::Zeroable)]
pub struct LodVoxel {
	pub colors: [[u8; 4]; 8],
}

#[repr(transparent)]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, bytemuck::Pod, bytemuck::Zeroable)]
pub struct MarchingVoxel(pub BasicVoxel);

impl LodVoxel {
	pub fn solid(color: [u8; 4]) -> Self {
		Self { colors: [color; 8] }
	}
}

impl VoxelType for BasicVoxel {
	const TYPE_ID: VoxelTypeId = VoxelTypeId(1);
}

impl VoxelType for LodVoxel {
	const TYPE_ID: VoxelTypeId = VoxelTypeId(2);
}

impl VoxelType for MarchingVoxel {
	const TYPE_ID: VoxelTypeId = VoxelTypeId(3);
}

impl VoxMaterialVoxel for BasicVoxel {
	fn from_vox_material(material: VoxMaterial) -> Self {
		Self { color: material.color, mass: 100 }
	}
}

impl VoxMaterialVoxel for MarchingVoxel {
	fn from_vox_material(material: VoxMaterial) -> Self {
		Self(BasicVoxel::from_vox_material(material))
	}
}

const PALETTE_NO_PALETTE: u8 = u8::MAX;
const GPU_HEADER_ALIGNMENT: usize = 16;

fn palette_bits_for_len(len: usize) -> Option<u8> {
	match len {
		0 | 1 => Some(0),
		2 => Some(1),
		3..=4 => Some(2),
		5..=16 => Some(4),
		17..=256 => Some(8),
		257..=65_536 => Some(16),
		_ => None,
	}
}

fn append_palette_header(header: &mut Vec<u8>, bits: u8, palette: &[[u8; 4]]) {
	header.push(bits);
	for color in palette {
		header.extend_from_slice(color);
	}
}

fn append_raw_header(header: &mut Vec<u8>) {
	header.push(PALETTE_NO_PALETTE);
}

fn aligned_header_len(header_len: usize) -> usize {
	header_len.next_multiple_of(GPU_HEADER_ALIGNMENT)
}

fn packed_index_bytes(index_count: usize, bits: u8) -> usize {
	if bits == 0 { 0 } else { (index_count * bits as usize).div_ceil(8) }
}

fn build_palette(colors: impl IntoIterator<Item = [u8; 4]>) -> Option<(Vec<[u8; 4]>, FxHashMap<[u8; 4], u16>, u8)> {
	let mut palette = Vec::new();
	let mut indices = FxHashMap::default();
	for color in colors {
		if indices.contains_key(&color) { continue; }
		if palette.len() == 65_536 { return None; }
		let index = palette.len() as u16;
		palette.push(color);
		indices.insert(color, index);
	}
	let bits = palette_bits_for_len(palette.len())?;
	Some((palette, indices, bits))
}

fn pack_indices(indices: &[u16], bits: u8, out: &mut Vec<u8>) {
	match bits {
		0 => {}
		1 | 2 | 4 => {
			let start = out.len();
			out.resize(start + packed_index_bytes(indices.len(), bits), 0);
			let mask = (1u16 << bits) - 1;
			for (index, value) in indices.iter().copied().enumerate() {
				debug_assert_eq!(value & !mask, 0);
				let bit_offset = index * bits as usize;
				out[start + bit_offset / 8] |= (value as u8) << (bit_offset % 8);
			}
		}
		8 => out.extend(indices.iter().map(|index| *index as u8)),
		16 => {
			for index in indices {
				out.extend_from_slice(&index.to_le_bytes());
			}
		}
		_ => unreachable!("unsupported palette bit width {bits}"),
	}
}

pub enum BasicVoxelGpuEncoder {
	Raw,
	Paletted { indices: FxHashMap<[u8; 4], u16>, bits: u8 },
}

impl VoxelGpuBlockEncoder for BasicVoxelGpuEncoder {
	fn write_node(&self, entries: &[VoxelGpuNodeEntry<'_>], out: &mut Vec<u8>) {
		let Some(last_data_index) = entries
			.iter()
			.rposition(|entry| matches!(entry, VoxelGpuNodeEntry::Data(_))) else { return; };
		match self {
			Self::Raw => {
				let start = out.len();
				out.resize(start + (last_data_index + 1) * 4, 0);
				for (index, entry) in entries.iter().enumerate() {
					let VoxelGpuNodeEntry::Data(voxel) = entry else { continue; };
					let voxel = BasicVoxel::from_voxel_ref(voxel);
					let offset = start + index * 4;
					out[offset..offset + 4].copy_from_slice(&voxel.color);
				}
			}
			Self::Paletted { indices, bits } => {
				if *bits == 0 { return; }
				let mut node_indices = vec![0u16; last_data_index + 1];
				for (index, entry) in entries.iter().enumerate() {
					let VoxelGpuNodeEntry::Data(voxel) = entry else { continue; };
					let voxel = BasicVoxel::from_voxel_ref(voxel);
					node_indices[index] = indices[&voxel.color];
				}
				pack_indices(&node_indices, *bits, out);
			}
		}
	}
}

pub enum LodVoxelGpuEncoder {
	Raw,
	Paletted { indices: FxHashMap<[u8; 4], u16>, bits: u8 },
}

impl VoxelGpuBlockEncoder for LodVoxelGpuEncoder {
	fn write_node(&self, entries: &[VoxelGpuNodeEntry<'_>], out: &mut Vec<u8>) {
		let Some(last_data_index) = entries
			.iter()
			.rposition(|entry| matches!(entry, VoxelGpuNodeEntry::Data(_))) else { return; };
		match self {
			Self::Raw => {
				let start = out.len();
				out.resize(start + (last_data_index + 1) * 32, 0);
				for (index, entry) in entries.iter().enumerate() {
					let VoxelGpuNodeEntry::Data(voxel) = entry else { continue; };
					let voxel = LodVoxel::from_voxel_ref(voxel);
					let offset = start + index * 32;
					out[offset..offset + 32].copy_from_slice(bytemuck::bytes_of(&voxel.colors));
				}
			}
			Self::Paletted { indices, bits } => {
				if *bits == 0 { return; }
				let mut node_indices = vec![0u16; (last_data_index + 1) * 8];
				for (index, entry) in entries.iter().enumerate() {
					let VoxelGpuNodeEntry::Data(voxel) = entry else { continue; };
					let voxel = LodVoxel::from_voxel_ref(voxel);
					for (octant, color) in voxel.colors.iter().copied().enumerate() {
						node_indices[index * 8 + octant] = indices[&color];
					}
				}
				pack_indices(&node_indices, *bits, out);
			}
		}
	}
}

impl VoxelGpuData for BasicVoxel {
	type Encoder = BasicVoxelGpuEncoder;

	fn shader_source() -> &'static str {
		"embedded://basic_voxel/shaders/basic_voxel.slang"
	}

	fn shader_sampler() -> &'static str {
		"BasicVoxelSampler"
	}

	fn create_voxel_gpu_encoder(voxels: &[voxel_data::voxels::VoxelRef<'_>], header: &mut Vec<u8>) -> Self::Encoder {
		let colors = voxels.iter().map(|voxel| BasicVoxel::from_voxel_ref(voxel).color);
		let Some((palette, indices, bits)) = build_palette(colors) else {
			append_raw_header(header);
			return BasicVoxelGpuEncoder::Raw;
		};
		let raw_size = aligned_header_len(1) + voxels.len() * 4;
		let palette_size = aligned_header_len(1 + palette.len() * 4) + packed_index_bytes(voxels.len(), bits);
		if palette_size < raw_size {
			append_palette_header(header, bits, &palette);
			BasicVoxelGpuEncoder::Paletted { indices, bits }
		} else {
			append_raw_header(header);
			BasicVoxelGpuEncoder::Raw
		}
	}
}

impl VoxelGpuData for LodVoxel {
	type Encoder = LodVoxelGpuEncoder;

	fn shader_source() -> &'static str {
		"embedded://basic_voxel/shaders/lod_voxel.slang"
	}

	fn shader_sampler() -> &'static str {
		"LodVoxelSampler"
	}

	fn create_voxel_gpu_encoder(voxels: &[voxel_data::voxels::VoxelRef<'_>], header: &mut Vec<u8>) -> Self::Encoder {
		let colors = voxels
			.iter()
			.flat_map(|voxel| LodVoxel::from_voxel_ref(voxel).colors);
		let Some((palette, indices, bits)) = build_palette(colors) else {
			append_raw_header(header);
			return LodVoxelGpuEncoder::Raw;
		};
		let raw_size = aligned_header_len(1) + voxels.len() * 32;
		let palette_size = aligned_header_len(1 + palette.len() * 4) + packed_index_bytes(voxels.len() * 8, bits);
		if palette_size < raw_size {
			append_palette_header(header, bits, &palette);
			LodVoxelGpuEncoder::Paletted { indices, bits }
		} else {
			append_raw_header(header);
			LodVoxelGpuEncoder::Raw
		}
	}
}

impl VoxelMassValue for BasicVoxel {
	fn voxel_mass(&self) -> u64 { u64::from(self.mass) }
}

impl VoxelMassValue for MarchingVoxel {
	fn voxel_mass(&self) -> u64 { self.0.voxel_mass() }
}
