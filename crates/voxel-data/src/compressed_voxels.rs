use std::ops::Deref;

use crate::voxels::Voxels;

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct CompressedVoxels(Vec<u8>);

impl CompressedVoxels {
	pub fn new(voxels: &Voxels) -> Result<Self, bincode::Error> {
		let raw = bincode::serialize(voxels)?;
		Ok(Self(lz4_flex::compress_prepend_size(&raw)))
	}

	pub fn from_voxels(voxels: Voxels) -> Result<Self, bincode::Error> {
		Self::new(&voxels)
	}

	pub fn decompress(&self) -> Result<Voxels, DecompressVoxelsError> {
		let raw = lz4_flex::decompress_size_prepended(&self.0)
			.map_err(DecompressVoxelsError::Decompress)?;
		bincode::deserialize(&raw).map_err(DecompressVoxelsError::Deserialize)
	}

	pub fn compressed_bytes(&self) -> &[u8] {
		&self.0
	}

	pub fn into_compressed_bytes(self) -> Vec<u8> {
		self.0
	}

	pub fn len(&self) -> usize {
		self.0.len()
	}

	pub fn is_empty(&self) -> bool {
		self.0.is_empty()
	}
}

impl Deref for CompressedVoxels {
	type Target = [u8];

	fn deref(&self) -> &Self::Target {
		&self.0
	}
}

impl TryFrom<&Voxels> for CompressedVoxels {
	type Error = bincode::Error;

	fn try_from(value: &Voxels) -> Result<Self, Self::Error> {
		Self::new(value)
	}
}

impl TryFrom<Voxels> for CompressedVoxels {
	type Error = bincode::Error;

	fn try_from(value: Voxels) -> Result<Self, Self::Error> {
		Self::from_voxels(value)
	}
}

#[derive(Debug)]
pub enum DecompressVoxelsError {
	Decompress(lz4_flex::block::DecompressError),
	Deserialize(bincode::Error),
}

impl std::fmt::Display for DecompressVoxelsError {
	fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
		match self {
			Self::Decompress(err) => write!(f, "failed to decompress voxels: {err}"),
			Self::Deserialize(err) => write!(f, "failed to deserialize voxels: {err}"),
		}
	}
}

impl std::error::Error for DecompressVoxelsError {}
