use std::io::Cursor;
use std::ops::Deref;

use crate::voxels::Voxels;

const ZSTD_LEVEL: i32 = 6;

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct CompressedVoxels(Vec<u8>);

impl CompressedVoxels {
	pub fn new(voxels: &Voxels) -> Result<Self, CompressVoxelsError> {
		let raw = bincode::serialize(voxels).map_err(CompressVoxelsError::Serialize)?;
		zstd::stream::encode_all(Cursor::new(raw), ZSTD_LEVEL)
			.map(Self)
			.map_err(CompressVoxelsError::Compress)
	}

	pub fn from_voxels(voxels: Voxels) -> Result<Self, CompressVoxelsError> {
		Self::new(&voxels)
	}

	pub fn decompress(&self) -> Result<Voxels, DecompressVoxelsError> {
		let raw = zstd::stream::decode_all(Cursor::new(&self.0))
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
	type Error = CompressVoxelsError;

	fn try_from(value: &Voxels) -> Result<Self, Self::Error> {
		Self::new(value)
	}
}

impl TryFrom<Voxels> for CompressedVoxels {
	type Error = CompressVoxelsError;

	fn try_from(value: Voxels) -> Result<Self, Self::Error> {
		Self::from_voxels(value)
	}
}

#[derive(Debug)]
pub enum CompressVoxelsError {
	Serialize(bincode::Error),
	Compress(std::io::Error),
}

impl std::fmt::Display for CompressVoxelsError {
	fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
		match self {
			Self::Serialize(err) => write!(f, "failed to serialize voxels for compression: {err}"),
			Self::Compress(err) => write!(f, "failed to compress voxels: {err}"),
		}
	}
}

impl std::error::Error for CompressVoxelsError {}

#[derive(Debug)]
pub enum DecompressVoxelsError {
	Decompress(std::io::Error),
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
