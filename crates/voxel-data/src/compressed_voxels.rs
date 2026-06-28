use std::io::Cursor;
use std::ops::Deref;

use crate::voxels::Voxels;

const RAW_ENCODING: u8 = 0;
const ZSTD_ENCODING: u8 = 1;

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct CompressedVoxels(Vec<u8>);

impl CompressedVoxels {
	pub fn new(voxels: &Voxels, compression: i32) -> Result<Self, CompressVoxelsError> {
		if compression <= 0 {
			let mut bytes = vec![RAW_ENCODING];
			voxels.write_to(&mut bytes).map_err(CompressVoxelsError::Serialize)?;
			return Ok(Self(bytes));
		}

		let mut encoder = zstd::stream::Encoder::new(Vec::new(), compression)
			.map_err(CompressVoxelsError::Compress)?;
		voxels.write_to(&mut encoder).map_err(CompressVoxelsError::Serialize)?;
		let mut bytes = vec![ZSTD_ENCODING];
		bytes.extend(encoder.finish().map_err(CompressVoxelsError::Compress)?);
		Ok(Self(bytes))
	}

	pub fn with_max_compression(voxels: &Voxels) -> Result<Self, CompressVoxelsError> {
		Self::new(voxels, 6)
	}

	pub fn decompress(&self) -> Result<Voxels, DecompressVoxelsError> {
		let Some((&encoding, payload)) = self.0.split_first() else {
			return Err(DecompressVoxelsError::Deserialize(std::io::Error::new(
				std::io::ErrorKind::UnexpectedEof,
				"compressed voxels payload is empty",
			)));
		};
		match encoding {
			RAW_ENCODING => Voxels::read_from(&mut Cursor::new(payload)).map_err(DecompressVoxelsError::Deserialize),
			ZSTD_ENCODING => {
				let mut decoder = zstd::stream::Decoder::new(Cursor::new(payload))
					.map_err(DecompressVoxelsError::Decompress)?;
				Voxels::read_from(&mut decoder).map_err(DecompressVoxelsError::Deserialize)
			}
			_ => Err(DecompressVoxelsError::Deserialize(std::io::Error::new(
				std::io::ErrorKind::InvalidData,
				"unknown compressed voxels encoding",
			))),
		}
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

#[derive(Debug)]
pub enum CompressVoxelsError {
	Serialize(std::io::Error),
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
	Deserialize(std::io::Error),
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
