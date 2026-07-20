use std::{io::{self, Read, Write}, marker::PhantomData};

use serde::{de::{self, SeqAccess, Visitor}, ser::SerializeTuple, Deserialize, Deserializer, Serialize, Serializer};

use super::{raw::RawGridTree, CellKind, GridCoord, GridTree, GridType, SIZE_CUBED, SIZE_USIZE_CUBED};

fn push_u64(out: &mut Vec<u8>, value: u64) { out.extend_from_slice(&value.to_le_bytes()); }
fn write_u8<W: Write>(writer: &mut W, value: u8) -> io::Result<()> { writer.write_all(&[value]) }
fn write_u64<W: Write>(writer: &mut W, value: u64) -> io::Result<()> { writer.write_all(&value.to_le_bytes()) }
fn write_i32<W: Write>(writer: &mut W, value: i32) -> io::Result<()> { writer.write_all(&value.to_le_bytes()) }
fn read_u8<R: Read>(reader: &mut R) -> io::Result<u8> { let mut buf = [0u8; 1]; reader.read_exact(&mut buf)?; Ok(buf[0]) }
fn read_u64_from<R: Read>(reader: &mut R) -> io::Result<u64> { let mut buf = [0u8; 8]; reader.read_exact(&mut buf)?; Ok(u64::from_le_bytes(buf)) }
fn read_i32<R: Read>(reader: &mut R) -> io::Result<i32> { let mut buf = [0u8; 4]; reader.read_exact(&mut buf)?; Ok(i32::from_le_bytes(buf)) }

fn read_u64(input: &mut &[u8]) -> Result<u64, String> {
	if input.len() < 8 { return Err("grid tree byte stream ended early while reading u64".into()); }
	let (head, tail) = input.split_at(8); *input = tail; Ok(u64::from_le_bytes(head.try_into().expect("split_at guarantees 8 bytes")))
}

fn serialize_node_bytes<G: GridType>(raw: &RawGridTree, grid_type: &G, node_index: u32, out: &mut Vec<u8>) {
	let data_mask = raw.data_mask(node_index);
	let node_mask = raw.node_mask(node_index);
	push_u64(out, data_mask);
	push_u64(out, node_mask);
	let data_size = grid_type.data_size_bytes();
	for index in 0..SIZE_USIZE_CUBED {
		if (data_mask & (1u64 << index)) != 0 {
			out.extend_from_slice(&raw.cell_bytes(node_index, index as u8)[..data_size]);
		}
	}
	for index in 0..SIZE_USIZE_CUBED {
		if (node_mask & (1u64 << index)) != 0 {
			serialize_node_bytes(raw, grid_type, raw.child_index(node_index, index as u8), out);
		}
	}
}

fn deserialize_node_bytes<G: GridType>(input: &mut &[u8], raw: &mut RawGridTree, grid_type: &G, parent_index: Option<u32>) -> Result<u32, String> {
	let data_mask = read_u64(input)?;
	let node_mask = read_u64(input)?;
	if data_mask & node_mask != 0 { return Err("grid tree node has overlapping data and node masks".into()); }
	let node_index = if raw.node_count() == 0 { raw.push_empty_node(0) } else { raw.push_empty_node(0) };
	raw.set_parent_offset(node_index, parent_index.map_or(0, |parent| node_index - parent));
	raw.set_used_cell_count(node_index, (data_mask | node_mask).count_ones() as u8);
	let data_size = grid_type.data_size_bytes();
	for index in 0..SIZE_USIZE_CUBED {
		if (data_mask & (1u64 << index)) != 0 {
			if input.len() < data_size { return Err("grid tree byte stream ended early while reading cell data".into()); }
			let (head, tail) = input.split_at(data_size); *input = tail;
			raw.set_data(grid_type, node_index, index as u8, grid_type.read_data(head));
		}
	}
	for index in 0..SIZE_USIZE_CUBED {
		if (node_mask & (1u64 << index)) != 0 {
			let child_index = deserialize_node_bytes(input, raw, grid_type, Some(node_index))?;
			raw.set_child_index(node_index, index as u8, child_index);
		}
	}
	Ok(node_index)
}

fn write_node_to<W: Write, G: GridType>(writer: &mut W, raw: &RawGridTree, grid_type: &G, node_index: u32) -> io::Result<()> {
	let data_mask = raw.data_mask(node_index);
	let node_mask = raw.node_mask(node_index);
	write_u64(writer, data_mask)?;
	write_u64(writer, node_mask)?;
	let data_size = grid_type.data_size_bytes();
	for index in 0..SIZE_USIZE_CUBED {
		if (data_mask & (1u64 << index)) != 0 {
			writer.write_all(&raw.cell_bytes(node_index, index as u8)[..data_size])?;
		}
	}
	for index in 0..SIZE_USIZE_CUBED {
		if (node_mask & (1u64 << index)) != 0 {
			write_node_to(writer, raw, grid_type, raw.child_index(node_index, index as u8))?;
		}
	}
	Ok(())
}

fn raw_occupied_count(raw: &RawGridTree, node_index: u32, node_depth: u8) -> u64 {
	let mut count = 0;
	for child in 0..SIZE_CUBED {
		match raw.cell_kind(node_index, child) {
			CellKind::Empty => {}
			CellKind::Data => {
				let s = super::child_size(node_depth) as u64;
				count += s * s * s;
			}
			CellKind::Node if node_depth == 0 => count += 1,
			CellKind::Node => count += raw_occupied_count(raw, raw.child_index(node_index, child), node_depth - 1),
		}
	}
	count
}

fn read_node_from<R: Read, G: GridType>(reader: &mut R, raw: &mut RawGridTree, grid_type: &G, parent_index: Option<u32>) -> io::Result<u32> {
	let data_mask = read_u64_from(reader)?;
	let node_mask = read_u64_from(reader)?;
	if data_mask & node_mask != 0 { return Err(io::Error::new(io::ErrorKind::InvalidData, "grid tree node has overlapping data and node masks")); }
	let node_index = raw.push_empty_node(0);
	raw.set_parent_offset(node_index, parent_index.map_or(0, |parent| node_index - parent));
	raw.set_used_cell_count(node_index, (data_mask | node_mask).count_ones() as u8);
	let mut data_buf = vec![0u8; grid_type.data_size_bytes()];
	for index in 0..SIZE_USIZE_CUBED {
		if (data_mask & (1u64 << index)) != 0 {
			reader.read_exact(&mut data_buf)?;
			raw.set_data(grid_type, node_index, index as u8, grid_type.read_data(&data_buf));
		}
	}
	for index in 0..SIZE_USIZE_CUBED {
		if (node_mask & (1u64 << index)) != 0 {
			let child_index = read_node_from(reader, raw, grid_type, Some(node_index))?;
			raw.set_child_index(node_index, index as u8, child_index);
		}
	}
	Ok(node_index)
}

impl<G: GridType, Co: GridCoord> GridTree<G, Co> {
	pub fn write_to<W: Write>(&self, writer: &mut W) -> io::Result<()> {
		write_i32(writer, self.raw.root_pos().x)?;
		write_i32(writer, self.raw.root_pos().y)?;
		write_i32(writer, self.raw.root_pos().z)?;
		write_u8(writer, self.raw.root_depth())?;
		write_u64(writer, self.raw.item_count())?;
		write_node_to(writer, &self.raw, &self.grid_type, 0)
	}
}

impl<G: GridType, Co: GridCoord> GridTree<G, Co> {
	pub fn read_from_with_type<R: Read>(grid_type: G, reader: &mut R) -> io::Result<Self> {
		let root_pos = bevy::math::IVec3::new(read_i32(reader)?, read_i32(reader)?, read_i32(reader)?);
		let root_depth = read_u8(reader)?;
		let _stored_item_count = read_u64_from(reader)?;
		if root_depth > Co::MAX_ROOT_DEPTH {
			return Err(io::Error::new(io::ErrorKind::InvalidData, "grid tree root depth exceeds coordinate depth"));
		}
		let mut raw = RawGridTree::new(grid_type.data_size_bytes());
		raw.clear_all_nodes();
		read_node_from(reader, &mut raw, &grid_type, None)?;
		let mut trailing = [0u8; 1];
		if reader.read(&mut trailing)? != 0 {
			return Err(io::Error::new(io::ErrorKind::InvalidData, "grid tree stream has trailing data"));
		}
		raw.set_root(root_pos, root_depth);
		raw.set_item_count(raw_occupied_count(&raw, 0, root_depth));
		Ok(GridTree { grid_type, raw, _coord: PhantomData })
	}
}

impl<G: GridType + Default, Co: GridCoord> GridTree<G, Co> {
	pub fn read_from<R: Read>(reader: &mut R) -> io::Result<Self> {
		Self::read_from_with_type(G::default(), reader)
	}
}

impl<G, Co> Serialize for GridTree<G, Co>
where
	G: GridType + Serialize,
	Co: GridCoord,
{
	fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
	where
		S: Serializer,
	{
		let mut nodes = Vec::new();
		serialize_node_bytes(&self.raw, &self.grid_type, 0, &mut nodes);
		let mut tuple = serializer.serialize_tuple(5)?;
		tuple.serialize_element(&self.grid_type)?;
		tuple.serialize_element(&self.raw.root_pos())?;
		tuple.serialize_element(&self.raw.root_depth())?;
		tuple.serialize_element(&self.raw.item_count())?;
		tuple.serialize_element(&nodes)?;
		tuple.end()
	}
}

impl<'de, G, Co> Deserialize<'de> for GridTree<G, Co>
where
	G: GridType + Deserialize<'de>,
	Co: GridCoord,
{
	fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
	where
		D: Deserializer<'de>,
	{
		struct GridTreeVisitor<G: GridType, Co: GridCoord>(PhantomData<(G, Co)>);

		impl<'de, G, Co> Visitor<'de> for GridTreeVisitor<G, Co>
		where
			G: GridType + Deserialize<'de>,
			Co: GridCoord,
		{
			type Value = GridTree<G, Co>;

			fn expecting(&self, formatter: &mut std::fmt::Formatter) -> std::fmt::Result { formatter.write_str("a grid tree tuple") }

			fn visit_seq<A>(self, mut seq: A) -> Result<Self::Value, A::Error>
			where
				A: SeqAccess<'de>,
			{
				let grid_type: G = seq.next_element()?.ok_or_else(|| de::Error::invalid_length(0, &self))?;
				let root_pos = seq.next_element()?.ok_or_else(|| de::Error::invalid_length(1, &self))?;
				let root_depth = seq.next_element()?.ok_or_else(|| de::Error::invalid_length(2, &self))?;
				let item_count = seq.next_element()?.ok_or_else(|| de::Error::invalid_length(3, &self))?;
				let node_bytes: Vec<u8> = seq.next_element()?.ok_or_else(|| de::Error::invalid_length(4, &self))?;

				let mut input: &[u8] = &node_bytes;
				let mut raw = RawGridTree::new(grid_type.data_size_bytes());
				raw.clear_all_nodes();
				deserialize_node_bytes(&mut input, &mut raw, &grid_type, None).map_err(de::Error::custom)?;
				if !input.is_empty() { return Err(de::Error::custom("grid tree byte stream has trailing data")); }
				raw.set_root(root_pos, root_depth);
				raw.set_item_count(item_count);
				Ok(GridTree { grid_type, raw, _coord: PhantomData })
			}
		}

		deserializer.deserialize_tuple(5, GridTreeVisitor::<G, Co>(PhantomData))
	}
}
