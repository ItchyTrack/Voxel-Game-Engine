use std::{io::{self, Read, Write}, marker::PhantomData};

use serde::{de::{self, SeqAccess, Visitor}, ser::SerializeTuple, Deserialize, Deserializer, Serialize, Serializer};

use super::{CellKind, GridCell, GridCoord, GridTree, GridTreeNode, SIZE_USIZE_CUBED};

fn push_u64(out: &mut Vec<u8>, value: u64) {
	out.extend_from_slice(&value.to_le_bytes());
}

fn write_u8<W: Write>(writer: &mut W, value: u8) -> io::Result<()> {
	writer.write_all(&[value])
}

fn write_u64<W: Write>(writer: &mut W, value: u64) -> io::Result<()> {
	writer.write_all(&value.to_le_bytes())
}

fn write_i32<W: Write>(writer: &mut W, value: i32) -> io::Result<()> {
	writer.write_all(&value.to_le_bytes())
}

fn read_u8<R: Read>(reader: &mut R) -> io::Result<u8> {
	let mut buf = [0u8; 1];
	reader.read_exact(&mut buf)?;
	Ok(buf[0])
}

fn read_u64_from<R: Read>(reader: &mut R) -> io::Result<u64> {
	let mut buf = [0u8; 8];
	reader.read_exact(&mut buf)?;
	Ok(u64::from_le_bytes(buf))
}

fn read_i32<R: Read>(reader: &mut R) -> io::Result<i32> {
	let mut buf = [0u8; 4];
	reader.read_exact(&mut buf)?;
	Ok(i32::from_le_bytes(buf))
}

fn read_u64(input: &mut &[u8]) -> Result<u64, String> {
	if input.len() < 8 {
		return Err("grid tree byte stream ended early while reading u64".into());
	}
	let (head, tail) = input.split_at(8);
	*input = tail;
	Ok(u64::from_le_bytes(head.try_into().expect("split_at guarantees 8 bytes")))
}

fn read_data<C: GridCell>(input: &mut &[u8]) -> Result<C::Data, String> {
	if input.len() < C::DATA_SIZE {
		return Err("grid tree byte stream ended early while reading cell data".into());
	}
	let (head, tail) = input.split_at(C::DATA_SIZE);
	*input = tail;
	Ok(C::read_data_bytes(head))
}

fn serialize_node_bytes<C: GridCell>(nodes: &[GridTreeNode<C>], node_index: u32, out: &mut Vec<u8>) {
	let node = &nodes[node_index as usize];
	let mut data_mask = 0u64;
	let mut node_mask = 0u64;
	for (index, cell) in node.contents.iter().copied().enumerate() {
		match cell.kind() {
			CellKind::Empty => {}
			CellKind::Data => data_mask |= 1u64 << index,
			CellKind::Node => node_mask |= 1u64 << index,
		}
	}
	push_u64(out, data_mask);
	push_u64(out, node_mask);
	for cell in node.contents.iter().copied() {
		if matches!(cell.kind(), CellKind::Data) {
			C::write_data_bytes(cell.data_value(), out);
		}
	}
	for cell in node.contents.iter().copied() {
		if matches!(cell.kind(), CellKind::Node) {
			serialize_node_bytes(nodes, node_index + cell.node_offset(), out);
		}
	}
}

fn deserialize_node_bytes<C: GridCell>(
	input: &mut &[u8],
	nodes: &mut Vec<GridTreeNode<C>>,
	parent_index: Option<u32>,
) -> Result<u32, String> {
	let data_mask = read_u64(input)?;
	let node_mask = read_u64(input)?;
	if data_mask & node_mask != 0 {
		return Err("grid tree node has overlapping data and node masks".into());
	}

	let node_index = nodes.len() as u32;
	nodes.push(GridTreeNode {
		contents: [C::EMPTY; SIZE_USIZE_CUBED],
		parent_offset: parent_index.map_or(0, |parent| (node_index - parent) as u16),
		used_cell_count: (data_mask | node_mask).count_ones() as u8,
	});

	for index in 0..SIZE_USIZE_CUBED {
		if (data_mask & (1u64 << index)) != 0 {
			let value = read_data::<C>(input)?;
			nodes[node_index as usize].contents[index] = C::data(value);
		}
	}
	for index in 0..SIZE_USIZE_CUBED {
		if (node_mask & (1u64 << index)) != 0 {
			let child_index = deserialize_node_bytes::<C>(input, nodes, Some(node_index))?;
			nodes[node_index as usize].contents[index] = C::node(child_index - node_index);
		}
	}
	Ok(node_index)
}

fn write_node_to<W: Write, C: GridCell>(writer: &mut W, nodes: &[GridTreeNode<C>], node_index: u32) -> io::Result<()> {
	let node = &nodes[node_index as usize];
	let mut data_mask = 0u64;
	let mut node_mask = 0u64;
	for (index, cell) in node.contents.iter().copied().enumerate() {
		match cell.kind() {
			CellKind::Empty => {}
			CellKind::Data => data_mask |= 1u64 << index,
			CellKind::Node => node_mask |= 1u64 << index,
		}
	}
	write_u64(writer, data_mask)?;
	write_u64(writer, node_mask)?;
	let mut data_buf = Vec::with_capacity(C::DATA_SIZE);
	for cell in node.contents.iter().copied() {
		if matches!(cell.kind(), CellKind::Data) {
			data_buf.clear();
			C::write_data_bytes(cell.data_value(), &mut data_buf);
			writer.write_all(&data_buf)?;
		}
	}
	for cell in node.contents.iter().copied() {
		if matches!(cell.kind(), CellKind::Node) {
			write_node_to(writer, nodes, node_index + cell.node_offset())?;
		}
	}
	Ok(())
}

fn read_node_from<R: Read, C: GridCell>(reader: &mut R, nodes: &mut Vec<GridTreeNode<C>>, parent_index: Option<u32>) -> io::Result<u32> {
	let data_mask = read_u64_from(reader)?;
	let node_mask = read_u64_from(reader)?;
	if data_mask & node_mask != 0 {
		return Err(io::Error::new(io::ErrorKind::InvalidData, "grid tree node has overlapping data and node masks"));
	}
	let node_index = nodes.len() as u32;
	nodes.push(GridTreeNode {
		contents: [C::EMPTY; SIZE_USIZE_CUBED],
		parent_offset: parent_index.map_or(0, |parent| (node_index - parent) as u16),
		used_cell_count: (data_mask | node_mask).count_ones() as u8,
	});
	let mut data_buf = vec![0u8; C::DATA_SIZE];
	for index in 0..SIZE_USIZE_CUBED {
		if (data_mask & (1u64 << index)) != 0 {
			reader.read_exact(&mut data_buf)?;
			nodes[node_index as usize].contents[index] = C::data(C::read_data_bytes(&data_buf));
		}
	}
	for index in 0..SIZE_USIZE_CUBED {
		if (node_mask & (1u64 << index)) != 0 {
			let child_index = read_node_from::<R, C>(reader, nodes, Some(node_index))?;
			nodes[node_index as usize].contents[index] = C::node(child_index - node_index);
		}
	}
	Ok(node_index)
}

impl<C: GridCell, Co: GridCoord> GridTree<C, Co> {
	pub fn write_to<W: Write>(&self, writer: &mut W) -> io::Result<()> {
		write_i32(writer, self.root_pos.x)?;
		write_i32(writer, self.root_pos.y)?;
		write_i32(writer, self.root_pos.z)?;
		write_u8(writer, self.root_depth)?;
		write_u64(writer, self.item_count)?;
		write_node_to(writer, &self.nodes, 0)
	}

	pub fn read_from<R: Read>(reader: &mut R) -> io::Result<Self> {
		let root_pos = bevy::math::IVec3::new(read_i32(reader)?, read_i32(reader)?, read_i32(reader)?);
		let root_depth = read_u8(reader)?;
		let item_count = read_u64_from(reader)?;
		let mut nodes = Vec::new();
		read_node_from::<R, C>(reader, &mut nodes, None)?;
		Ok(GridTree {
			nodes,
			root_pos,
			root_depth,
			item_count,
			dead_nodes: 0,
			_coord: PhantomData,
		})
	}
}

impl<C: GridCell, Co: GridCoord> Serialize for GridTree<C, Co> {
	fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
	where
		S: Serializer,
	{
		let mut nodes = Vec::new();
		serialize_node_bytes(&self.nodes, 0, &mut nodes);
		let mut tuple = serializer.serialize_tuple(4)?;
		tuple.serialize_element(&self.root_pos)?;
		tuple.serialize_element(&self.root_depth)?;
		tuple.serialize_element(&self.item_count)?;
		tuple.serialize_element(&nodes)?;
		tuple.end()
	}
}

impl<'de, C: GridCell, Co: GridCoord> Deserialize<'de> for GridTree<C, Co> {
	fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
	where
		D: Deserializer<'de>,
	{
		struct GridTreeVisitor<C: GridCell, Co: GridCoord>(PhantomData<(C, Co)>);

		impl<'de, C: GridCell, Co: GridCoord> Visitor<'de> for GridTreeVisitor<C, Co> {
			type Value = GridTree<C, Co>;

			fn expecting(&self, formatter: &mut std::fmt::Formatter) -> std::fmt::Result {
				formatter.write_str("a grid tree tuple")
			}

			fn visit_seq<A>(self, mut seq: A) -> Result<Self::Value, A::Error>
			where
				A: SeqAccess<'de>,
			{
				let root_pos = seq.next_element()?.ok_or_else(|| de::Error::invalid_length(0, &self))?;
				let root_depth = seq.next_element()?.ok_or_else(|| de::Error::invalid_length(1, &self))?;
				let item_count = seq.next_element()?.ok_or_else(|| de::Error::invalid_length(2, &self))?;
				let node_bytes: Vec<u8> = seq.next_element()?.ok_or_else(|| de::Error::invalid_length(3, &self))?;

				let mut input: &[u8] = &node_bytes;
				let mut nodes = Vec::new();
				deserialize_node_bytes::<C>(&mut input, &mut nodes, None).map_err(de::Error::custom)?;
				if !input.is_empty() {
					return Err(de::Error::custom("grid tree byte stream has trailing data"));
				}

				Ok(GridTree {
					nodes,
					root_pos,
					root_depth,
					item_count,
					dead_nodes: 0,
					_coord: PhantomData,
				})
			}
		}

		deserializer.deserialize_tuple(4, GridTreeVisitor::<C, Co>(PhantomData))
	}
}
