use std::fmt::Debug;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum CellKind {
	Empty,
	Data,
	Node,
}

/// How a single cell of a node is encoded (empty / data leaf / child-node offset).
pub trait GridCell: Copy + Eq + Debug {
	type Data: Copy + Eq + Ord + Debug;
	const EMPTY: Self;
	const MAX_DATA: Self::Data;
	const MAX_NODE_OFFSET: u32;
	const DATA_SIZE: usize;
	fn data(value: Self::Data) -> Self;
	fn node(offset: u32) -> Self;
	fn kind(self) -> CellKind;
	fn data_value(self) -> Self::Data;
	fn node_offset(self) -> u32;
	fn write_data_bytes(value: Self::Data, out: &mut Vec<u8>);
	fn read_data_bytes(bytes: &[u8]) -> Self::Data;
}
