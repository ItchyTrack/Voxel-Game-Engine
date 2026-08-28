use std::fmt::Debug;

pub trait GridData: Copy + Eq + Debug {}

impl<T: Copy + Eq + Debug> GridData for T {}

pub trait GridType: Clone + Debug + 'static {
	type Data<'a>: GridData
	where
		Self: 'a;

	const MAX_NODE_OFFSET: u32 = u32::MAX;

	fn data_size_bytes(&self) -> usize;
	fn read_data<'a>(&self, bytes: &'a [u8]) -> Self::Data<'a>;
	fn write_data(&self, data: Self::Data<'_>, bytes: &mut [u8]);
	fn data_eq_bytes(&self, data: Self::Data<'_>, bytes: &[u8]) -> bool;
}
