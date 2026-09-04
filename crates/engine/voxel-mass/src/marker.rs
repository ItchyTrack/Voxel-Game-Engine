use voxel_trees::grid_tree::GridType;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct MarkerGridType;

impl GridType for MarkerGridType {
	type Data<'a> = ();

	fn data_size_bytes(&self) -> usize { 0 }
	fn read_data<'a>(&self, _bytes: &'a [u8]) -> Self::Data<'a> { () }
	fn write_data(&self, _data: Self::Data<'_>, _bytes: &mut [u8]) {}
	fn data_eq_bytes(&self, _data: Self::Data<'_>, _bytes: &[u8]) -> bool { true }
}
