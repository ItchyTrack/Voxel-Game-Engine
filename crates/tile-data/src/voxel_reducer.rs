use std::{collections::HashMap, sync::{Arc, RwLock}};

use bevy::prelude::Resource;
use voxel_data::voxels::{VoxelTypeId, Voxels};

use crate::{CHUNK_SIZE, NonZeroChunkRegion, VoxelRegionResult};

pub trait TileVoxelReducer: Send + Sync + 'static {
	fn input_type_id(&self) -> VoxelTypeId;
	fn output_type_id(&self) -> VoxelTypeId;
	fn reduce(&self, area: NonZeroChunkRegion, output_lod: u8, inputs: &[&VoxelRegionResult]) -> Option<Voxels>;
}

#[derive(Resource, Clone, Default)]
pub struct TileVoxelReducerRegistry {
	reducers: Arc<RwLock<HashMap<(VoxelTypeId, VoxelTypeId), Arc<dyn TileVoxelReducer>>>>,
}

impl TileVoxelReducerRegistry {
	pub fn insert<R: TileVoxelReducer>(&self, reducer: R) {
		let key = (reducer.input_type_id(), reducer.output_type_id());
		let previous = self.reducers.write().unwrap().insert(key, Arc::new(reducer));
		assert!(previous.is_none(), "tile voxel reducer already registered for {key:?}");
	}

	pub fn reduce(
		&self,
		area: NonZeroChunkRegion,
		output_lod: u8,
		output_type: VoxelTypeId,
		inputs: &[VoxelRegionResult],
	) -> Option<Voxels> {
		let mut exact = Vec::new();
		let mut groups: Vec<(VoxelTypeId, Vec<&VoxelRegionResult>)> = Vec::new();
		for input in inputs {
			let input_type = input.voxels.voxel_type_id();
			if input.lod == output_lod && input_type == output_type {
				exact.push(input);
			} else if input.lod <= output_lod {
				if let Some((_, group)) = groups.iter_mut().find(|(group_type, _)| *group_type == input_type) {
					group.push(input);
				} else {
					groups.push((input_type, vec![input]));
				}
			}
		}

		let reducers = self.reducers.read().unwrap();
		let mut output: Option<Voxels> = None;
		for (input_type, group) in groups {
			let Some(reducer) = reducers.get(&(input_type, output_type)) else { continue };
			let Some(voxels) = reducer.reduce(area, output_lod, &group) else { continue };
			output.get_or_insert_with(|| Voxels::new_with_type(voxels.voxel_type_info())).merge_from(&voxels, bevy::math::IVec3::ZERO);
		}
		drop(reducers);

		let step = 1i32.checked_shl(output_lod as u32)?;
		for input in exact {
			let offset = ((input.area.min() - area.min()) * CHUNK_SIZE).div_euclid(bevy::math::IVec3::splat(step));
			output.get_or_insert_with(|| Voxels::new_with_type(input.voxels.voxel_type_info())).merge_from(&input.voxels, offset);
		}
		output.filter(|voxels| !voxels.is_empty())
	}
}
