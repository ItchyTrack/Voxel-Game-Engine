use std::collections::{HashMap, HashSet};

use bevy::prelude::*;
use bevy::math::IVec3;

use voxel_data::grid::GridId;

use crate::LodLoadResult;

#[bevy_trait_query::queryable]
pub trait ChunkConsumer {
	fn needed(&self) -> &HashMap<GridId, HashSet<IVec3>>;
	fn needed_mut(&mut self) -> &mut HashMap<GridId, HashSet<IVec3>>;
	fn outstanding(&self) -> usize;
	fn outstanding_mut(&mut self) -> &mut usize;
	fn push_lod(&mut self, result: LodLoadResult);
	fn drain_lod(&mut self) -> Vec<LodLoadResult>;
}

/// Defines a [`ChunkConsumer`] component. Register it with [`VoxelStreamingAppExt::register_chunk_consumer`].
#[macro_export]
macro_rules! chunk_consumer {
	($vis:vis $name:ident) => {
		#[derive($crate::__bevy::prelude::Component, Default, Debug)]
		$vis struct $name {
			needed: ::std::collections::HashMap<
				$crate::__GridId,
				::std::collections::HashSet<$crate::__bevy::math::IVec3>,
			>,
			outstanding: usize,
			lod_inbox: ::std::vec::Vec<$crate::LodLoadResult>,
		}
		impl $crate::ChunkConsumer for $name {
			fn needed(
				&self,
			) -> &::std::collections::HashMap<
				$crate::__GridId,
				::std::collections::HashSet<$crate::__bevy::math::IVec3>,
			> {
				&self.needed
			}
			fn needed_mut(
				&mut self,
			) -> &mut ::std::collections::HashMap<
				$crate::__GridId,
				::std::collections::HashSet<$crate::__bevy::math::IVec3>,
			> {
				&mut self.needed
			}
			fn outstanding(&self) -> usize {
				self.outstanding
			}
			fn outstanding_mut(&mut self) -> &mut usize {
				&mut self.outstanding
			}
			fn push_lod(&mut self, result: $crate::LodLoadResult) {
				self.lod_inbox.push(result);
			}
			fn drain_lod(&mut self) -> ::std::vec::Vec<$crate::LodLoadResult> {
				::std::mem::take(&mut self.lod_inbox)
			}
		}
	};
}

pub trait VoxelStreamingAppExt {
	fn register_chunk_consumer<T: ChunkConsumer + Component>(&mut self) -> &mut Self;
}

impl VoxelStreamingAppExt for App {
	fn register_chunk_consumer<T: ChunkConsumer + Component>(&mut self) -> &mut Self {
		use bevy_trait_query::RegisterExt;
		self.register_component_as::<dyn ChunkConsumer, T>();
		self
	}
}

pub fn chunks_ready<T: ChunkConsumer + Component>(query: Option<Single<&T>>) -> bool {
	query.map_or(true, |consumer| consumer.outstanding() == 0)
}
