use bevy::prelude::*;
use rustc_hash::FxHashMap;
use voxel_sources::{ChunkSource, SourceManager};

use crate::{GridMassProperties, SourceMassChange, VoxelMassReaders, VoxelMassSet};

/// A source owns its mass bookkeeping and any work needed to produce these updates.
pub trait SourceMass: ChunkSource {
	fn take_mass_changes(&mut self, readers: &VoxelMassReaders) -> Vec<SourceMassChange>;
}

pub trait SourceMassAppExt {
	/// Registers direct mass updates for one concrete source type.
	fn register_source_mass<S: SourceMass + 'static>(&mut self) -> &mut Self;
}

impl SourceMassAppExt for App {
	fn register_source_mass<S: SourceMass + 'static>(&mut self) -> &mut Self {
		self.add_systems(FixedUpdate, apply_source_mass_changes::<S>.in_set(VoxelMassSet::ApplySourceChanges))
	}
}

pub fn apply_source_mass_changes<S: SourceMass + 'static>(
	mut sources: ResMut<SourceManager>,
	readers: Res<VoxelMassReaders>,
	mut grids: Query<&mut GridMassProperties>,
) {
	let Some(source) = sources.get_source_mut::<S>() else { return };
	let mut by_grid = FxHashMap::<Entity, Vec<SourceMassChange>>::default();
	for change in source.take_mass_changes(&readers) {
		by_grid.entry(change.grid()).or_default().push(change);
	}
	for (grid, changes) in by_grid {
		if let Ok(mut properties) = grids.get_mut(grid) {
			properties.apply_batch(&changes);
		}
	}
}
