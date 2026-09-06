use bevy::prelude::*;
use rustc_hash::FxHashMap;
use voxel_sources::{ChunkSource, SourceManager};

use crate::{GridMassProperties, SourceMassUpdate, VoxelMassReaders, VoxelMassSet};

pub trait SourceMass: ChunkSource {
	fn take_mass_updates(&mut self, readers: &VoxelMassReaders) -> Vec<SourceMassUpdate>;
}

pub trait SourceMassAppExt {
	fn register_source_mass<S: SourceMass + 'static>(&mut self) -> &mut Self;
}

impl SourceMassAppExt for App {
	fn register_source_mass<S: SourceMass + 'static>(&mut self) -> &mut Self {
		self.add_systems(FixedUpdate, apply_source_mass_updates::<S>.in_set(VoxelMassSet::ApplySourceChanges))
	}
}

pub fn apply_source_mass_updates<S: SourceMass + 'static>(
	mut sources: ResMut<SourceManager>,
	readers: Res<VoxelMassReaders>,
	mut grids: Query<&mut GridMassProperties>,
) {
	let Some(source) = sources.get_source_mut::<S>() else { return };
	let mut by_grid = FxHashMap::<Entity, Vec<SourceMassUpdate>>::default();
	for mass_updates in source.take_mass_updates(&readers) {
		by_grid.entry(mass_updates.grid()).or_default().push(mass_updates);
	}
	for (grid, mass_updates) in by_grid {
		if let Ok(mut properties) = grids.get_mut(grid) {
			properties.apply_batch(&mass_updates);
		}
	}
}
