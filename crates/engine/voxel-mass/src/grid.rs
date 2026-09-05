use bevy::prelude::{Commands, Component, Entity, Message, MessageReader, Query};
use rustc_hash::FxHashMap;
use voxel_data::grid::GridId;
use voxel_sources::SourceId;

use crate::{MassError, MassProperties};

#[derive(Clone, Copy, Debug, Message)]
pub struct SourceMassChange {
	source_id: SourceId,
	grid: GridId,
	before: MassProperties,
	after: MassProperties,
	new_error: MassError,
}

impl SourceMassChange {
	pub const fn new(source_id: SourceId, grid: GridId, before: MassProperties, after: MassProperties, new_error: MassError) -> Self {
		Self { source_id, grid, before, after, new_error }
	}

	pub const fn source_id(&self) -> SourceId { self.source_id }
	pub const fn grid(&self) -> GridId { self.grid }
	pub const fn before(&self) -> MassProperties { self.before }
	pub const fn after(&self) -> MassProperties { self.after }
	pub const fn new_error(&self) -> MassError { self.new_error }
}

#[derive(Component, Clone, Debug, Default)]
pub struct GridMassProperties {
	nominal: MassProperties,
	source_errors: FxHashMap<SourceId, MassError>,
	initialized: bool,
}

impl GridMassProperties {
	pub fn new(nominal: MassProperties) -> Self {
		Self { nominal, source_errors: FxHashMap::default(), initialized: true }
	}

	pub const fn nominal(&self) -> &MassProperties { &self.nominal }
	pub const fn is_initialized(&self) -> bool { self.initialized }
	pub fn source_errors(&self) -> &FxHashMap<SourceId, MassError> { &self.source_errors }
	pub fn source_error(&self, source_id: SourceId) -> Option<MassError> { self.source_errors.get(&source_id).copied() }

	pub fn aggregate_error(&self) -> MassError {
		self.source_errors.values().copied().fold(MassError::ZERO, MassError::checked_add)
	}

	pub fn apply(&mut self, change: &SourceMassChange) {
		self.apply_batch(std::iter::once(change));
	}

	pub fn apply_batch<'a>(&mut self, changes: impl IntoIterator<Item = &'a SourceMassChange>) {
		let changes: Vec<_> = changes.into_iter().collect();
		self.nominal = self.nominal.checked_replaced(changes.iter().map(|change| (change.before, change.after)));
		for change in changes {
			self.source_errors.insert(change.source_id, change.new_error);
		}
		self.initialized = true;
	}
}

pub fn apply_source_mass_changes(
	mut commands: Commands,
	mut changes: MessageReader<SourceMassChange>,
	mut grids: Query<&mut GridMassProperties>,
) {
	let mut by_grid = FxHashMap::<Entity, Vec<SourceMassChange>>::default();
	for change in changes.read() {
		by_grid.entry(change.grid).or_default().push(*change);
	}
	for (grid, changes) in by_grid {
		if let Ok(mut properties) = grids.get_mut(grid) {
			properties.apply_batch(&changes);
		} else {
			let mut properties = GridMassProperties::default();
			properties.apply_batch(&changes);
			commands.entity(grid).insert(properties);
		}
	}
}
