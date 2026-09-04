use std::sync::{Arc, Mutex};

use bevy::math::IVec3;
use rustc_hash::FxHashMap;
use voxel_data::grid::GridId;
use voxel_sources::SourceId;
use voxel_trees::signed_grid_tree::SignedGridTree;

use crate::{MarkerGridType, MassDelta, MassError, MassProperties, SourceMassChange};

#[derive(Debug)]
struct PendingChange {
	delta: MassDelta,
	new_error: MassError,
}

#[derive(Debug, Default)]
struct GridState {
	current_error: MassError,
	pending: Option<PendingChange>,
	reconciled_chunks: SignedGridTree<MarkerGridType>,
}

impl GridState {
	fn queue(&mut self, delta: MassDelta, new_error: MassError) {
		self.pending = Some(match self.pending.take() {
			Some(pending) => PendingChange {
				delta: pending.delta.checked_add(delta),
				new_error,
			},
			None => PendingChange { delta, new_error },
		});
	}
}

#[derive(Debug, Default)]
struct SourceMassStateInner {
	source_id: Option<SourceId>,
	grids: FxHashMap<GridId, GridState>,
}

#[derive(Clone, Debug, Default)]
pub struct SourceMassState {
	inner: Arc<Mutex<SourceMassStateInner>>,
}

impl SourceMassState {
	pub fn set_source_id(&self, source_id: SourceId) {
		let mut inner = self.inner.lock().expect("source mass state lock poisoned");
		match inner.source_id {
			Some(current) => assert_eq!(current, source_id, "source mass state was rebound"),
			None => inner.source_id = Some(source_id),
		}
	}

	pub fn source_id(&self) -> SourceId {
		self.inner.lock().expect("source mass state lock poisoned").source_id.expect("source mass state was not initialized")
	}

	/// Queues the initial estimate and returns `false` when the grid was already initialized.
	pub fn initialize_grid_once(&self, grid: GridId, estimate: MassProperties, error: MassError) -> bool {
		let mut inner = self.inner.lock().expect("source mass state lock poisoned");
		if inner.grids.contains_key(&grid) { return false; }
		inner.grids.insert(grid, GridState {
			current_error: error,
			pending: Some(PendingChange {
				delta: MassDelta::checked_difference(estimate, MassProperties::ZERO),
				new_error: error,
			}),
			reconciled_chunks: SignedGridTree::new(),
		});
		true
	}

	pub fn replace_grid_estimate(
		&self,
		grid: GridId,
		previous: MassProperties,
		estimate: MassProperties,
		error: MassError,
	) {
		let mut inner = self.inner.lock().expect("source mass state lock poisoned");
		let state = inner.grids.get_mut(&grid).expect("mass grid was not initialized");
		state.current_error = error;
		state.reconciled_chunks = SignedGridTree::new();
		state.queue(MassDelta::checked_difference(estimate, previous), error);
	}

	pub fn current_error(&self, grid: GridId) -> Option<MassError> {
		self.inner.lock().expect("source mass state lock poisoned").grids.get(&grid).map(|state| state.current_error)
	}

	pub fn is_chunk_reconciled(&self, grid: GridId, lod0_chunk: IVec3) -> bool {
		self.inner.lock().expect("source mass state lock poisoned")
			.grids.get(&grid)
			.is_some_and(|state| state.reconciled_chunks.get(lod0_chunk).is_some())
	}

	/// Replaces one chunk estimate with exact data. Repeated reconciliation is ignored.
	pub fn reconcile_generated_chunk(
		&self,
		grid: GridId,
		lod0_chunk: IVec3,
		estimate: MassProperties,
		error: MassError,
		exact: MassProperties,
	) -> bool {
		let mut inner = self.inner.lock().expect("source mass state lock poisoned");
		let state = inner.grids.get_mut(&grid).expect("mass grid was not initialized");
		if state.reconciled_chunks.get(lod0_chunk).is_some() { return false; }

		let mut new_error = state.current_error;
		new_error.checked_reduce(error);
		let delta = MassDelta::checked_difference(exact, estimate);
		let new_pending_delta = state.pending.as_ref().map_or(delta, |pending| pending.delta.checked_add(delta));

		state.current_error = new_error;
		state.pending = Some(PendingChange { delta: new_pending_delta, new_error });
		state.reconciled_chunks.insert(lod0_chunk, ());
		true
	}

	/// Adds temporary uncertainty while an edit's exact result is unavailable.
	pub fn reserve_edit_error(&self, grid: GridId, error: MassError) {
		let mut inner = self.inner.lock().expect("source mass state lock poisoned");
		let state = inner.grids.get_mut(&grid).expect("mass grid was not initialized");
		let new_error = state.current_error.checked_expanded(error);
		state.current_error = new_error;
		state.queue(MassDelta::ZERO, new_error);
	}

	/// Applies an exact edit delta and removes its earlier error reservation.
	pub fn apply_exact_edit_delta(&self, grid: GridId, delta: MassDelta, reserved_error: MassError) {
		let mut inner = self.inner.lock().expect("source mass state lock poisoned");
		let state = inner.grids.get_mut(&grid).expect("mass grid was not initialized");
		let new_error = state.current_error.checked_reduced(reserved_error);
		let new_pending_delta = state.pending.as_ref().map_or(delta, |pending| pending.delta.checked_add(delta));
		state.current_error = new_error;
		state.pending = Some(PendingChange { delta: new_pending_delta, new_error });
	}

	pub fn apply_exact_edit(
		&self,
		grid: GridId,
		before: MassProperties,
		after: MassProperties,
		reserved_error: MassError,
	) {
		self.apply_exact_edit_delta(grid, MassDelta::checked_difference(after, before), reserved_error);
	}

	/// Changes source uncertainty without changing reconciliation markers.
	pub fn expand_grid_error(&self, grid: GridId, error: MassError) {
		self.reserve_edit_error(grid, error);
	}

	/// Changes source uncertainty without changing reconciliation markers.
	pub fn reduce_grid_error(&self, grid: GridId, error: MassError) {
		self.apply_exact_edit_delta(grid, MassDelta::ZERO, error);
	}

	pub fn drain_changes(&self) -> Vec<SourceMassChange> {
		let mut inner = self.inner.lock().expect("source mass state lock poisoned");
		let source_id = inner.source_id.expect("source mass state was not initialized");
		inner.grids.iter_mut().filter_map(|(grid, state)| {
			let pending = state.pending.take()?;
			Some(SourceMassChange::new(source_id, *grid, pending.delta, pending.new_error))
		}).collect()
	}

	pub fn forget_grid(&self, grid: GridId) -> bool {
		self.inner.lock().expect("source mass state lock poisoned").grids.remove(&grid).is_some()
	}
}
