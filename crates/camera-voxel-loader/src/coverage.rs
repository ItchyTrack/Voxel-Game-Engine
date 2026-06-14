use bevy::prelude::*;
use voxel_streaming::CHUNK_SIZE;

use crate::camera_voxel_loader::CameraVoxelLoader;
use crate::types::{ChunkKey, TileKey};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub(crate) enum CoverageSource {
	Chunk(ChunkKey),
	Tile(TileKey),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum SourceResolution {
	Requested,
	Visible(Entity),
	Empty,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum SourceState {
	Desired(SourceResolution),
	RetiringVisible(Entity),
}

#[derive(Clone, Debug)]
pub(crate) struct CoverageRecord {
	pub(crate) cells: Box<[ChunkKey]>,
	pub(crate) state: SourceState,
}

#[derive(Clone, Debug, Default)]
pub(crate) struct CoverageCell {
	pub(crate) desired: Vec<CoverageSource>,
	pub(crate) visible: Vec<CoverageSource>,
	pub(crate) empty: Vec<CoverageSource>,
}

pub(crate) fn source_chunks(source: CoverageSource) -> Vec<ChunkKey> {
	match source {
		CoverageSource::Chunk(chunk) => vec![chunk],
		CoverageSource::Tile(tile) => {
			let min = tile.min;
			let max = tile.min + tile.size();
			let mut chunks = Vec::with_capacity((tile.size().x * tile.size().y * tile.size().z) as usize);
			for x in min.x..max.x {
				for y in min.y..max.y {
					for z in min.z..max.z {
						chunks.push(ChunkKey { grid: tile.grid, chunk: IVec3::new(x, y, z) });
					}
				}
			}
			chunks
		}
	}
}

#[cfg(test)]
pub(crate) fn chunks_for_subgrid(grid: Entity, subgrid_pos: IVec3) -> Vec<ChunkKey> {
	chunks_for_subgrid_bounds(grid, subgrid_pos, IVec3::ZERO, IVec3::splat(voxel_data::grid::SUB_GRID_SIZE - 1))
}

pub(crate) fn chunks_for_subgrid_bounds(grid: Entity, subgrid_pos: IVec3, bounds_min: IVec3, bounds_max: IVec3) -> Vec<ChunkKey> {
	let min = (subgrid_pos + bounds_min).div_euclid(IVec3::splat(CHUNK_SIZE));
	let max = (subgrid_pos + bounds_max).div_euclid(IVec3::splat(CHUNK_SIZE));
	let mut chunks = Vec::new();
	for x in min.x..=max.x {
		for y in min.y..=max.y {
			for z in min.z..=max.z {
				chunks.push(ChunkKey { grid, chunk: IVec3::new(x, y, z) });
			}
		}
	}
	chunks
}

pub(crate) fn request_source(loader: &mut CameraVoxelLoader, source: CoverageSource) {
	if !loader.coverage_sources.contains_key(&source) {
		let cells = source_chunks(source).into_boxed_slice();
		loader.coverage_sources.insert(source, CoverageRecord { cells, state: SourceState::Desired(SourceResolution::Requested) });
		add_to_cells(loader, source, CellSet::Desired);
		debug_assert_invariants(loader);
		return;
	}

	let state = loader.coverage_sources.get(&source).map(|record| record.state);
	match state {
		Some(SourceState::Desired(_)) => {}
		Some(SourceState::RetiringVisible(entity)) => {
			if let Some(record) = loader.coverage_sources.get_mut(&source) {
				record.state = SourceState::Desired(SourceResolution::Visible(entity));
			}
			add_to_cells(loader, source, CellSet::Desired);
			add_to_cells(loader, source, CellSet::Visible);
		}
		None => unreachable!(),
	}
	debug_assert_invariants(loader);
}

pub(crate) fn undesire_source(loader: &mut CameraVoxelLoader, source: CoverageSource) {
	let Some(state) = loader.coverage_sources.get(&source).map(|record| record.state) else { return };
	match state {
		SourceState::Desired(SourceResolution::Requested | SourceResolution::Empty) => {
			remove_source(loader, source);
			return;
		}
		SourceState::Desired(SourceResolution::Visible(entity)) => {
			remove_from_cells(loader, source, CellSet::Desired);
			if let Some(record) = loader.coverage_sources.get_mut(&source) {
				record.state = SourceState::RetiringVisible(entity);
			}
		}
		SourceState::RetiringVisible(_) => {}
	}
	debug_assert_invariants(loader);
}

pub(crate) fn resolve_empty(loader: &mut CameraVoxelLoader, source: CoverageSource) {
	let Some(state) = loader.coverage_sources.get(&source).map(|record| record.state) else { return };
	match state {
		SourceState::Desired(_) => {
			remove_from_cells(loader, source, CellSet::Visible);
			add_to_cells(loader, source, CellSet::Empty);
			if let Some(record) = loader.coverage_sources.get_mut(&source) {
				record.state = SourceState::Desired(SourceResolution::Empty);
			}
		}
		SourceState::RetiringVisible(_) => {
			remove_source(loader, source);
			return;
		}
	}
	debug_assert_invariants(loader);
}

pub(crate) fn resolve_visible(loader: &mut CameraVoxelLoader, source: CoverageSource, entity: Entity) {
	let Some(state) = loader.coverage_sources.get(&source).map(|record| record.state) else { return };
	match state {
		SourceState::Desired(_) => {
			remove_from_cells(loader, source, CellSet::Empty);
			add_to_cells(loader, source, CellSet::Visible);
			if let Some(record) = loader.coverage_sources.get_mut(&source) {
				record.state = SourceState::Desired(SourceResolution::Visible(entity));
			}
		}
		SourceState::RetiringVisible(_) => {
			add_to_cells(loader, source, CellSet::Visible);
			if let Some(record) = loader.coverage_sources.get_mut(&source) {
				record.state = SourceState::RetiringVisible(entity);
			}
		}
	}
	debug_assert_invariants(loader);
}

pub(crate) fn ready_retiring_sources(loader: &CameraVoxelLoader) -> Vec<CoverageSource> {
	loader
		.coverage_sources
		.iter()
		.filter_map(|(&source, record)| matches!(record.state, SourceState::RetiringVisible(_)).then_some(source))
		.filter(|&source| can_retire(loader, source))
		.collect()
}

pub(crate) fn retiring_visible_chunks(loader: &CameraVoxelLoader) -> Vec<ChunkKey> {
	loader
		.coverage_sources
		.iter()
		.filter_map(|(&source, record)| match (source, record.state) {
			(CoverageSource::Chunk(chunk), SourceState::RetiringVisible(_)) => Some(chunk),
			_ => None,
		})
		.collect()
}

pub(crate) fn remove_source(loader: &mut CameraVoxelLoader, source: CoverageSource) {
	let Some(record) = loader.coverage_sources.remove(&source) else { return };
	for chunk in record.cells.iter().copied() {
		if let Some(cell) = loader.coverage_cells.get_mut(&chunk) {
			remove_source_from_vec(&mut cell.desired, source);
			remove_source_from_vec(&mut cell.visible, source);
			remove_source_from_vec(&mut cell.empty, source);
		}
		if loader
			.coverage_cells
			.get(&chunk)
			.is_some_and(|cell| cell.desired.is_empty() && cell.visible.is_empty() && cell.empty.is_empty())
		{
			loader.coverage_cells.remove(&chunk);
		}
	}
	debug_assert_invariants(loader);
}

#[derive(Clone, Copy)]
enum CellSet {
	Desired,
	Visible,
	Empty,
}

fn add_to_cells(loader: &mut CameraVoxelLoader, source: CoverageSource, set: CellSet) {
	let Some(cells) = loader.coverage_sources.get(&source).map(|record| record.cells.clone()) else { return };
	for chunk in cells.iter().copied() {
		let cell = loader.coverage_cells.entry(chunk).or_default();
		match set {
			CellSet::Desired => add_source_once(&mut cell.desired, source),
			CellSet::Visible => add_source_once(&mut cell.visible, source),
			CellSet::Empty => add_source_once(&mut cell.empty, source),
		}
	}
}

fn remove_from_cells(loader: &mut CameraVoxelLoader, source: CoverageSource, set: CellSet) {
	let Some(cells) = loader.coverage_sources.get(&source).map(|record| record.cells.clone()) else { return };
	for chunk in cells.iter().copied() {
		let Some(cell) = loader.coverage_cells.get_mut(&chunk) else { continue };
		match set {
			CellSet::Desired => remove_source_from_vec(&mut cell.desired, source),
			CellSet::Visible => remove_source_from_vec(&mut cell.visible, source),
			CellSet::Empty => remove_source_from_vec(&mut cell.empty, source),
		}
	}
}

fn can_retire(loader: &CameraVoxelLoader, source: CoverageSource) -> bool {
	let Some(record) = loader.coverage_sources.get(&source) else { return true };
	record.cells.iter().all(|chunk| {
		let Some(cell) = loader.coverage_cells.get(chunk) else { return true };
		!matches!(coverage_cell_replacement_state(loader, cell, source), CoverageCellReplacementState::Waiting)
	})
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum CoverageCellReplacementState {
	NoDesiredCoverage,
	Waiting,
	Replaced,
}

pub(crate) fn coverage_cell_replacement_state(loader: &CameraVoxelLoader, cell: &CoverageCell, retiring_source: CoverageSource) -> CoverageCellReplacementState {
	if cell.desired.is_empty() {
		return CoverageCellReplacementState::NoDesiredCoverage;
	}
	if coverage_cell_has_desired_replacement(loader, cell, retiring_source) {
		CoverageCellReplacementState::Replaced
	} else {
		CoverageCellReplacementState::Waiting
	}
}

fn coverage_cell_has_desired_replacement(loader: &CameraVoxelLoader, cell: &CoverageCell, retiring_source: CoverageSource) -> bool {
	cell.empty.iter().any(|candidate| *candidate != retiring_source && source_is_desired_empty(loader, *candidate))
		|| cell.visible.iter().any(|candidate| *candidate != retiring_source && source_is_desired_visible(loader, *candidate))
}

fn source_is_desired_visible(loader: &CameraVoxelLoader, source: CoverageSource) -> bool {
	matches!(
		loader.coverage_sources.get(&source).map(|record| record.state),
		Some(SourceState::Desired(SourceResolution::Visible(_)))
	)
}

fn source_is_desired_empty(loader: &CameraVoxelLoader, source: CoverageSource) -> bool {
	matches!(
		loader.coverage_sources.get(&source).map(|record| record.state),
		Some(SourceState::Desired(SourceResolution::Empty))
	)
}

fn add_source_once(sources: &mut Vec<CoverageSource>, source: CoverageSource) {
	if !sources.contains(&source) {
		sources.push(source);
	}
}

fn remove_source_from_vec(sources: &mut Vec<CoverageSource>, source: CoverageSource) {
	sources.retain(|candidate| *candidate != source);
}

#[cfg(debug_assertions)]
fn debug_assert_invariants(loader: &CameraVoxelLoader) {
	if let Err(message) = validate_invariants(loader) {
		panic!("coverage invariant violated: {message}");
	}
}

#[cfg(not(debug_assertions))]
fn debug_assert_invariants(_loader: &CameraVoxelLoader) {}

#[cfg(debug_assertions)]
fn validate_invariants(loader: &CameraVoxelLoader) -> Result<(), String> {
	for (&source, record) in &loader.coverage_sources {
		for &chunk in record.cells.iter() {
			let Some(cell) = loader.coverage_cells.get(&chunk) else {
				return Err(format!("source {source:?} references missing cell {chunk:?}"));
			};
			match record.state {
				SourceState::Desired(SourceResolution::Requested) => {
					require_contains(&cell.desired, source, "desired", chunk)?;
					require_absent(&cell.visible, source, "visible", chunk)?;
					require_absent(&cell.empty, source, "empty", chunk)?;
				}
				SourceState::Desired(SourceResolution::Visible(_)) => {
					require_contains(&cell.desired, source, "desired", chunk)?;
					require_contains(&cell.visible, source, "visible", chunk)?;
					require_absent(&cell.empty, source, "empty", chunk)?;
				}
				SourceState::Desired(SourceResolution::Empty) => {
					require_contains(&cell.desired, source, "desired", chunk)?;
					require_absent(&cell.visible, source, "visible", chunk)?;
					require_contains(&cell.empty, source, "empty", chunk)?;
				}
				SourceState::RetiringVisible(_) => {
					require_absent(&cell.desired, source, "desired", chunk)?;
					require_contains(&cell.visible, source, "visible", chunk)?;
					require_absent(&cell.empty, source, "empty", chunk)?;
				}
			}
		}
	}

	for (&chunk, cell) in &loader.coverage_cells {
		check_unique(&cell.desired, "desired", chunk)?;
		check_unique(&cell.visible, "visible", chunk)?;
		check_unique(&cell.empty, "empty", chunk)?;
		for &source in cell.desired.iter().chain(cell.visible.iter()).chain(cell.empty.iter()) {
			let Some(record) = loader.coverage_sources.get(&source) else {
				return Err(format!("cell {chunk:?} references missing source {source:?}"));
			};
			if !record.cells.contains(&chunk) {
				return Err(format!("cell {chunk:?} references source {source:?} that does not cover it"));
			}
		}
	}
	Ok(())
}

#[cfg(debug_assertions)]
fn require_contains(sources: &[CoverageSource], source: CoverageSource, set: &str, chunk: ChunkKey) -> Result<(), String> {
	if sources.contains(&source) { Ok(()) } else { Err(format!("{set} cell {chunk:?} missing source {source:?}")) }
}

#[cfg(debug_assertions)]
fn require_absent(sources: &[CoverageSource], source: CoverageSource, set: &str, chunk: ChunkKey) -> Result<(), String> {
	if sources.contains(&source) { Err(format!("{set} cell {chunk:?} unexpectedly contains source {source:?}")) } else { Ok(()) }
}

#[cfg(debug_assertions)]
fn check_unique(sources: &[CoverageSource], set: &str, chunk: ChunkKey) -> Result<(), String> {
	for (index, source) in sources.iter().enumerate() {
		if sources[index + 1..].contains(source) {
			return Err(format!("{set} cell {chunk:?} contains duplicate source {source:?}"));
		}
	}
	Ok(())
}
