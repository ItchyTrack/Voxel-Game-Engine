#![allow(dead_code)]

use std::collections::HashMap;

use bevy::prelude::*;
use gpu_voxel_data::SubGridGpuState;
use voxel_data::grid::{Grid, GridId, SUB_GRID_SIZE};
use voxel_data::subgrid::SubGrid;
use voxel_streaming::{GridStreaming, CHUNK_SIZE};

use crate::camera_voxel_loader::CameraVoxelLoader;
use crate::coverage::{coverage_cell_replacement_state, CoverageCellReplacementState, CoverageSource, SourceResolution, SourceState};
use crate::types::{ChunkKey, PolicyDebugBoxKind, TileStatus};

#[derive(Default, Reflect, GizmoConfigGroup)]
pub(crate) struct CameraVoxelLoaderGizmos;

pub(crate) fn draw_lod_policy_bounds_gizmos(
	mut gizmos: Gizmos,
	camera_voxel_loaders: Query<&CameraVoxelLoader, With<Camera3d>>,
	grids: Query<(GridId, &GlobalTransform), With<GridStreaming>>,
) {
	for camera_voxel_loader in &camera_voxel_loaders {
		for (grid, grid_transform) in &grids {
			if let Some((min_chunk, max_chunk)) = lod0_chunk_bounds(camera_voxel_loader, grid) {
				draw_lod_bound_box(&mut gizmos, grid_transform, min_chunk, max_chunk, lod_bound_color(0, camera_voxel_loader.settings.max_lod));
			}

			for key in camera_voxel_loader.desired_tiles.iter().filter(|key| key.grid == grid) {
				draw_lod_bound_box(&mut gizmos, grid_transform, key.min, key.min + key.size(), lod_bound_color(key.lod, camera_voxel_loader.settings.max_lod));
			}
		}
	}
}

fn lod0_chunk_bounds(camera_voxel_loader: &CameraVoxelLoader, grid: GridId) -> Option<(IVec3, IVec3)> {
	let mut min = IVec3::splat(i32::MAX);
	let mut max = IVec3::splat(i32::MIN);
	let mut any = false;
	for chunk in camera_voxel_loader.desired_chunks.iter().filter(|chunk| chunk.grid == grid) {
		any = true;
		min = min.min(chunk.chunk);
		max = max.max(chunk.chunk + IVec3::ONE);
	}
	any.then_some((min, max))
}

fn draw_centered_chunk_box<C: GizmoConfigGroup>(gizmos: &mut Gizmos<C>, grid_transform: &GlobalTransform, center: IVec3, radius: i32, color: Color) {
	let min_chunk = center - IVec3::splat(radius);
	let max_chunk = center + IVec3::splat(radius + 1);
	draw_lod_bound_box(gizmos, grid_transform, min_chunk, max_chunk, color);
}

fn draw_lod_bound_box<C: GizmoConfigGroup>(gizmos: &mut Gizmos<C>, grid_transform: &GlobalTransform, min_chunk: IVec3, max_chunk: IVec3, color: Color) {
	draw_transformed_box(
		gizmos,
		grid_transform,
		(min_chunk * CHUNK_SIZE).as_vec3(),
		(max_chunk * CHUNK_SIZE).as_vec3(),
		color,
	);
}

fn lod_bound_color(lod: u8, max_lod: u8) -> Color {
	let t = if max_lod == 0 { 0.0 } else { lod as f32 / max_lod as f32 };
	Color::srgba(0.05 + 0.25 * t, 0.20 + 0.45 * t, 1.0, 0.45 + 0.35 * (1.0 - t))
}

pub(crate) fn draw_policy_delta_gizmos(
	mut gizmos: Gizmos,
	cameras: Query<&CameraVoxelLoader>,
	grid_transforms: Query<&GlobalTransform, With<GridStreaming>>,
) {
	for camera_voxel_loader in &cameras {
		for debug_box in &camera_voxel_loader.policy_debug_boxes {
			let Ok(grid_transform) = grid_transforms.get(debug_box.grid) else { continue };
			draw_transformed_box(
				&mut gizmos,
				grid_transform,
				(debug_box.min * CHUNK_SIZE).as_vec3(),
				(debug_box.max * CHUNK_SIZE).as_vec3(),
				policy_delta_color(debug_box.kind, debug_box.entering),
			);
		}
	}
}

fn policy_delta_color(kind: PolicyDebugBoxKind, entering: bool) -> Color {
	let alpha = match kind {
		PolicyDebugBoxKind::NearChunks => 0.95,
		PolicyDebugBoxKind::LodOuter(_) => 0.75,
		PolicyDebugBoxKind::LodInner(_) => 0.55,
		PolicyDebugBoxKind::LodNearExclusion(_) => 0.35,
	};
	if entering {
		Color::srgba(0.1, 1.0, 0.15, alpha)
	} else {
		Color::srgba(1.0, 0.1, 0.05, alpha)
	}
}

pub(crate) fn draw_waiting_coverage_gizmos(
	mut gizmos: Gizmos<CameraVoxelLoaderGizmos>,
	cameras: Query<&CameraVoxelLoader>,
	grid_transforms: Query<&GlobalTransform, With<GridStreaming>>,
	grids: Query<&Grid>,
	subgrids: Query<&SubGrid>,
	subgrid_gpu: Query<&SubGridGpuState, With<SubGrid>>,
	mut suspicious_requested_tile_frames: Local<HashMap<CoverageSource, u32>>,
) {
	let waiting_cell_color = Color::srgba(1.0, 0.0, 0.85, 0.95);
	let waiting_subgrid_color = Color::srgba(0.0, 1.0, 1.0, 0.9);
	let retiring_source_color = Color::srgba(1.0, 0.25, 0.0, 0.65);

	for camera_voxel_loader in &cameras {
		for (&retiring_source, record) in &camera_voxel_loader.coverage_sources {
			if !matches!(record.state, SourceState::RetiringVisible(_)) {
				continue;
			}
			draw_coverage_source_box(&mut gizmos, &grid_transforms, retiring_source, retiring_source_color);

			for &chunk in record.cells.iter() {
				let Some(cell) = camera_voxel_loader.coverage_cells.get(&chunk) else { continue };
				if coverage_cell_replacement_state(camera_voxel_loader, cell, retiring_source) != CoverageCellReplacementState::Waiting {
					continue;
				}
				draw_chunk_key_box(&mut gizmos, &grid_transforms, chunk, waiting_cell_color);

				for &desired_source in cell.desired.iter().filter(|&&source| source != retiring_source) {
					if !source_is_desired_requested(camera_voxel_loader, desired_source) {
						continue;
					}
					warn_if_suspicious_requested_tile(&mut suspicious_requested_tile_frames, camera_voxel_loader, desired_source, retiring_source, chunk);
					draw_coverage_source_box(&mut gizmos, &grid_transforms, desired_source, coverage_source_debug_color(camera_voxel_loader, desired_source));
					if let CoverageSource::Chunk(waiting_chunk) = desired_source {
						draw_waiting_subgrid_uploads(
							&mut gizmos,
							&grid_transforms,
							&grids,
							&subgrids,
							&subgrid_gpu,
							waiting_chunk,
							waiting_subgrid_color,
						);
					}
				}
			}
		}
	}
}

fn source_is_desired_requested(camera_voxel_loader: &CameraVoxelLoader, source: CoverageSource) -> bool {
	matches!(
		camera_voxel_loader.coverage_sources.get(&source).map(|record| record.state),
		Some(SourceState::Desired(SourceResolution::Requested))
	)
}

fn draw_cell_coverage_sources<C: GizmoConfigGroup>(
	gizmos: &mut Gizmos<C>,
	grid_transforms: &Query<&GlobalTransform, With<GridStreaming>>,
	camera_voxel_loader: &CameraVoxelLoader,
	retiring_source: CoverageSource,
	sources: impl Iterator<Item = CoverageSource>,
) {
	let mut drawn = Vec::new();
	for source in sources {
		if source == retiring_source || drawn.contains(&source) {
			continue;
		}
		drawn.push(source);
		draw_coverage_source_box(gizmos, grid_transforms, source, coverage_source_debug_color(camera_voxel_loader, source));
	}
}

fn coverage_source_debug_color(camera_voxel_loader: &CameraVoxelLoader, source: CoverageSource) -> Color {
	match camera_voxel_loader.coverage_sources.get(&source).map(|record| record.state) {
		Some(SourceState::Desired(SourceResolution::Requested)) => requested_source_debug_color(camera_voxel_loader, source),
		Some(SourceState::Desired(SourceResolution::Empty)) => Color::srgba(0.45, 0.48, 0.52, 0.75),
		Some(SourceState::Desired(SourceResolution::Visible(_))) => Color::srgba(0.1, 1.0, 0.25, 0.75),
		Some(SourceState::RetiringVisible(_)) => Color::srgba(1.0, 0.25, 0.0, 0.65),
		None => Color::srgba(1.0, 0.0, 0.0, 0.9),
	}
}

fn requested_source_debug_color(camera_voxel_loader: &CameraVoxelLoader, source: CoverageSource) -> Color {
	let CoverageSource::Tile(tile) = source else { return Color::srgba(1.0, 0.95, 0.05, 0.9) };
	match camera_voxel_loader.tiles.get(&tile).map(|record| record.status) {
		None => Color::srgba(1.0, 0.0, 0.0, 1.0),
		Some(TileStatus::Queued) => Color::srgba(1.0, 1.0, 1.0, 0.95),
		Some(TileStatus::Loading) => Color::srgba(1.0, 0.95, 0.05, 0.95),
		Some(TileStatus::LoadedWaitingGpu) => Color::srgba(0.0, 1.0, 1.0, 0.95),
		Some(TileStatus::Ready | TileStatus::Retiring) => Color::srgba(0.8, 0.0, 1.0, 0.95),
	}
}

fn draw_tile_record_lifecycle_gizmos<C: GizmoConfigGroup>(
	gizmos: &mut Gizmos<C>,
	grid_transforms: &Query<&GlobalTransform, With<GridStreaming>>,
	camera_voxel_loader: &CameraVoxelLoader,
) {
	for (&tile, record) in &camera_voxel_loader.tiles {
		draw_coverage_source_box(gizmos, grid_transforms, CoverageSource::Tile(tile), tile_record_debug_color(record.status));
	}
}

fn tile_record_debug_color(status: TileStatus) -> Color {
	match status {
		TileStatus::Queued => Color::srgba(1.0, 1.0, 1.0, 0.25),
		TileStatus::Loading => Color::srgba(1.0, 0.95, 0.05, 0.35),
		TileStatus::LoadedWaitingGpu => Color::srgba(0.0, 1.0, 1.0, 0.45),
		TileStatus::Ready => Color::srgba(0.0, 1.0, 0.25, 0.18),
		TileStatus::Retiring => Color::srgba(1.0, 0.25, 0.0, 0.45),
	}
}

fn warn_if_suspicious_requested_tile(
	suspicious_requested_tile_frames: &mut HashMap<CoverageSource, u32>,
	camera_voxel_loader: &CameraVoxelLoader,
	desired_source: CoverageSource,
	retiring_source: CoverageSource,
	blocked_chunk: ChunkKey,
) {
	let CoverageSource::Tile(tile) = desired_source else { return };
	let record = camera_voxel_loader.tiles.get(&tile);
	let suspicious = match record.map(|record| record.status) {
		None => true,
		Some(TileStatus::Ready | TileStatus::Retiring) => true,
		Some(TileStatus::Queued | TileStatus::Loading | TileStatus::LoadedWaitingGpu) => false,
	};
	if !suspicious {
		suspicious_requested_tile_frames.remove(&desired_source);
		return;
	}

	let frames = suspicious_requested_tile_frames.entry(desired_source).and_modify(|frames| *frames += 1).or_insert(1);
	if *frames != 1 && *frames % 120 != 0 {
		return;
	}

	let status = record.map(|record| record.status);
	let entity = record.and_then(|record| record.entity);
	let stale_entity = record.and_then(|record| record.stale_entity);
	let generation = record.map(|record| record.generation);
	let coverage_state = camera_voxel_loader.coverage_sources.get(&desired_source).map(|record| record.state);
	bevy::log::warn!(
		"camera voxel loader: suspicious requested LOD coverage; retiring_source={retiring_source:?}, blocked_chunk={blocked_chunk:?}, desired_tile={tile:?}, coverage={coverage_state:?}, tile_status={status:?}, entity={entity:?}, stale_entity={stale_entity:?}, generation={generation:?}, queue_contains={}, desired_tiles_contains={}, frames_seen={frames}",
		camera_voxel_loader.queue.iter().any(|queued| *queued == tile),
		camera_voxel_loader.desired_tiles.contains(&tile),
	);
}

fn draw_coverage_source_box<C: GizmoConfigGroup>(gizmos: &mut Gizmos<C>, grid_transforms: &Query<&GlobalTransform, With<GridStreaming>>, source: CoverageSource, color: Color) {
	match source {
		CoverageSource::Chunk(chunk) => draw_chunk_key_box(gizmos, grid_transforms, chunk, color),
		CoverageSource::Tile(tile) => {
			let Ok(grid_transform) = grid_transforms.get(tile.grid) else { return };
			draw_lod_bound_box(gizmos, grid_transform, tile.min, tile.min + tile.size(), color);
		}
	}
}

fn draw_chunk_key_box<C: GizmoConfigGroup>(gizmos: &mut Gizmos<C>, grid_transforms: &Query<&GlobalTransform, With<GridStreaming>>, chunk: ChunkKey, color: Color) {
	let Ok(grid_transform) = grid_transforms.get(chunk.grid) else { return };
	draw_lod_bound_box(gizmos, grid_transform, chunk.chunk, chunk.chunk + IVec3::ONE, color);
}

fn draw_waiting_subgrid_uploads<C: GizmoConfigGroup>(
	gizmos: &mut Gizmos<C>,
	grid_transforms: &Query<&GlobalTransform, With<GridStreaming>>,
	grids: &Query<&Grid>,
	subgrids: &Query<&SubGrid>,
	subgrid_gpu: &Query<&SubGridGpuState, With<SubGrid>>,
	chunk: ChunkKey,
	color: Color,
) {
	let Ok(grid) = grids.get(chunk.grid) else { return };
	let Ok(grid_transform) = grid_transforms.get(chunk.grid) else { return };
	let min = chunk.chunk * CHUNK_SIZE;
	for entity in grid.subgrid_entities_in_area(min, IVec3::splat(CHUNK_SIZE)) {
		if subgrid_gpu.get(entity).is_ok() {
			continue;
		}
		let Ok(subgrid) = subgrids.get(entity) else { continue };
		let min = subgrid.sub_grid_pos();
		let max = min + IVec3::splat(SUB_GRID_SIZE);
		draw_transformed_box(gizmos, grid_transform, min.as_vec3(), max.as_vec3(), color);
	}
}

fn draw_retiring_lod_gizmos(
	mut gizmos: Gizmos,
	cameras: Query<&CameraVoxelLoader>,
	grid_transforms: Query<&GlobalTransform, With<GridStreaming>>,
) {
	let color = Color::srgba(1.0, 0.15, 0.0, 0.9);
	for camera_voxel_loader in &cameras {
		for (key, record) in &camera_voxel_loader.tiles {
			if record.status != TileStatus::Retiring {
				continue;
			}
			let Ok(grid_transform) = grid_transforms.get(key.grid) else { continue };
			let min = (key.min * CHUNK_SIZE).as_vec3();
			let max = ((key.min + key.size()) * CHUNK_SIZE).as_vec3();
			draw_transformed_box(&mut gizmos, grid_transform, min, max, color);
		}
	}
}

fn draw_transformed_box<C: GizmoConfigGroup>(gizmos: &mut Gizmos<C>, transform: &GlobalTransform, min: Vec3, max: Vec3, color: Color) {
	let corner = |x: f32, y: f32, z: f32| transform.transform_point(Vec3::new(x, y, z));
	let c000 = corner(min.x, min.y, min.z);
	let c100 = corner(max.x, min.y, min.z);
	let c010 = corner(min.x, max.y, min.z);
	let c001 = corner(min.x, min.y, max.z);
	let c110 = corner(max.x, max.y, min.z);
	let c101 = corner(max.x, min.y, max.z);
	let c011 = corner(min.x, max.y, max.z);
	let c111 = corner(max.x, max.y, max.z);

	for (a, b) in [
		(c000, c100),
		(c010, c110),
		(c001, c101),
		(c011, c111),
		(c000, c010),
		(c100, c110),
		(c001, c011),
		(c101, c111),
		(c000, c001),
		(c100, c101),
		(c010, c011),
		(c110, c111),
	] {
		gizmos.line(a, b, color);
	}
}

