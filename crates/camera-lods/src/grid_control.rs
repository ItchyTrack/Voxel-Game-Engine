use bevy::prelude::*;
use lod_manager::{LoadedLodEvent, LodKey, LodRequestMap, LodRetainCount};

use crate::debug::{CameraLodDebug, CameraLodDebugState};
use crate::render_set::CameraVoxelRenderSet;

/// Low-level camera LOD controller.
///
/// This is the imperative half of camera LODs: policy code calls `set_lod` /
/// `release_lod`, while this component owns renderer swap state and loaded-delta
/// handling.
#[derive(Component, Default, Debug)]
pub struct CameraLodGridControl {
	visible_lods: Vec<LodKey>,
	waiting_lods: Vec<LodKey>,
}

impl CameraLodGridControl {
	pub fn set_lod(&mut self, requests: &mut LodRequestMap, key: LodKey, priority: f32) {
		requests.request_gpu(key, priority);
		if !self.waiting_lods.contains(&key) && !self.visible_lods.contains(&key) {
			self.waiting_lods.push(key);
		}
	}

	pub fn release_lod(&mut self, requests: &mut LodRequestMap, key: LodKey) {
		requests.release(key);
		self.waiting_lods.retain(|existing| *existing != key);
		self.visible_lods.retain(|existing| *existing != key);
	}

	pub fn visible_lods(&self) -> &[LodKey] {
		&self.visible_lods
	}

	pub fn waiting_lods(&self) -> &[LodKey] {
		&self.waiting_lods
	}

	pub fn note_loaded(&mut self, event: LoadedLodEvent, render_set: &mut CameraVoxelRenderSet) {
		self.waiting_lods.retain(|key| *key != event.key);
		if !self.visible_lods.contains(&event.key) {
			self.visible_lods.push(event.key);
		}
		render_set.show_lod(event.entity);
	}
}

/// Drain manager loaded deltas into the camera render set.
pub fn apply_loaded_lod_deltas(
	mut cameras: Query<(
		&mut CameraLodGridControl,
		&mut LodRequestMap,
		&mut CameraVoxelRenderSet,
		Option<&mut CameraLodDebug>,
	)>,
) {
	for (mut control, mut requests, mut render_set, debug) in cameras.iter_mut() {
		for event in requests.drain_loaded_delta() {
			control.note_loaded(event, &mut render_set);
		}

		if let Some(mut debug) = debug {
			debug.clear();
			for key in control.visible_lods() {
				debug.push_area(key.grid, key.min, key.size, CameraLodDebugState::Lod(key.level));
			}
			for key in control.waiting_lods() {
				debug.push_area(key.grid, key.min, key.size, CameraLodDebugState::WaitingOnLod);
			}
		}
	}
}

/// Convenience helper for systems that need to hold an old LOD during a visual swap.
pub fn retain_lod_entity(mut retains: Query<&mut LodRetainCount>, entity: Entity) {
	if let Ok(mut retain) = retains.get_mut(entity) {
		retain.retain();
	}
}

pub fn release_lod_entity(mut retains: Query<&mut LodRetainCount>, entity: Entity) {
	if let Ok(mut retain) = retains.get_mut(entity) {
		retain.release();
	}
}
