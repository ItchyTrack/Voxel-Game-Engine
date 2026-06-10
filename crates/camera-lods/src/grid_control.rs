use bevy::prelude::*;
use lod_manager::{LodKey, LodRequestMap};

use crate::debug::CameraLodDebug;

#[derive(Component, Default, Debug)]
pub struct CameraLodGridControl {
	active: Vec<(LodKey, f32)>,
}

impl CameraLodGridControl {
	pub fn sync(&mut self, requests: &mut LodRequestMap, desired: impl IntoIterator<Item = (LodKey, f32)>) {
		let desired: Vec<_> = desired.into_iter().collect();
		for (key, _) in self
			.active
			.iter()
			.copied()
			.filter(|(key, _)| !desired.iter().any(|(next, _)| next == key))
		{
			requests.remove_area(key.grid, key.min, key.size);
		}
		for (key, priority) in &desired {
			if self.active.iter().any(|(old, old_priority)| old == key && old_priority != priority) {
				requests.remove_area(key.grid, key.min, key.size);
			}
			requests.set_priority(*priority);
			requests.set_area(key.grid, key.min, key.size, key.level);
		}
		self.active = desired;
	}
}

pub fn update_camera_lod_debug(mut cameras: Query<(&LodRequestMap, Option<&mut CameraLodDebug>)>) {
	for (requests, debug) in cameras.iter_mut() {
		if let Some(mut debug) = debug {
			debug.clear();
			for v in requests.visible() {
				debug.push_area(v.grid, v.min, v.size, v.actual_lod);
			}
		}
	}
}
