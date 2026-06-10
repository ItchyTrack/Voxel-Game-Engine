use std::collections::HashMap;

use bevy::prelude::*;
use lod_manager::{LodKey, LodRequestMap};

#[derive(Component, Default, Debug)]
pub struct CameraLodGridControl {
    active: HashMap<LodKey, f32>,
}

impl CameraLodGridControl {
    pub fn sync(&mut self, requests: &mut LodRequestMap, desired: impl IntoIterator<Item = (LodKey, f32)>) {
        let desired: HashMap<LodKey, f32> = desired.into_iter().collect();

        for key in self.active.keys() {
            if !desired.contains_key(key) {
                requests.remove_area(key.grid, key.min, key.size);
            }
        }

        for (&key, &priority) in &desired {
            if self.active.contains_key(&key) {
                continue;
            }
            requests.set_priority(priority);
            requests.set_area(key.grid, key.min, key.size, key.level);
        }

        self.active = desired;
    }
}
