use std::collections::HashMap;

use bevy::prelude::*;
use voxel_data::grid::GridId;
use voxel_data::grid_tree::GridCell;

use crate::request_tree::{LodAreaDelta, LodRequestCell, LodRequestGridTree, LodVisibleDelta};

#[derive(Component, Debug)]
pub struct LodRequestMap {
    trees: HashMap<GridId, LodRequestGridTree>,
    area_delta: Vec<LodAreaDelta>,
    visible: Vec<LodVisibleDelta>,
    visible_added: Vec<LodVisibleDelta>,
    visible_removed: Vec<LodVisibleDelta>,
    priority: f32,
}

impl Default for LodRequestMap {
    fn default() -> Self {
        Self {
            trees: HashMap::new(),
            area_delta: Vec::new(),
            visible: Vec::new(),
            visible_added: Vec::new(),
            visible_removed: Vec::new(),
            priority: 0.0,
        }
    }
}

impl LodRequestMap {
    pub fn set_area(&mut self, grid: GridId, min: IVec3, size: IVec3, lod: u32) {
        if size.cmple(IVec3::ZERO).any() {
            return;
        }
        let lod = lod.min(LodRequestCell::MAX_DATA as u32);
        let tree = self.trees.entry(grid).or_insert_with(LodRequestGridTree::new);
        let old = tree.get(&min).map(|v| v as u32);
        if old == Some(lod) {
            return;
        }
        tree.add_area(&min, size, lod as u16);
        self.area_delta.push(LodAreaDelta { grid, min, size, old_lod: old, new_lod: Some(lod) });
    }

    pub fn remove_area(&mut self, grid: GridId, min: IVec3, size: IVec3) {
        if size.cmple(IVec3::ZERO).any() {
            return;
        }
        let Some(tree) = self.trees.get_mut(&grid) else {
            return;
        };
        let old = tree.get(&min).map(|v| v as u32);
        if old.is_none() {
            return;
        }
        tree.remove_area(&min, size);
        self.area_delta.push(LodAreaDelta { grid, min, size, old_lod: old, new_lod: None });
    }

    pub fn clear(&mut self) {
        for (&grid, tree) in &self.trees {
            for (min, size, lod) in tree.iter() {
                self.area_delta.push(LodAreaDelta { grid, min, size: IVec3::splat(size as i32), old_lod: Some(lod as u32), new_lod: None });
            }
        }
        self.trees.clear();
    }

    pub fn tree(&self, grid: GridId) -> Option<&LodRequestGridTree> {
        self.trees.get(&grid)
    }
    pub fn visible(&self) -> &[LodVisibleDelta] {
        &self.visible
    }
    pub fn set_priority(&mut self, priority: f32) {
        self.priority = priority;
    }
    pub fn priority(&self) -> f32 {
        self.priority
    }

    pub fn drain_visible_added_delta(&mut self) -> Vec<LodVisibleDelta> {
        std::mem::take(&mut self.visible_added)
    }
    pub fn drain_visible_removed_delta(&mut self) -> Vec<LodVisibleDelta> {
        std::mem::take(&mut self.visible_removed)
    }

    pub(crate) fn drain_area_delta(&mut self) -> Vec<LodAreaDelta> {
        std::mem::take(&mut self.area_delta)
    }

    pub(crate) fn replace_visible_in_area(&mut self, grid: GridId, min: IVec3, size: IVec3, next: Vec<LodVisibleDelta>) {
        let hi = min + size;
        let mut removed = Vec::new();
        self.visible.retain(|v| {
            let v_hi = v.min + v.size;
            let overlaps = v.grid == grid && v.min.cmplt(hi).all() && v_hi.cmpgt(min).all();
            if overlaps {
                removed.push(*v);
            }
            !overlaps
        });

        for v in removed {
            if !next.iter().any(|n| n.entity == v.entity && n.kind == v.kind) {
                self.visible_removed.push(v);
            }
        }
        for v in next {
            if !self.visible.iter().any(|old| old.entity == v.entity && old.kind == v.kind) {
                self.visible_added.push(v);
            }
            self.visible.push(v);
        }
    }
}
