use bevy::prelude::*;

/// Per-camera whitelist consumed by the renderer.
///
/// If this component is absent, the renderer draws every loaded sub-grid/LOD. If it is
/// present, only listed entities are considered for residency and extraction.
#[derive(Component, Debug, Clone)]
pub struct CameraVoxelRenderSet {
	/// When false, renderer systems ignore this whitelist and draw all loaded voxel data.
	pub active: bool,
	pub subgrids: Vec<Entity>,
	pub lods: Vec<Entity>,
}

impl Default for CameraVoxelRenderSet {
	fn default() -> Self {
		Self { active: false, subgrids: Vec::new(), lods: Vec::new() }
	}
}

impl CameraVoxelRenderSet {
	pub fn clear(&mut self) {
		self.active = false;
		self.subgrids.clear();
		self.lods.clear();
	}

	pub fn show_subgrid(&mut self, entity: Entity) {
		self.active = true;
		if !self.subgrids.contains(&entity) {
			self.subgrids.push(entity);
		}
	}

	pub fn hide_subgrid(&mut self, entity: Entity) {
		self.subgrids.retain(|e| *e != entity);
	}

	pub fn replace_subgrids(&mut self, subgrids: impl IntoIterator<Item = Entity>) {
		self.subgrids.clear();
		for entity in subgrids {
			self.show_subgrid(entity);
		}
		self.active = !self.subgrids.is_empty() || !self.lods.is_empty();
	}

	pub fn show_lod(&mut self, entity: Entity) {
		self.active = true;
		if !self.lods.contains(&entity) {
			self.lods.push(entity);
		}
	}

	pub fn hide_lod(&mut self, entity: Entity) {
		self.lods.retain(|e| *e != entity);
	}
}
