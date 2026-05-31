use std::collections::HashMap;

use bevy::prelude::*;

#[derive(SystemSet, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum GpuUploadSet {
	Clear,
	Collect,
	Upload,
}

#[derive(Clone, Copy, Debug)]
pub struct LodRequest {
	pub lod_level: f32,
	pub priority: f32,
}

/// The highest-quality request per sub-grid entity for the current frame,
/// rebuilt every frame from all registered requester components.
#[derive(Resource, Default)]
pub struct DesiredLods {
	map: HashMap<Entity, LodRequest>,
}

impl DesiredLods {
	pub fn clear(&mut self) {
		self.map.clear();
	}

	pub fn request(&mut self, entity: Entity, lod_level: f32, priority: f32) {
		self.map
			.entry(entity)
			.and_modify(|r| {
				r.lod_level = r.lod_level.min(lod_level);
				r.priority = r.priority.max(priority);
			})
			.or_insert(LodRequest { lod_level, priority });
	}

	pub fn get(&self, entity: Entity) -> Option<LodRequest> {
		self.map.get(&entity).copied()
	}
}

pub trait LodRequester: Component {
	fn lod_level(&self) -> f32;
	fn priority(&self) -> f32;
}

/// Defines a LOD-requester component. Lower `lod_level` is higher quality.
#[macro_export]
macro_rules! lod_requester {
	($vis:vis $name:ident) => {
		#[derive($crate::__bevy::prelude::Component, Clone, Copy, Debug)]
		$vis struct $name {
			pub lod_level: f32,
			pub priority: f32,
		}
		impl $crate::LodRequester for $name {
			fn lod_level(&self) -> f32 { self.lod_level }
			fn priority(&self) -> f32 { self.priority }
		}
	};
}

pub trait GpuVoxelDataAppExt {
	fn register_lod_requester<T: LodRequester>(&mut self) -> &mut Self;
}

impl GpuVoxelDataAppExt for App {
	fn register_lod_requester<T: LodRequester>(&mut self) -> &mut Self {
		self.add_systems(Update, collect_lod_requests::<T>.in_set(GpuUploadSet::Collect))
	}
}

fn collect_lod_requests<T: LodRequester>(query: Query<(Entity, &T)>, mut desired: ResMut<DesiredLods>) {
	for (entity, requester) in query.iter() {
		desired.request(entity, requester.lod_level(), requester.priority());
	}
}

pub(crate) fn clear_desired_lods(mut desired: ResMut<DesiredLods>) {
	desired.clear();
}
