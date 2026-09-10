pub mod aabb;
pub mod bvh;
mod body;
mod query;
mod scale;
mod transform;

pub use body::{Body, BodyId};
pub use query::TransformQuery;
pub use scale::Scale;
pub use transform::Transform;

use bevy::{camera::{CameraUpdateSystems, visibility::VisibilitySystems}, prelude::{App, Camera, Camera3d, Entity, GlobalTransform, IntoScheduleConfigs, Plugin, PostUpdate, Query, ResMut, Resource, SystemSet, With}};
use voxel_math::FixedVec3;

/// Shared fixed-point origin for all rendering snapshots.
#[derive(Resource, Default, Debug, Clone, Copy, PartialEq, Eq)]
pub struct RenderOrigin(pub FixedVec3);

#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum TransformSystems {
	/// Schedule simulation before this set and render consumers after it.
	PrepareRender,
}

#[derive(Default)]
pub struct VoxelTransformPlugin;

impl Plugin for VoxelTransformPlugin {
	fn build(&self, app: &mut App) {
		app.init_resource::<RenderOrigin>()
			.configure_sets(PostUpdate, TransformSystems::PrepareRender
				.before(CameraUpdateSystems)
				.before(VisibilitySystems::CalculateBounds)
				.before(VisibilitySystems::UpdateFrusta)
				.before(VisibilitySystems::VisibilityPropagate)
				.before(VisibilitySystems::CheckVisibility))
			.add_systems(PostUpdate, prepare_cameras.in_set(TransformSystems::PrepareRender));
	}
}

fn prepare_cameras(
	transforms: TransformQuery,
	mut origin: ResMut<RenderOrigin>,
	mut cameras: Query<(Entity, &Camera, &mut bevy::prelude::Transform, &mut GlobalTransform), (With<Camera3d>, With<Transform>)>,
) {
	let anchor = cameras.iter()
		.filter(|(_, camera, _, _)| camera.is_active)
		.filter_map(|(entity, _, _, _)| transforms.get_world(entity).map(|world| (entity, world.translation)))
		.min_by_key(|(entity, _)| entity.to_bits());
	origin.0 = anchor.map_or(FixedVec3::ZERO, |(_, translation)| translation);
	for (entity, _, mut local, mut global) in &mut cameras {
		let Some(world) = transforms.get_world(entity) else { continue };
		let snapshot = world.relative_to(origin.0);
		*global = GlobalTransform::from(snapshot);
		*local = snapshot;
	}
}
