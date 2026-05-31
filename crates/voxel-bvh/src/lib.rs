pub mod bvh;
pub mod gpu_bvh;

use bevy::prelude::*;

/// Placeholder plugin. The CPU/GPU BVH resources are currently rebuilt on
/// demand by the renderer and physics; this plugin exists so the crate can own
/// shared BVH state in the future.
#[derive(Default)]
pub struct VoxelBvhPlugin;

impl Plugin for VoxelBvhPlugin {
	fn build(&self, _app: &mut App) {}
}
