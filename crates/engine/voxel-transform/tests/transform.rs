use bevy::{ecs::system::SystemState, math::{IVec3, Quat}, prelude::{App, Camera, Camera3d, ChildOf, GlobalTransform, PostUpdate, World}};
use voxel_data::grid::GridLocalTransform;
use voxel_math::{Fixed, FixedVec3, Ray};
use voxel_transform::{Body, RenderOrigin, Scale, Transform, TransformQuery, VoxelTransformPlugin, aabb::aabb_of_transformed_aabb, bvh::BVH};

#[test]
fn scale_rounds_ties_even_and_rejects_invalid_results() {
	assert_eq!(Scale::ONE.to_bits(), 1 << 16);
	assert_eq!(Scale::from_num(2).to_fixed(), Fixed::from_num(2));
	assert_eq!(Scale::from_num(0.5).to_bits(), 1 << 15);
	assert!(Scale::from_bits(0).is_none());
	assert!(Scale::checked_from_fixed(Fixed::NEG_ONE).is_none());
	assert!(Scale::checked_from_fixed(Fixed::from_bits(128)).is_none());
	assert_eq!(Scale::from_fixed(Fixed::from_bits(384)).to_bits(), 2);
	assert_eq!(Scale::from_fixed(Fixed::from_bits(640)).to_bits(), 2);
	assert!(Scale::checked_from_fixed(Fixed::MAX).is_none());
	assert!(Scale::from_bits(u32::MAX).unwrap().checked_mul(Scale::from_num(2)).is_none());
	assert!(Scale::from_bits(1).unwrap().checked_mul(Scale::from_bits(1).unwrap()).is_none());
	assert!(Scale::from_bits(1).unwrap().checked_recip().is_none());
	assert_eq!(Scale::from_num(2).recip(), Scale::from_num(0.5));
}

#[test]
fn constructors_composition_inverse_and_trait_use_fixed_coordinates() {
	let parent = Transform::from_xyz(10, 2.5, -3).with_scale(Scale::from_num(2));
	let child = Transform::from_xyz(1.5, 0, 4);
	let world = parent * child;
	assert_eq!(world.translation, Transform::from_xyz(13, 2.5, 5).translation);
	let point = FixedVec3::from(IVec3::new(3, -2, 7));
	assert_eq!(world * point, parent * (child * point));
	assert_eq!(world.inverse() * (world * point), point);
	assert_eq!(voxel_math::Transform::transform_point(&world, point), world * point);
	assert_eq!(world.inverse_transform_point(world * point), point);
	assert_eq!(world.transform_vector(point), point * Fixed::from_num(2));
	let rotation = Transform::from_rotation(Quat::from_rotation_y(std::f32::consts::FRAC_PI_2));
	assert!((rotation * FixedVec3::Z).abs_diff_eq(FixedVec3::X, Fixed::from_bits(4)));
	assert_eq!(Transform::IDENTITY.looking_to(FixedVec3::NEG_Z, FixedVec3::Y).rotation, Quat::IDENTITY);
	assert!(Transform::IDENTITY.looking_at(FixedVec3::X, FixedVec3::Y).forward().abs_diff_eq(FixedVec3::X, Fixed::from_bits(4)));
}

#[test]
fn render_origin_is_subtracted_before_float_conversion() {
	let origin = Transform::from_xyz(1i64 << 32, 0, 0).translation;
	let transform = Transform::from_translation(origin + FixedVec3::X * Fixed::from_num(0.25));
	assert_eq!(transform.relative_to(origin).translation.x, 0.25);
}

#[test]
fn world_query_recomposes_current_parents_and_integer_offsets() {
	let mut world = World::new();
	let root = world.spawn((Body, Transform::from_xyz(1i64 << 32, 0, 0).with_scale(Scale::from_num(2)))).id();
	let group = world.spawn(ChildOf(root)).id();
	let tile = world.spawn((GridLocalTransform(IVec3::new(32, -64, 96)), ChildOf(group))).id();
	let empty = world.spawn_empty().id();
	let mut query = SystemState::<TransformQuery>::new(&mut world);
	let expected = Transform::from_xyz((1i64 << 32) + 64, -128, 192).with_scale(Scale::from_num(2));
	assert_eq!(query.get(&world).unwrap().get_world(tile), Some(expected));
	assert_eq!(query.get(&world).unwrap().get_world(empty), None);
	world.get_mut::<Transform>(root).unwrap().translation += FixedVec3::Y;
	assert_eq!(query.get(&world).unwrap().get_world(tile).unwrap().translation, expected.translation + FixedVec3::Y);
	assert!(world.get::<bevy::prelude::Transform>(root).is_none());
	world.entity_mut(root).insert(ChildOf(tile));
	assert_eq!(query.get(&world).unwrap().get_world(tile), None);
}

#[test]
fn camera_snapshots_share_the_first_active_camera_origin() {
	let mut app = App::new();
	app.add_plugins(VoxelTransformPlugin);
	let root = app.world_mut().spawn((Body, Transform::from_xyz(1i64 << 32, 0, 0))).id();
	let first = app.world_mut().spawn((Camera3d::default(), Transform::from_xyz(1, 0, 0), ChildOf(root))).id();
	let second = app.world_mut().spawn((Camera3d::default(), Transform::from_xyz(5, 0, 0), ChildOf(root))).id();
	app.world_mut().run_schedule(PostUpdate);
	let anchor_x = if first.to_bits() < second.to_bits() { 1 } else { 5 };
	let origin = app.world().resource::<RenderOrigin>().0;
	assert_eq!(origin, Transform::from_xyz((1i64 << 32) + anchor_x, 0, 0).translation);
	for (camera, x) in [(first, 1), (second, 5)] {
		assert_eq!(app.world().get::<bevy::prelude::Transform>(camera).unwrap().translation.x, (x - anchor_x) as f32);
		assert_eq!(app.world().get::<GlobalTransform>(camera).unwrap().translation().x, (x - anchor_x) as f32);
		assert_eq!(app.world().get::<Transform>(camera).unwrap().translation.x, Fixed::from_num(x));
	}
	app.world_mut().get_mut::<Camera>(first).unwrap().is_active = false;
	app.world_mut().run_schedule(PostUpdate);
	assert_eq!(app.world().resource::<RenderOrigin>().0, Transform::from_xyz((1i64 << 32) + 5, 0, 0).translation);
}

#[test]
fn bvh_queries_preserve_large_fixed_positions_and_parallel_rays() {
	let origin = Transform::from_xyz(1i64 << 32, 0, 0).translation;
	let items = (0..32u16).map(|i| {
		let min = origin + FixedVec3::Z * Fixed::from_num(i * 4);
		(i, (min, min + FixedVec3::ONE))
	}).collect();
	let bvh = BVH::new(items);
	let ray = Ray { origin: origin - FixedVec3::Z, direction: FixedVec3::Z };
	let hits: Vec<_> = bvh.raycast(&ray, Some(Fixed::from_num(10))).collect();
	assert_eq!(hits, vec![(0, Fixed::ONE), (1, Fixed::from_num(5)), (2, Fixed::from_num(9))]);
	let outside = Ray { origin: ray.origin - FixedVec3::X, ..ray };
	assert_eq!(bvh.raycast(&outside, None).next(), None);
	assert_eq!(bvh.collisions(&(origin, origin + FixedVec3::ONE)), vec![0]);
	let transform = Transform::from_translation(origin).with_scale(Scale::from_num(2));
	assert_eq!(aabb_of_transformed_aabb(&transform, FixedVec3::ZERO, FixedVec3::ONE), (origin, origin + FixedVec3::ONE * Fixed::from_num(2)));
}
