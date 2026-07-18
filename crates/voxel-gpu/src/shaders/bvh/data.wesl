// Shared BVH data layout, bindings, and low-level helpers.

struct BVHNode {
	c0_min_and_ref:   vec4<f32>,
	c0_max_and_ref2:  vec4<f32>,
	c1_min_and_flags: vec4<f32>,
	c1_max_pad:       vec4<f32>,
}

struct BVHItem {
	min_x: f32, min_y: f32, min_z: f32,
	aabb_size:    u32,
	item_index:   u32,
	item_index_2: u32,
	pos_x:  f32, pos_y:  f32, pos_z:  f32,
	quat_x: f32, quat_y: f32, quat_z: f32, quat_w: f32,
	scale:  f32,
}

@group(1) @binding(0) var<storage, read> bvh:       array<BVHNode>;
@group(1) @binding(1) var<storage, read> bvh_items: array<BVHItem>;
