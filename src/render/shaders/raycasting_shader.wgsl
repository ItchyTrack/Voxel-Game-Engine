// main_shader.wgsl
// 0 : camera uniform
// 1 : BVH (nodes + items)
// 2 : grid tree buffer
// 3 : intermediate_textured

@group(3) @binding(0) var intermediate_textured: texture_storage_2d<rgba32uint, write>;

// Camera uniform
struct CameraUniform {
    camera_transform: mat4x4<f32>,
    camera_view_size: vec2<f32>,
}
@group(0) @binding(0) var<uniform> camera: CameraUniform;

@compute
@workgroup_size(8, 4)
fn main(@builtin(global_invocation_id) global_invocation_id: vec3<u32>) {
	let texture_size = textureDimensions(intermediate_textured);
	let screen_pos = vec2<f32>(f32(global_invocation_id.x) / f32(texture_size.x), f32(global_invocation_id.y) / f32(texture_size.y));
	if (screen_pos.x >= 1.0 || screen_pos.y >= 1.0) { return; }

    // Reconstruct world-space ray from the screen position.
    let ray_start = camera.camera_transform[3].xyz;
    let ray_dir = normalize((camera.camera_transform * vec4<f32>(
        (-screen_pos.x + 0.5) * camera.camera_view_size.x * 2.0,
        ( screen_pos.y - 0.5) * camera.camera_view_size.y * 2.0,
        1.0,
        0.0,
    )).xyz);

    let hit = full_raycast(ray_start, ray_dir, 1e38);

	let sun_dir = normalize(vec3<f32>(0.5, 1.0, 0.2));

    if hit.normal == 0 {
		textureStore(intermediate_textured, global_invocation_id.xy, vec4<u32>(0, 0, 0, 0));
		return;
    }

	let hit_pos = ray_start + (hit.total_dist - 0.01) * ray_dir;
	// let sky_hit = full_raycast(hit_pos, sun_dir, 1e38);
    // let light_visible = sky_hit.normal == 0;
    let light_visible = true;
	textureStore(intermediate_textured, global_invocation_id.xy, vec4<u32>(
		hit.normal + 256u * u32(light_visible),
		hit.bvh_item_idx,
		hit.voxel_data_index + (hit.voxel_index << 16),
		bitcast<u32>(hit.total_dist),
	));
}
