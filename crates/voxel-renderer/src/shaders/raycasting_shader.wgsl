// 0 : camera uniform
// 1 : BVH (nodes + items)
// 2 : grid tree buffer
// 3 : intermediate_textured (write)
// 4 : bvh_beam_textured (read)

@group(3) @binding(0) var intermediate_textured: texture_storage_2d<rgba32uint, write>;
@group(4) @binding(0) var bvh_beam_textured: texture_2d<f32>;

struct CameraUniform {
	camera_transform: mat4x4<f32>,
	camera_view_size: vec2<f32>,
}
@group(0) @binding(0) var<uniform> camera: CameraUniform;

struct RenderSettings {
	values: vec4<u32>,
}
@group(0) @binding(1) var<uniform> render_settings: RenderSettings;

@compute
@workgroup_size(8, 4)
fn main(@builtin(global_invocation_id) global_invocation_id: vec3<u32>) {
	let texture_size = textureDimensions(intermediate_textured);
	let pixel = global_invocation_id.xy;
	if pixel.x >= texture_size.x || pixel.y >= texture_size.y { return; }

	// Pixel centre in [0, 1] — consistent with both beam shader and coloring.
	let screen_pos = (vec2<f32>(pixel) + vec2<f32>(0.5)) / vec2<f32>(texture_size);

	let ray_start = camera.camera_transform[3].xyz;
	let ray_dir = normalize((camera.camera_transform * vec4<f32>(
		(-screen_pos.x + 0.5) * camera.camera_view_size.x * 2.0,
		( screen_pos.y - 0.5) * camera.camera_view_size.y * 2.0,
		1.0,
		0.0,
	)).xyz);

	// -- Beam early-out --------------------------------------------------------
	//
	// Convert this pixel's position into beam-texture space. The beam texture
	// is BVH_BEAM_TEXTURE_FACTOR* smaller, so beam_uv maps to the same
	// fractional position.
	//
	// We sample the 2*2 block of beam texels whose corners surround the
	// sub-texel position (i.e. the four nearest neighbours), then take the
	// minimum. This is conservative: if any neighbouring beam ray hit
	// something nearby, we use that distance as an upper bound. We must use
	// the *minimum* (not maximum) because we need a bound that is definitely
	// no farther than the true nearest hit visible from this pixel.
	//
	// Using textureLoad (not textureSample) because the texture is unfilterable
	// float (r32float with filterable: false).
	let beam_size = textureDimensions(bvh_beam_textured);
	// Beam-space floating-point coordinate of this pixel's centre.
	// Subtract 0.5 so that the integer part indexes the texel whose *centre*
	// is just above/left of this pixel, giving us the correct 2*2 neighbourhood.
	let beam_coord_f = screen_pos * vec2<f32>(beam_size) - vec2<f32>(0.5);
	let beam_coord_i = vec2<i32>(beam_coord_f);   // top-left of the 2*2 block

	// Clamp each of the four corners to valid texel range.
	let bsz = vec2<i32>(beam_size);
	let c00 = clamp(beam_coord_i,                   vec2<i32>(0), bsz - vec2<i32>(1));
	let c10 = clamp(beam_coord_i + vec2<i32>(1, 0), vec2<i32>(0), bsz - vec2<i32>(1));
	let c01 = clamp(beam_coord_i + vec2<i32>(0, 1), vec2<i32>(0), bsz - vec2<i32>(1));
	let c11 = clamp(beam_coord_i + vec2<i32>(1, 1), vec2<i32>(0), bsz - vec2<i32>(1));

	let d00 = textureLoad(bvh_beam_textured, c00, 0).r;
	let d10 = textureLoad(bvh_beam_textured, c10, 0).r;
	let d01 = textureLoad(bvh_beam_textured, c01, 0).r;
	let d11 = textureLoad(bvh_beam_textured, c11, 0).r;

	// Minimum of the four neighbours
	let beam_min_dist = min(min(d00, d10), min(d01, d11));

	// If beam_max_dist >= 1e37 all four neighbours were misses
	if (beam_min_dist >= 1e37) {
		textureStore(intermediate_textured, pixel, vec4<u32>(0, 0, 0, 0));
		return;
	}
	// textureStore(intermediate_textured, pixel, vec4<u32>(u32(beam_min_dist * 10.0), 0, 0, 0));
	// return;

	let hit = full_raycast(ray_start + ray_dir * beam_min_dist, ray_dir, 1e38);

	let sun_dir = normalize(vec3<f32>(0.5, 1.0, 0.2));

	if hit.normal == 0 {
		textureStore(intermediate_textured, pixel, vec4<u32>(0, 0, 0, 0));
		return;
	}

	var light_visible = true;
	if render_settings.values.x != 0u {
		let hit_pos = ray_start + (hit.total_dist + beam_min_dist - 0.01) * ray_dir;
		let sky_hit = full_raycast(hit_pos, sun_dir, 1e38);
		light_visible = sky_hit.normal == 0;
	}
	textureStore(intermediate_textured, pixel, vec4<u32>(
		hit.normal + 256u * u32(light_visible),
		hit.bvh_item_idx,
		hit.voxel_data_index + (hit.voxel_index << 16),
		bitcast<u32>(hit.total_dist + beam_min_dist),
	));
}
