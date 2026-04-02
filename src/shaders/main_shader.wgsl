// main_shader.wgsl
// Vertex + fragment entry points.
// In production, prepend (or #include) bvh_raycast.wgsl, dda_raycast.wgsl,
// and combined_raycast.wgsl before this file.
//
// Groups:
//   0 : camera uniform
//   1 : BVH (nodes + items)   — defined in bvh_raycast.wgsl
//   2 : grid tree buffer      — defined in dda_raycast.wgsl

// ── Vertex ────────────────────────────────────────────────────────────────────

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) screen_pos: vec2<f32>,
}

@vertex
fn vs_main(@builtin(vertex_index) vertex_index: u32) -> VertexOutput {
    var out: VertexOutput;
    out.screen_pos    = vec2<f32>(f32((vertex_index << 1u) & 2u), f32(vertex_index & 2u));
    out.clip_position = vec4<f32>(out.screen_pos * 2.0 - 1.0, 0.0, 1.0);
    return out;
}

// ── Camera uniform ────────────────────────────────────────────────────────────

struct CameraUniform {
    // Column-major 4x4.  [3].xyz is the camera world position.
    // Upper-left 3x3 is the camera-to-world rotation.
    camera_transform: mat4x4<f32>,
    // tan(fov/2) per axis, used to reconstruct view-space ray directions.
    camera_view_size: vec2<f32>,
}
@group(0) @binding(0) var<uniform> camera: CameraUniform;

// ── Simple lighting / shading helpers ────────────────────────────────────────

fn voxel_color(value: u32) -> vec3<f32> {
    // Hue-cycle by voxel data value — replace with palette lookup as needed.
    let h = f32(value % 17u) / 17.0 * 6.0;
    let s = u32(h); let f = h - f32(s); let q = 1.0 - f;
    switch s % 6u {
        case 0u: { return vec3<f32>(1.0, f,   0.0); }
        case 1u: { return vec3<f32>(q,   1.0, 0.0); }
        case 2u: { return vec3<f32>(0.0, 1.0, f  ); }
        case 3u: { return vec3<f32>(0.0, q,   1.0); }
        case 4u: { return vec3<f32>(f,   0.0, 1.0); }
        default: { return vec3<f32>(1.0, 0.0, q  ); }
    }
}

fn shade(base_color: vec3<f32>, world_normal: vec3<f32>/*, hit_sky: bool*/) -> vec3<f32> {
	let light_dir = normalize(vec3<f32>(0.5, 1.0, 0.2));
    // Simple directional + ambient.
    let ndotl     = max(dot(world_normal, light_dir), 0.0);
	// using hit sky
	// let ndotl = select(0, 0.7, hit_sky);
    let ambient = 0.25;
    return base_color * (ambient + (1.0 - ambient) * ndotl);
}

// ── Fragment ──────────────────────────────────────────────────────────────────

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    // Reconstruct world-space ray from the screen position.
    let ray_start = camera.camera_transform[3].xyz;
    let ray_dir   = normalize((camera.camera_transform * vec4<f32>(
        (-in.screen_pos.x + 0.5) * camera.camera_view_size.x * 2.0,
        ( in.screen_pos.y - 0.5) * camera.camera_view_size.y * 2.0,
        1.0,
        0.0,
    )).xyz);

    // Full two-pass raycast: BVH broad phase → DDA precise voxel test.
    let hit = full_raycast(ray_start, ray_dir, 1e38);

    if !hit.hit {
        // Sky / background.
        let t  = ray_dir.y * 0.5 + 0.5;
        let bg = mix(vec3<f32>(0.15, 0.15, 0.18), vec3<f32>(0.05, 0.07, 0.12), t);
        return vec4<f32>(bg, 1.0);
    }

	// let light_dir = normalize(vec3<f32>(0.5, 1.0, 0.2));
	// let hit_pos = ray_start + (hit.total_dist - 0.01) * ray_dir;
	// let sky_hit = full_raycast(hit_pos, light_dir, 1e38);

	let item_index = bvh_items[hit.bvh_item_idx].item_index;   // grid tree offset
	let base_color = dda_palette_color(item_index, hit.voxel_value).xyz;
    let color = shade(base_color, hit.world_normal/*, !sky_hit.hit*/);

    return vec4<f32>(color, 1.0);

	// debug
    // return vec4<f32>(f32(hit.dda_iter_count) / 200.0, f32(hit.dda_iter_count) / 200.0, f32(hit.dda_iter_count) / 200.0, 1.0);
}
