// Vertex shader

struct VertexOutput {
	@builtin(position) clip_position: vec4<f32>,
	@location(0) screen_pos: vec2<f32>,
}

@vertex
fn vs_main(
	@builtin(vertex_index) vertex_index : u32,
) -> VertexOutput {
	var out: VertexOutput;
	out.screen_pos = vec2<f32>(f32((vertex_index << 1) & 2), f32(vertex_index & 2));
    out.clip_position = vec4<f32>(out.screen_pos * 2.0f + -1.0f, 0.0f, 1.0f);
	return out;
}

// Fragment shader
struct CameraUniform {
    camera_transform: mat4x4<f32>,
    camera_view_size: vec2<f32>,
};
@group(0) @binding(0)
var<uniform> camera: CameraUniform;

struct BVHNode {
    min_x: f32,
    min_y: f32,
    min_z: f32,
    max_x: f32,
    max_y: f32,
    max_z: f32,
    data_1_and_2: u32, // data_1 in low 16 bits, data_2 in high 16 bits
    is_leaf: u32,
}

@group(1) @binding(0)
var<storage, read> bvh: array<BVHNode>;

struct BVHItem {
    min_x: f32,
    min_y: f32,
    min_z: f32,
    max_x: f32,
    max_y: f32,
    max_z: f32,
    item_index: u32,
    _padding: u32,
}

@group(1) @binding(1)  // FIX: was binding(0), conflicting with bvh
var<storage, read> bvh_items: array<BVHItem>;

fn ray_aabb_intersection(ray_pos: vec3<f32>, ray_dir: vec3<f32>, aabb_min: vec3<f32>, aabb_max: vec3<f32>) -> f32 {
    let inv_dir = 1.0 / ray_dir;
    let t1 = (aabb_min - ray_pos) * inv_dir;
    let t2 = (aabb_max - ray_pos) * inv_dir;
    let t_min = max(max(min(t1.x, t2.x), min(t1.y, t2.y)), min(t1.z, t2.z));
    let t_max = min(min(max(t1.x, t2.x), max(t1.y, t2.y)), max(t1.z, t2.z));
    if t_max < 0.0 || t_min > t_max {
        return -1.0;
    }
    return max(t_min, 0.0);
}

const BVH_STACK_SIZE: u32 = 64u;

struct BVHHit {
    index: u32,
    dist: f32,
    steps: u32,
}

fn bvh_raycast(ray_pos: vec3<f32>, ray_dir: vec3<f32>, max_dist: f32) -> BVHHit {
    var stack: array<u32, 64>;
    var stack_top: i32 = 0;
    var stack_dist: array<f32, 64>;

    var best_dist = max_dist;
    var best_index = 0xFFFFFFFFu;
    var steps = 0u;

    let root_min = vec3<f32>(bvh[0].min_x, bvh[0].min_y, bvh[0].min_z);
    let root_max = vec3<f32>(bvh[0].max_x, bvh[0].max_y, bvh[0].max_z);
    let root_dist = ray_aabb_intersection(ray_pos, ray_dir, root_min, root_max);
    if root_dist < 0.0 || root_dist > best_dist {
        return BVHHit(best_index, -1.0, 0u);
    }

    stack[0] = 0u;
    stack_dist[0] = root_dist;
    stack_top = 1;

    while stack_top > 0 {
        stack_top -= 1;
        steps += 1u;
        let node_idx = stack[stack_top];
        let node_entry_dist = stack_dist[stack_top];

        if node_entry_dist > best_dist {
            continue;
        }

        let node = bvh[node_idx];
        let data_1 = node.data_1_and_2 & 0xFFFFu;
        let data_2 = (node.data_1_and_2 >> 16u) & 0xFFFFu;

        if node.is_leaf != 0u {
            let start = data_1;
            let count = data_2;
            // FIX: test each item's own AABB, track closest hit, store item_index
            for (var i = 0u; i < count; i++) {
                let item = bvh_items[start + i];
                let item_min = vec3<f32>(item.min_x, item.min_y, item.min_z);
                let item_max = vec3<f32>(item.max_x, item.max_y, item.max_z);
                let item_dist = ray_aabb_intersection(ray_pos, ray_dir, item_min, item_max);
                if item_dist >= 0.0 && item_dist < best_dist {
                    best_dist = item_dist;
                    best_index = item.item_index;
                }
            }
        } else {
            let sub1 = data_1;
            let sub2 = data_2;

            let n1 = bvh[sub1];
            let n1_min = vec3<f32>(n1.min_x, n1.min_y, n1.min_z);
            let n1_max = vec3<f32>(n1.max_x, n1.max_y, n1.max_z);
            let d1 = ray_aabb_intersection(ray_pos, ray_dir, n1_min, n1_max);

            let n2 = bvh[sub2];
            let n2_min = vec3<f32>(n2.min_x, n2.min_y, n2.min_z);
            let n2_max = vec3<f32>(n2.max_x, n2.max_y, n2.max_z);
            let d2 = ray_aabb_intersection(ray_pos, ray_dir, n2_min, n2_max);

            if d1 >= 0.0 && d1 <= best_dist && d2 >= 0.0 && d2 <= best_dist {
                if d1 < d2 {
                    stack[stack_top] = sub2; stack_dist[stack_top] = d2; stack_top += 1;
                    stack[stack_top] = sub1; stack_dist[stack_top] = d1; stack_top += 1;
                } else {
                    stack[stack_top] = sub1; stack_dist[stack_top] = d1; stack_top += 1;
                    stack[stack_top] = sub2; stack_dist[stack_top] = d2; stack_top += 1;
                }
            } else if d1 >= 0.0 && d1 <= best_dist {
                stack[stack_top] = sub1; stack_dist[stack_top] = d1; stack_top += 1;
            } else if d2 >= 0.0 && d2 <= best_dist {
                stack[stack_top] = sub2; stack_dist[stack_top] = d2; stack_top += 1;
            }
        }
    }

    if best_index == 0xFFFFFFFFu {
        return BVHHit(0u, -1.0, steps);
    }
    return BVHHit(best_index, best_dist, steps);
}

// FIX: debug color per item index using hue cycling
fn index_to_color(index: u32) -> vec3<f32> {
    let hue = f32(index % 17u) / 17.0; // 17 gives good visual separation
    let h = hue * 6.0;
    let i = u32(h);
    let f = h - f32(i);
    let q = 1.0 - f;
    switch i % 6u {
        case 0u: { return vec3<f32>(1.0, f,   0.0); }
        case 1u: { return vec3<f32>(q,   1.0, 0.0); }
        case 2u: { return vec3<f32>(0.0, 1.0, f  ); }
        case 3u: { return vec3<f32>(0.0, q,   1.0); }
        case 4u: { return vec3<f32>(f,   0.0, 1.0); }
        default: { return vec3<f32>(1.0, 0.0, q  ); }
    }
}

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    let ray_start = camera.camera_transform[3].xyz;
    let ray_dir = normalize((camera.camera_transform * vec4<f32>(
        (-in.screen_pos.x + 0.5) * camera.camera_view_size.x * 2.0,
        (in.screen_pos.y - 0.5) * camera.camera_view_size.y * 2.0,
        1.0,
        0.0
    )).xyz);

    let hit = bvh_raycast(ray_start, ray_dir, 1e38);

    if hit.dist < 0.0 {
        return vec4<f32>(0.05, 0.05, 0.05, 1.0); // background
    }

    // FIX: color by item index for debug visualization
    let color = index_to_color(hit.index);
    return vec4<f32>(color, 1.0);
}