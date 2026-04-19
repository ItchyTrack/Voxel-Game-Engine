// beam_bvh_raycast.wgsl
//
// Low-resolution BVH-only ray cast. Writes the nearest AABB hit distance
// (or 1e38 for a miss) into bvh_beam_textured (r32float).
//
// The high-resolution raycasting pass samples a small neighbourhood of this
// texture and uses the minimum value as an initial max_dist, culling all BVH
// items that lie beyond the nearest depth already found by a surrounding beam.
//
// -- AABB expansion -----------------------------------------------------------
//
// The beam texture is BVH_BEAM_TEXTURE_FACTOR* smaller than the screen.
// At depth t each beam ray represents a footprint of roughly:
//
//   dx = t * (view_size.x * 2) / W_beam
//   dy = t * (view_size.y * 2) / H_beam
//
// An AABB whose cross-section is smaller than that footprint can fall entirely
// between four beam samples and be missed. We prevent this by expanding every
// AABB outward by the half-diagonal of one beam cell at that depth:
//
//   expand(t) = t * expand_k
//   expand_k  = 0.5 * length(angle_step)    <- constant for the whole dispatch
//
// Using the distance to the AABB centre as the depth proxy is conservative
// (slightly over-expands), which is intentional — false positives are harmless,
// false negatives would break the optimisation.
//
// -- Traversal ----------------------------------------------------------------
//
// Plain stack with (idx, dist) entries. Far child pushed before near child
// so the near child is popped first (nearest-first approximation). Each pop
// prunes the entry if its dist >= the current best, so the stack drains fast
// once a hit tightens min_dist. No iterator wrapper needed since we only
// care about the single nearest distance, not a sequence of hits.

// -- Bindings -----------------------------------------------------------------

@group(2) @binding(0) var bvh_beam_textured: texture_storage_2d<r32float, write>;

struct CameraUniform {
    camera_transform: mat4x4<f32>,
    camera_view_size: vec2<f32>,
}
@group(0) @binding(0) var<uniform> camera: CameraUniform;

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
}

@group(1) @binding(0) var<storage, read> bvh:       array<BVHNode>;
@group(1) @binding(1) var<storage, read> bvh_items: array<BVHItem>;

// -- Ray–AABB intersection -----------------------------------------------------
//
// Returns vec2(t_enter, t_exit). t_enter = 1e38 means miss.

fn ray_aabb(rp: vec3<f32>, inv: vec3<f32>, mn: vec3<f32>, mx: vec3<f32>) -> vec2<f32> {
    let t1   = (mn - rp) * inv;
    let t2   = (mx - rp) * inv;
    let tmin = max(max(min(t1.x, t2.x), min(t1.y, t2.y)), min(t1.z, t2.z));
    let tmax = min(min(max(t1.x, t2.x), max(t1.y, t2.y)), max(t1.z, t2.z));
    if tmax < 0.0 || tmin > tmax { return vec2<f32>(1e38, 0.0); }
    return vec2<f32>(max(tmin, 0.0), tmax);
}

const STACK_DEPTH: i32 = 32;

struct StackEntry { idx: u32, dist: f32 }

// -- Main ----------------------------------------------------------------------

@compute
@workgroup_size(4, 4)
fn main(@builtin(global_invocation_id) global_invocation_id: vec3<u32>) {
    let texture_size = textureDimensions(bvh_beam_textured);
    let pixel = global_invocation_id.xy;
    if pixel.x >= texture_size.x || pixel.y >= texture_size.y { return; }

    // Pixel centre in normalised [0, 1] screen space.
    let screen_pos = (vec2<f32>(pixel) + vec2<f32>(0.5)) / vec2<f32>(texture_size);

    // World-space ray.
    let ray_pos = camera.camera_transform[3].xyz;
    let ray_dir = normalize((camera.camera_transform * vec4<f32>(
        (-screen_pos.x + 0.5) * camera.camera_view_size.x * 2.0,
        ( screen_pos.y - 0.5) * camera.camera_view_size.y * 2.0,
        1.0,
        0.0,
    )).xyz);
    let inv_dir = vec3<f32>(1.0) / ray_dir;

    // Precompute the expansion coefficient for this dispatch.
    // angle_step = angular size of one beam pixel on each axis.
    // expand_k   = half-diagonal of one beam cell = maximum angular offset from
    //              a beam ray to any corner of the footprint it represents.
    let angle_step = vec2<f32>(
        camera.camera_view_size.x * 2.0 / f32(texture_size.x),
        camera.camera_view_size.y * 2.0 / f32(texture_size.y),
    );
    let expand_k = 0.5 * length(angle_step);

    // -- BVH traversal --------------------------------------------------------

    var stack: array<StackEntry, 32>;
    var top: i32 = 0;
    var min_dist: f32 = 1e38;

    // Seed from sentinel (index 0). The sentinel's c0 holds the world AABB;
    // c0_min_and_ref.w bitcasts to the true root index.
    let sentinel = bvh[0];
    let root_ref = bitcast<u32>(sentinel.c0_min_and_ref.w);
    if root_ref != 0u {
        // No expansion on the world AABB — it already contains everything.
        let d = ray_aabb(ray_pos, inv_dir,
                         sentinel.c0_min_and_ref.xyz,
                         sentinel.c0_max_and_ref2.xyz);
        if d.x < 1e38 {
            stack[0] = StackEntry(root_ref, d.x);
            top = 1;
        }
    }

    while top > 0 {
        top -= 1;
        let e = stack[top];

        // Cull: this subtree can't improve our best known distance.
        if e.dist >= min_dist { continue; }

        let node  = bvh[e.idx];
        let flags = bitcast<u32>(node.c1_min_and_flags.w);
        let c0r   = bitcast<u32>(node.c0_min_and_ref.w);
        let c1r   = bitcast<u32>(node.c0_max_and_ref2.w);

        if (flags & 1u) != 0u {
            // -- Leaf: test each item with depth-proportional AABB expansion --
            //
            // We use the distance from the ray origin to the AABB centre as the
            // depth proxy. This is conservative: the centre is at most at the
            // t_exit distance, never closer than t_enter, so we may expand
            // slightly more than strictly necessary — which is the safe direction.
            let base  = c0r;
            let count = c1r;
            for (var i = 0u; i < count; i++) {
                let item = bvh_items[base + i];
                let sz   = unpack4xU8(item.aabb_size);
                let mn   = vec3<f32>(item.min_x, item.min_y, item.min_z);
                let mx   = mn + vec3<f32>(f32(sz.x), f32(sz.y), f32(sz.z));

                let center       = (mn + mx) * 0.5;
                let depth        = length(center - ray_pos);
                let expand       = depth * expand_k;
                let mn_exp       = mn - vec3<f32>(expand);
                let mx_exp       = mx + vec3<f32>(expand);

                let d = ray_aabb(ray_pos, inv_dir, mn_exp, mx_exp);
                if d.x < min_dist {
                    min_dist = d.x;
                }
            }
        } else {
            // -- Internal: expand both child AABBs, push hits nearest-first --
            //
            // Child AABBs are already embedded in the node (no extra read).
            // Expansion uses the child AABB centre as depth proxy, same logic
            // as for items above.

            let c0_mn  = node.c0_min_and_ref.xyz;
            let c0_mx  = node.c0_max_and_ref2.xyz;
            let c0_ctr = (c0_mn + c0_mx) * 0.5;
            let c0_exp = length(c0_ctr - ray_pos) * expand_k;
            let r0 = ray_aabb(ray_pos, inv_dir,
                              c0_mn - vec3<f32>(c0_exp),
                              c0_mx + vec3<f32>(c0_exp));

            let c1_mn  = node.c1_min_and_flags.xyz;
            let c1_mx  = node.c1_max_pad.xyz;
            let c1_ctr = (c1_mn + c1_mx) * 0.5;
            let c1_exp = length(c1_ctr - ray_pos) * expand_k;
            let r1 = ray_aabb(ray_pos, inv_dir,
                              c1_mn - vec3<f32>(c1_exp),
                              c1_mx + vec3<f32>(c1_exp));

            let h0 = r0.x < min_dist;
            let h1 = r1.x < min_dist;

            // Push far child first so near child sits on top (nearest-first).
            if h0 && h1 {
                if r0.x <= r1.x {
                    // c0 is nearer: push c1 (far) then c0 (near, on top)
                    if top < STACK_DEPTH { stack[top] = StackEntry(c1r, r1.x); top += 1; }
                    if top < STACK_DEPTH { stack[top] = StackEntry(c0r, r0.x); top += 1; }
                } else {
                    if top < STACK_DEPTH { stack[top] = StackEntry(c0r, r0.x); top += 1; }
                    if top < STACK_DEPTH { stack[top] = StackEntry(c1r, r1.x); top += 1; }
                }
            } else if h0 {
                if top < STACK_DEPTH { stack[top] = StackEntry(c0r, r0.x); top += 1; }
            } else if h1 {
                if top < STACK_DEPTH { stack[top] = StackEntry(c1r, r1.x); top += 1; }
            }
        }
    }

    // Write nearest hit distance. 1e38 encodes a miss; the full-res pass should
    // treat values >= 1e37 as "no beam data" and skip the early-out.
    textureStore(bvh_beam_textured, vec2<i32>(pixel), vec4<f32>(min_dist, 0.0, 0.0, 0.0));
}