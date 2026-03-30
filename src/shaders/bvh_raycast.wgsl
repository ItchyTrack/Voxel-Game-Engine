// bvh_raycast.wgsl
// Bindings (group 1):
//   @binding(0)  bvh        : array<BVHNode>
//   @binding(1)  bvh_items  : array<BVHItem>

struct BVHNode {
    min_x: f32, min_y: f32, min_z: f32,
    max_x: f32, max_y: f32, max_z: f32,
    data_1_and_2: u32,
    is_leaf: u32,
}

struct BVHItem {
    min_x: f32, min_y: f32, min_z: f32,
    aabb_size: u32,      // packed u8×4 via unpack4xU8
    item_index: u32,     // byte offset of this grid's tree in grid_tree_buf
    pos_x: f32, pos_y: f32, pos_z: f32,       // global grid world position
    quat_x: f32, quat_y: f32, quat_z: f32, quat_w: f32, // global grid world rotation
}

@group(1) @binding(0) var<storage, read> bvh:       array<BVHNode>;
@group(1) @binding(1) var<storage, read> bvh_items: array<BVHItem>;

// ── AABB slab test ────────────────────────────────────────────────────────────

fn ray_aabb(rp: vec3<f32>, inv_rd: vec3<f32>, mn: vec3<f32>, mx: vec3<f32>) -> f32 {
    let t1   = (mn - rp) * inv_rd;
    let t2   = (mx - rp) * inv_rd;
    let tmin = max(max(min(t1.x,t2.x), min(t1.y,t2.y)), min(t1.z,t2.z));
    let tmax = min(min(max(t1.x,t2.x), max(t1.y,t2.y)), max(t1.z,t2.z));
    if tmax < 0.0 || tmin > tmax { return -1.0; }
    return max(tmin, 0.0);
}

// ── BVH traversal — returns sorted candidate list ─────────────────────────────
//
// We collect all leaf AABB hits sorted front-to-back so combined_raycast can
// early-out as soon as bvh_dist > best precise hit so far.

const BVH_MAX_CANDIDATES: u32 = 64u;

struct BVHCandidate {
    bvh_item_idx: u32,
    dist:         f32,
}

struct BVHResult {
    candidates: array<BVHCandidate, 64>,
    count:      u32,
}

fn bvh_raycast(ray_pos: vec3<f32>, ray_dir: vec3<f32>, max_dist: f32) -> BVHResult {
    var result: BVHResult;
    result.count = 0u;

    let inv = vec3<f32>(1.0) / ray_dir;

    var stk: array<u32, 64>;
    var sd:  array<f32, 64>;
    var top: i32 = 0;

    let r0 = ray_aabb(ray_pos, inv,
        vec3<f32>(bvh[0].min_x, bvh[0].min_y, bvh[0].min_z),
        vec3<f32>(bvh[0].max_x, bvh[0].max_y, bvh[0].max_z));
    if r0 < 0.0 || r0 > max_dist { return result; }
    stk[0] = 0u; sd[0] = r0; top = 1;

    while top > 0 {
        top -= 1;
        if sd[top] > max_dist { continue; }
        let nd = bvh[stk[top]];
        let d1 = nd.data_1_and_2 & 0xFFFFu;
        let d2 = (nd.data_1_and_2 >> 16u) & 0xFFFFu;

        if nd.is_leaf != 0u {
            for (var i = 0u; i < d2; i++) {
                let it  = bvh_items[d1 + i];
                let sz  = unpack4xU8(it.aabb_size);
                let imn = vec3<f32>(it.min_x, it.min_y, it.min_z);
                let imx = imn + vec3<f32>(f32(sz.x), f32(sz.y), f32(sz.z));
                let d   = ray_aabb(ray_pos, inv, imn, imx);
                if d >= 0.0 && d <= max_dist && result.count < BVH_MAX_CANDIDATES {
                    var idx = result.count;
                    result.count += 1u;
                    result.candidates[idx] = BVHCandidate(d1 + i, d);
                    // Insertion sort: bubble new entry to its sorted position.
                    while idx > 0u && result.candidates[idx].dist < result.candidates[idx - 1u].dist {
                        let tmp                        = result.candidates[idx];
                        result.candidates[idx]         = result.candidates[idx - 1u];
                        result.candidates[idx - 1u]    = tmp;
                        idx -= 1u;
                    }
                }
            }
        } else {
            let n1 = bvh[d1]; let n2 = bvh[d2];
            let e1 = ray_aabb(ray_pos, inv,
                vec3<f32>(n1.min_x,n1.min_y,n1.min_z),
                vec3<f32>(n1.max_x,n1.max_y,n1.max_z));
            let e2 = ray_aabb(ray_pos, inv,
                vec3<f32>(n2.min_x,n2.min_y,n2.min_z),
                vec3<f32>(n2.max_x,n2.max_y,n2.max_z));
            if e1 >= 0.0 && e1 <= max_dist && e2 >= 0.0 && e2 <= max_dist {
                if e1 < e2 {
                    stk[top]=d2; sd[top]=e2; top+=1;
                    stk[top]=d1; sd[top]=e1; top+=1;
                } else {
                    stk[top]=d1; sd[top]=e1; top+=1;
                    stk[top]=d2; sd[top]=e2; top+=1;
                }
            } else if e1 >= 0.0 && e1 <= max_dist { stk[top]=d1; sd[top]=e1; top+=1; }
              else if e2 >= 0.0 && e2 <= max_dist { stk[top]=d2; sd[top]=e2; top+=1; }
        }
    }

    return result;
}
