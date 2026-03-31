// bvh_raycast.wgsl  (rewritten)
// Bindings (group 1):
//   @binding(0)  bvh        : array<BVHNode>
//   @binding(1)  bvh_items  : array<BVHItem>

struct BVHNode {
    min_x: f32, min_y: f32, min_z: f32,
    max_x: f32, max_y: f32, max_z: f32,
    // Interior: data_1 = left child idx,  data_2 = right child idx  (u16 each)
    // Leaf:     data_1 = bvh_items base,  data_2 = item count       (u16 each)
    data_1_and_2: u32,
    is_leaf: u32,
}

struct BVHItem {
    min_x: f32, min_y: f32, min_z: f32,
    aabb_size: u32,           // packed u8×4: size_x, size_y, size_z, _pad
    item_index: u32,
    pos_x: f32, pos_y: f32, pos_z: f32,
    quat_x: f32, quat_y: f32, quat_z: f32, quat_w: f32,
}

@group(1) @binding(0) var<storage, read> bvh:       array<BVHNode>;
@group(1) @binding(1) var<storage, read> bvh_items: array<BVHItem>;

// ── AABB slab test ─────────────────────────────────────────────────────────────
// Returns entry distance, or -1 on miss.

fn ray_aabb(rp: vec3<f32>, inv_rd: vec3<f32>, mn: vec3<f32>, mx: vec3<f32>) -> f32 {
    let t1   = (mn - rp) * inv_rd;
    let t2   = (mx - rp) * inv_rd;
    let tmin = max(max(min(t1.x, t2.x), min(t1.y, t2.y)), min(t1.z, t2.z));
    let tmax = min(min(max(t1.x, t2.x), max(t1.y, t2.y)), max(t1.z, t2.z));
    if tmax < 0.0 || tmin > tmax { return -1.0; }
    return max(tmin, 0.0);
}

// ── Stack entry packs node index + entry distance into one slot ────────────────

struct BVHStackEntry {
    node_idx: u32,
    dist:     f32,
}

const BVH_STACK_SIZE: u32 = 64u;

// ── Iterator ───────────────────────────────────────────────────────────────────
//
// Yields BVHItem indices one at a time in approximately front-to-back order.
// IMPORTANT: order is *not* strict — the caller must NOT early-exit solely on
// distance without checking all candidates within the current node's AABB range.
// Use the tighter per-item dist returned in BVHHit for actual comparisons.

struct BVHIter {
    ray_pos:  vec3<f32>,
    inv_dir:  vec3<f32>,
    // max_dist is NOT stored here — pass a (possibly tightened) budget each call.

    stk: array<BVHStackEntry, 64>,
    top: i32,   // next free slot; stack is empty when top == 0

    // Mid-leaf resume state.
    // leaf_items_base == 0xFFFFFFFF means "not inside a leaf".
    leaf_items_base:  u32,
    leaf_items_count: u32,
    leaf_item_i:      u32,
}

struct BVHHit {
    bvh_item_idx: u32,
    dist:         f32,   // entry distance into this item's AABB
    valid:        bool,
}

// ── bvh_iter_new ──────────────────────────────────────────────────────────────

fn bvh_iter_new(ray_pos: vec3<f32>, ray_dir: vec3<f32>) -> BVHIter {
    var it: BVHIter;
    it.ray_pos = ray_pos;
    it.inv_dir = vec3<f32>(1.0) / ray_dir;
    it.top     = 0;
    it.leaf_items_base  = 0xFFFFFFFFu;
    it.leaf_items_count = 0u;
    it.leaf_item_i      = 0u;

    // Seed with root node — use a large initial budget; caller filters by dist.
    let r0 = ray_aabb(ray_pos, it.inv_dir,
        vec3<f32>(bvh[0].min_x, bvh[0].min_y, bvh[0].min_z),
        vec3<f32>(bvh[0].max_x, bvh[0].max_y, bvh[0].max_z));
    if r0 >= 0.0 {
        it.stk[0].node_idx = 0u;
        it.stk[0].dist     = r0;
        it.top             = 1;
    }
    return it;
}

// ── bvh_iter_next ─────────────────────────────────────────────────────────────
//
// max_dist: current best hit distance from the *ray origin*.
//           Pass best.total_dist so already-beaten candidates are skipped.
//           The iterator never mutates this value; the caller owns it.

fn bvh_iter_next(it: ptr<function, BVHIter>, max_dist: f32) -> BVHHit {
    var miss: BVHHit;
    miss.valid        = false;
    miss.bvh_item_idx = 0u;
    miss.dist         = 0.0;

    let rp  = (*it).ray_pos;
    let inv = (*it).inv_dir;

    // ── Resume inside a leaf ───────────────────────────────────────────────
    if (*it).leaf_items_base != 0xFFFFFFFFu {
        let base  = (*it).leaf_items_base;
        let count = (*it).leaf_items_count;

        while (*it).leaf_item_i < count {
            let i     = (*it).leaf_item_i;
            (*it).leaf_item_i += 1u;

            let item = bvh_items[base + i];
            let sz   = unpack4xU8(item.aabb_size);
            let imn  = vec3<f32>(item.min_x, item.min_y, item.min_z);
            let imx  = imn + vec3<f32>(f32(sz.x), f32(sz.y), f32(sz.z));
            let d    = ray_aabb(rp, inv, imn, imx);
            if d >= 0.0 && d < max_dist {
                var hit: BVHHit;
                hit.valid        = true;
                hit.bvh_item_idx = base + i;
                hit.dist         = d;
                return hit;
            }
        }
        (*it).leaf_items_base = 0xFFFFFFFFu;   // leaf exhausted
    }

    // ── Main stack loop ────────────────────────────────────────────────────
    while (*it).top > 0 {
        (*it).top -= 1;
        let entry = (*it).stk[(*it).top];

        // Skip if this node's AABB entry is already past our best hit.
        // NOTE: entry.dist was computed at push time with the *original* budget;
        // re-checking against the tightened max_dist prunes more aggressively.
        if entry.dist >= max_dist { continue; }

        let nd = bvh[entry.node_idx];
        let d1 = nd.data_1_and_2 & 0xFFFFu;
        let d2 = (nd.data_1_and_2 >> 16u) & 0xFFFFu;

        if nd.is_leaf != 0u {
            let base  = d1;
            let count = d2;

            var i: u32 = 0u;
            while i < count {
                let item = bvh_items[base + i];
                let sz   = unpack4xU8(item.aabb_size);
                let imn  = vec3<f32>(item.min_x, item.min_y, item.min_z);
                let imx  = imn + vec3<f32>(f32(sz.x), f32(sz.y), f32(sz.z));
                let d    = ray_aabb(rp, inv, imn, imx);
                i += 1u;
                if d >= 0.0 && d < max_dist {
                    // Suspend: save resume state for the rest of this leaf.
                    (*it).leaf_items_base  = base;
                    (*it).leaf_items_count = count;
                    (*it).leaf_item_i      = i;   // already advanced past hit item
                    var hit: BVHHit;
                    hit.valid        = true;
                    hit.bvh_item_idx = base + (i - 1u);
                    hit.dist         = d;
                    return hit;
                }
            }
            // Leaf fully tested with no hit — continue to next stack entry.

        } else {
            // Interior node — test both children, push hit ones far-first.
            let n1 = bvh[d1];
            let n2 = bvh[d2];
            let e1 = ray_aabb(rp, inv,
                vec3<f32>(n1.min_x, n1.min_y, n1.min_z),
                vec3<f32>(n1.max_x, n1.max_y, n1.max_z));
            let e2 = ray_aabb(rp, inv,
                vec3<f32>(n2.min_x, n2.min_y, n2.min_z),
                vec3<f32>(n2.max_x, n2.max_y, n2.max_z));
            let hit1 = e1 >= 0.0 && e1 < max_dist;
            let hit2 = e2 >= 0.0 && e2 < max_dist;

            // Guard against stack overflow before pushing.
            if hit1 && hit2 {
                // Push far child first so near child is on top.
                if (*it).top + 2 <= i32(BVH_STACK_SIZE) {
                    if e1 <= e2 {
                        (*it).stk[(*it).top].node_idx = d2;
                        (*it).stk[(*it).top].dist     = e2;
                        (*it).top += 1;
                        (*it).stk[(*it).top].node_idx = d1;
                        (*it).stk[(*it).top].dist     = e1;
                        (*it).top += 1;
                    } else {
                        (*it).stk[(*it).top].node_idx = d1;
                        (*it).stk[(*it).top].dist     = e1;
                        (*it).top += 1;
                        (*it).stk[(*it).top].node_idx = d2;
                        (*it).stk[(*it).top].dist     = e2;
                        (*it).top += 1;
                    }
                }
            } else if hit1 {
                if (*it).top < i32(BVH_STACK_SIZE) {
                    (*it).stk[(*it).top].node_idx = d1;
                    (*it).stk[(*it).top].dist     = e1;
                    (*it).top += 1;
                }
            } else if hit2 {
                if (*it).top < i32(BVH_STACK_SIZE) {
                    (*it).stk[(*it).top].node_idx = d2;
                    (*it).stk[(*it).top].dist     = e2;
                    (*it).top += 1;
                }
            }
        }
    }

    return miss;
}