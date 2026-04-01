// bvh_raycast_strict.wgsl
// Strict front-to-back BVH traversal using a fixed-size min-heap.

// Bindings (group 1):
//   @binding(0)  bvh        : array<BVHNode>
//   @binding(1)  bvh_items  : array<BVHItem>

struct BVHNode {
    min_x: f32, min_y: f32, min_z: f32,
    max_x: f32, max_y: f32, max_z: f32,
    data_1_and_2: u32, // (u16 | u16)
    is_leaf: u32,
}

struct BVHItem {
    min_x: f32, min_y: f32, min_z: f32,
    aabb_size: u32, // packed u8×4
    item_index: u32,
    pos_x: f32, pos_y: f32, pos_z: f32,
    quat_x: f32, quat_y: f32, quat_z: f32, quat_w: f32,
}

@group(1) @binding(0) var<storage, read> bvh:       array<BVHNode>;
@group(1) @binding(1) var<storage, read> bvh_items: array<BVHItem>;

// ── Constants ────────────────────────────────────────────────────────────────

const BVH_STACK_SIZE: u32 = 64u;

// ── Stack entry (used by heap) ───────────────────────────────────────────────

struct BVHStackEntry {
    node_idx: u32,
    dist:     f32,
}

// ── Min-heap (strict ordering) ───────────────────────────────────────────────

struct BVHHeap {
    data: array<BVHStackEntry, BVH_STACK_SIZE>,
    size: u32,
}

fn heap_push(h: ptr<function, BVHHeap>, node_idx: u32, dist: f32) {
    var i = (*h).size;
    if i >= BVH_STACK_SIZE { return; }

    (*h).size += 1u;

    loop {
        if i == 0u { break; }
        let parent = (i - 1u) >> 1u;

        if (*h).data[parent].dist <= dist { break; }

        (*h).data[i] = (*h).data[parent];
        i = parent;
    }

    (*h).data[i].node_idx = node_idx;
    (*h).data[i].dist     = dist;
}

fn heap_pop(h: ptr<function, BVHHeap>) -> BVHStackEntry {
    let result = (*h).data[0];
    (*h).size -= 1u;

    let last = (*h).data[(*h).size];
    var i: u32 = 0u;

    loop {
        let left  = i * 2u + 1u;
        let right = left + 1u;

        if left >= (*h).size { break; }

        var smallest = left;
        if right < (*h).size &&
           (*h).data[right].dist < (*h).data[left].dist {
            smallest = right;
        }

        if (*h).data[smallest].dist >= last.dist { break; }

        (*h).data[i] = (*h).data[smallest];
        i = smallest;
    }

    (*h).data[i] = last;
    return result;
}

// ── AABB slab test ───────────────────────────────────────────────────────────

fn ray_aabb(rp: vec3<f32>, inv_rd: vec3<f32>, mn: vec3<f32>, mx: vec3<f32>) -> vec2<f32> {
    let t1 = (mn - rp) * inv_rd;
    let t2 = (mx - rp) * inv_rd;

    let tmin = max(max(min(t1.x, t2.x), min(t1.y, t2.y)), min(t1.z, t2.z));
    let tmax = min(min(max(t1.x, t2.x), max(t1.y, t2.y)), max(t1.z, t2.z));

    // No intersection
    if tmax < 0.0 || tmin > tmax {
        return vec2<f32>(-1.0, -1.0);
    }

    // Clamp entry to 0 to avoid "behind ray origin"
    let entry = max(tmin, 0.0);
    let exit  = tmax;

    return vec2<f32>(entry, exit); // entry and exit distances along the ray
}

// ── Iterator ─────────────────────────────────────────────────────────────────

struct BVHIter {
    ray_pos:  vec3<f32>,
    inv_dir:  vec3<f32>,

    heap: BVHHeap,

    // Leaf resume state
    leaf_items_base:  u32,
    leaf_items_count: u32,
    leaf_item_i:      u32,
}

struct BVHHit {
    bvh_item_idx:       u32,
    dist:               f32,
    aabb_internal_dist: f32, // distance from entrace to exit of the aabb
    valid:              bool,
}

// ── Init ─────────────────────────────────────────────────────────────────────

fn bvh_iter_new(ray_pos: vec3<f32>, ray_dir: vec3<f32>) -> BVHIter {
    var it: BVHIter;

    it.ray_pos = ray_pos;
    it.inv_dir = vec3<f32>(1.0) / ray_dir;

    it.heap.size = 0u;

    it.leaf_items_base  = 0xFFFFFFFFu;
    it.leaf_items_count = 0u;
    it.leaf_item_i      = 0u;

    let r0 = ray_aabb(ray_pos, it.inv_dir,
        vec3<f32>(bvh[0].min_x, bvh[0].min_y, bvh[0].min_z),
        vec3<f32>(bvh[0].max_x, bvh[0].max_y, bvh[0].max_z));

    if r0.x >= 0.0 {  // entry distance >= 0
        heap_push(&it.heap, 0u, r0.x);
    }

    return it;
}

// ── Next (STRICT ORDER) ──────────────────────────────────────────────────────

fn bvh_iter_next(it: ptr<function, BVHIter>, max_dist: f32) -> BVHHit {
    var miss: BVHHit;
    miss.valid = false;
    miss.bvh_item_idx = 0u;
    miss.dist = 0.0;
    miss.aabb_internal_dist = 0.0;

    let rp  = (*it).ray_pos;
    let inv = (*it).inv_dir;

    // ── Resume leaf ────────────────────────────────────────────────────────
    if (*it).leaf_items_base != 0xFFFFFFFFu {
        let base  = (*it).leaf_items_base;
        let count = (*it).leaf_items_count;

        while (*it).leaf_item_i < count {
            let i = (*it).leaf_item_i;
            (*it).leaf_item_i += 1u;

            let item = bvh_items[base + i];
            let sz   = unpack4xU8(item.aabb_size);
            let imn  = vec3<f32>(item.min_x, item.min_y, item.min_z);
            let size = vec3<f32>(f32(sz.x), f32(sz.y), f32(sz.z));
            let imx  = imn + size;

            let d = ray_aabb(rp, inv, imn, imx);

            if d.x >= 0.0 && d.x < max_dist {  // entry
                var hit: BVHHit;
                hit.valid              = true;
                hit.bvh_item_idx       = base + i;
                hit.dist               = d.x;               // entry
                hit.aabb_internal_dist = d.y - d.x;         // exit - entry
                return hit;
            }
        }

        (*it).leaf_items_base = 0xFFFFFFFFu;
    }

    // ── Heap traversal ─────────────────────────────────────────────────────
    while (*it).heap.size > 0u {
        let entry = heap_pop(&(*it).heap);

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
                let size = vec3<f32>(f32(sz.x), f32(sz.y), f32(sz.z));
                let imx  = imn + size;

                let d = ray_aabb(rp, inv, imn, imx);
                i += 1u;

                if d.x >= 0.0 && d.x < max_dist {
                    (*it).leaf_items_base  = base;
                    (*it).leaf_items_count = count;
                    (*it).leaf_item_i      = i;

                    var hit: BVHHit;
                    hit.valid              = true;
                    hit.bvh_item_idx       = base + (i - 1u);
                    hit.dist               = d.x;             // entry
                    hit.aabb_internal_dist = d.y - d.x;       // exit - entry
                    return hit;
                }
            }

        } else {
            let n1 = bvh[d1];
            let n2 = bvh[d2];

            let e1 = ray_aabb(rp, inv,
                vec3<f32>(n1.min_x, n1.min_y, n1.min_z),
                vec3<f32>(n1.max_x, n1.max_y, n1.max_z));

            let e2 = ray_aabb(rp, inv,
                vec3<f32>(n2.min_x, n2.min_y, n2.min_z),
                vec3<f32>(n2.max_x, n2.max_y, n2.max_z));

            if e1.x >= 0.0 && e1.x < max_dist {
                heap_push(&(*it).heap, d1, e1.x);
            }

            if e2.x >= 0.0 && e2.x < max_dist {
                heap_push(&(*it).heap, d2, e2.x);
            }
        }
    }

    return miss;
}
