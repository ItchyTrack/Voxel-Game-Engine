struct BVHNode {
    min_x: f32, min_y: f32, min_z: f32,
    max_x: f32, max_y: f32, max_z: f32,
    data_1_and_2:       u32, // lo16 = left/base, hi16 = right/count
    is_leaf_and_parent: u32, // bit0 = is_leaf, hi16 = parent
}

struct BVHItem {
    min_x: f32, min_y: f32, min_z: f32,
    aabb_size: u32,
    item_index: u32,
    item_index_2: u32,
    pos_x: f32, pos_y: f32, pos_z: f32,
    quat_x: f32, quat_y: f32, quat_z: f32, quat_w: f32,
}

@group(1) @binding(0) var<storage, read> bvh:       array<BVHNode>;
@group(1) @binding(1) var<storage, read> bvh_items: array<BVHItem>;

const BVH_STACK_SIZE: u32 = 32u;
const ITEM_FLAG: u32 = 0x80000000u;

struct BVHStackEntry {
    node_idx: u32,
    dist:     f32,
}

struct BVHHeap {
    data: array<BVHStackEntry, BVH_STACK_SIZE>,
    size: u32,
}

fn heap_push(iter: ptr<function, BVHIter>, node_idx: u32, dist: f32) {
    let h = &(*iter).heap;
    if (*h).size >= BVH_STACK_SIZE { return; }

    var i = (*h).size;
    (*h).size += 1u;

    loop {
        if i == 0u { break; }
        let parent = (i - 1u) >> 1u;

        if (*h).data[parent].dist <= dist { break; }

        (*h).data[i] = (*h).data[parent];
        i = parent;
    }

    (*h).data[i] = BVHStackEntry(node_idx, dist);
}

fn heap_pop(iter: ptr<function, BVHIter>) -> BVHStackEntry {
    let h = &(*iter).heap;
    let root = (*h).data[0];
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
    return root;
}

fn ray_aabb(rp: vec3<f32>, inv_rd: vec3<f32>, mn: vec3<f32>, mx: vec3<f32>) -> vec2<f32> {
    let t1 = (mn - rp) * inv_rd;
    let t2 = (mx - rp) * inv_rd;

    let tmin = max(max(min(t1.x, t2.x), min(t1.y, t2.y)), min(t1.z, t2.z));
    let tmax = min(min(max(t1.x, t2.x), max(t1.y, t2.y)), max(t1.z, t2.z));

    if tmax < 0.0 || tmin > tmax {
        return vec2<f32>(1e38, 1e38);
    }

    return vec2<f32>(max(tmin, 0.0), tmax);
}

// Iterator

struct BVHIter {
    ray_pos: vec3<f32>,
    inv_dir: vec3<f32>,
    heap: BVHHeap,
    best_dist: f32,
}

struct BVHHit {
    bvh_item_idx:       u32,
    dist:               f32,
    aabb_internal_dist: f32,
    valid:              bool,
}

// Init (matches top behavior)
fn bvh_iter_new(ray_pos: vec3<f32>, ray_dir: vec3<f32>) -> BVHIter {
    var it: BVHIter;

    it.ray_pos   = ray_pos;
    it.inv_dir   = vec3<f32>(1.0) / ray_dir;
    it.heap.size = 0u;
    it.best_dist = 1e38;

    let root = bvh[0];
    let r0 = ray_aabb(ray_pos, it.inv_dir,
        vec3<f32>(root.min_x, root.min_y, root.min_z),
        vec3<f32>(root.max_x, root.max_y, root.max_z));

    if r0.x < 1e38 {
        heap_push(&it, 0u, r0.x);
    }

    return it;
}

fn bvh_iter_next(it: ptr<function, BVHIter>, max_dist: f32) -> BVHHit {
    var miss: BVHHit;
    miss.valid = false;

    let rp  = (*it).ray_pos;
    let inv = (*it).inv_dir;

    loop {
        if (*it).heap.size == 0u { break; }

        let entry = heap_pop(it);

        let limit = max_dist;

        if entry.dist >= limit {
            continue;
        }

        // ITEM
        if (entry.node_idx & ITEM_FLAG) != 0u {
            let item_idx = entry.node_idx & 0x7FFFFFFFu;
            let item     = bvh_items[item_idx];

            let sz  = unpack4xU8(item.aabb_size);
            let mn  = vec3<f32>(item.min_x, item.min_y, item.min_z);
            let mx  = mn + vec3<f32>(f32(sz.x), f32(sz.y), f32(sz.z));

            let d = ray_aabb(rp, inv, mn, mx);

            if d.x < limit {
                var hit: BVHHit;
                hit.valid              = true;
                hit.bvh_item_idx       = item_idx;
                hit.dist               = d.x;
                hit.aabb_internal_dist = d.y - d.x;
                return hit;
            }
            continue;
        }

        // NODE
        let node = bvh[entry.node_idx];
        let d1 = node.data_1_and_2 & 0xFFFFu;
        let d2 = (node.data_1_and_2 >> 16u) & 0xFFFFu;

        if (node.is_leaf_and_parent & 1u) != 0u {
            // LEAF
            let base  = d1;
            let count = d2;

            for (var i = 0u; i < count; i++) {
                let idx  = base + i;
                let item = bvh_items[idx];

                let sz = unpack4xU8(item.aabb_size);
                let mn = vec3<f32>(item.min_x, item.min_y, item.min_z);
                let mx = mn + vec3<f32>(f32(sz.x), f32(sz.y), f32(sz.z));

                let d = ray_aabb(rp, inv, mn, mx);

                if d.x < limit {
                    heap_push(it, ITEM_FLAG | idx, d.x);
                }
            }

        } else {
            // INTERNAL
            let c1 = d1;
            let c2 = d2;

            let r1 = ray_aabb(rp, inv,
                vec3<f32>(bvh[c1].min_x, bvh[c1].min_y, bvh[c1].min_z),
                vec3<f32>(bvh[c1].max_x, bvh[c1].max_y, bvh[c1].max_z));

            let r2 = ray_aabb(rp, inv,
                vec3<f32>(bvh[c2].min_x, bvh[c2].min_y, bvh[c2].min_z),
                vec3<f32>(bvh[c2].max_x, bvh[c2].max_y, bvh[c2].max_z));

            if r1.x < limit { heap_push(it, c1, r1.x); }
            if r2.x < limit { heap_push(it, c2, r2.x); }
        }
    }

    return miss;
}
