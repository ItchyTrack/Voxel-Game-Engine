struct BVHNode {
    min_x: f32, min_y: f32, min_z: f32,
    max_x: f32, max_y: f32, max_z: f32,
    data_1_and_2:       u32, // lo16 = sub left/start,  hi16 = sub right/count
    is_leaf_and_parent: u32, // bit0 = is_leaf,     hi16 = parent idx
}

struct BVHItem {
    min_x: f32, min_y: f32, min_z: f32,
    aabb_size:  u32,
    item_index: u32,
    pos_x: f32, pos_y: f32, pos_z: f32,
    quat_x: f32, quat_y: f32, quat_z: f32, quat_w: f32,
}

@group(1) @binding(0) var<storage, read> bvh:       array<BVHNode>;
@group(1) @binding(1) var<storage, read> bvh_items: array<BVHItem>;

// ── Ray / AABB ────────────────────────────────────────────────────────────

fn ray_aabb(ray_pos: vec3<f32>, inv_rd: vec3<f32>, min_point: vec3<f32>, max_point: vec3<f32>) -> vec2<f32> {
    let t1   = (min_point - ray_pos) * inv_rd;
    let t2   = (max_point - ray_pos) * inv_rd;
    let tmin = max(max(min(t1.x, t2.x), min(t1.y, t2.y)), min(t1.z, t2.z));
    let tmax = min(min(max(t1.x, t2.x), max(t1.y, t2.y)), max(t1.z, t2.z));
    if tmax < 0.0 || tmin > tmax { return vec2<f32>(-1.0, -1.0); }
    return vec2<f32>(max(tmin, 0.0), tmax);
}

// ── Stackless BVH iterator ────────────────────────────────────────────────

const NO_PARENT: u32 = 0xFFFFu;

// The iterator holds only O(1) state: which node we are at, which node we
// came from, and where we are inside the current leaf's item list.
struct BVHIter {
    ray_pos:    vec3<f32>,
    inv_dir:    vec3<f32>,
    current:    u32,  // node we are currently inside
    state_stack: u32, // 0 -> none, 1 is left->right, 2 is right->left, 3 is done
	state_stack_index: u32,
    leaf_base:  u32,  // bvh_items base index of the active leaf
    leaf_count: u32,  // number of items in the active leaf
    leaf_i:     u32,  // next item to test within the leaf
}

struct BVHHit {
    bvh_item_idx:       u32,
    dist:               f32,
    aabb_internal_dist: f32,
    valid:              bool,
}

// Walk the parent-pointer tree until we land on the next leaf the ray may
// intersect, setting (*it).leaf_* on success.
fn bvh_advance(it: ptr<function, BVHIter>, max_dist: f32) {
    let ray_pos  = (*it).ray_pos;
    let inv_dir = (*it).inv_dir;

    loop {
		let state_stack_index = (*it).state_stack_index;
		let state = ((*it).state_stack >> state_stack_index) & 3;
		if (state == 3) {
			if (state_stack_index == 0) {
				(*it).state_stack_index = 0xFFFFFFFFu;
				return;
			}
			(*it).state_stack &= ~(3u << state_stack_index);
            (*it).state_stack_index = state_stack_index - 2u;
			let node = bvh[(*it).current];
            (*it).current = (node.is_leaf_and_parent >> 16u) & 0xFFFFu;
			continue;
		}
		let node = bvh[(*it).current];
		let is_leaf = (node.is_leaf_and_parent & 1u) != 0u;
		if (is_leaf) {
			if (state_stack_index == 0) {
				(*it).state_stack_index = 0xFFFFFFFFu;
				return;
			}
			(*it).state_stack &= ~(3u << state_stack_index);
            (*it).state_stack_index = state_stack_index - 2u;
            (*it).current = (node.is_leaf_and_parent >> 16u) & 0xFFFFu;
			continue;
		}

        let data1 =  node.data_1_and_2         & 0xFFFFu;
        let data2 = (node.data_1_and_2 >> 16u) & 0xFFFFu;

        // ── Internal node ─────────────────────────────────────────────────
		// check both nodes
		if (state == 0) {
			let node1 = bvh[data1];
			let hit1 = ray_aabb(ray_pos, inv_dir,
				vec3<f32>(node1.min_x, node1.min_y, node1.min_z),
				vec3<f32>(node1.max_x, node1.max_y, node1.max_z));
			let node2 = bvh[data2];
			let hit2 = ray_aabb(ray_pos, inv_dir,
				vec3<f32>(node2.min_x, node2.min_y, node2.min_z),
				vec3<f32>(node2.max_x, node2.max_y, node2.max_z));
			if ((hit1.x < hit2.x || hit2.x < 0.0) && hit1.x >= 0.0) {
				if (hit1.x > max_dist) {
					if (state_stack_index == 0) {
						(*it).state_stack_index = 0xFFFFFFFFu;
						return;
					}
					(*it).state_stack &= ~(3u << state_stack_index);
					(*it).state_stack_index = state_stack_index - 2u;
					(*it).current = (node.is_leaf_and_parent >> 16u) & 0xFFFFu;
					continue;
				}
				(*it).state_stack |= (1u + 2u * u32(hit2.x > max_dist)) << state_stack_index;
				(*it).state_stack_index = state_stack_index + 2u;
				(*it).current = data1;
				if (node1.is_leaf_and_parent & 1u) != 0u {
					(*it).leaf_base  = node1.data_1_and_2 & 0xFFFFu;
					(*it).leaf_count = (node1.data_1_and_2 >> 16u) & 0xFFFFu;
					(*it).leaf_i     = 0u;
					return;
				}
				continue;
			} else {
				if (hit2.x > max_dist || hit2.x < 0.0) {
					if (state_stack_index == 0) {
						(*it).state_stack_index = 0xFFFFFFFFu;
						return;
					}
					(*it).state_stack &= ~(3u << state_stack_index);
					(*it).state_stack_index = state_stack_index - 2u;
					(*it).current = (node.is_leaf_and_parent >> 16u) & 0xFFFFu;
					continue;
				}
				(*it).state_stack |= (2u + u32(hit1.x > max_dist)) << state_stack_index;
				(*it).state_stack_index = state_stack_index + 2u;
				(*it).current = data2;
				if (node2.is_leaf_and_parent & 1u) != 0u {
					(*it).leaf_base  = node2.data_1_and_2 & 0xFFFFu;
					(*it).leaf_count = (node2.data_1_and_2 >> 16u) & 0xFFFFu;
					(*it).leaf_i     = 0u;
					return;
				}
				continue;
			}
		}
		if (state == 1) {
			let node2 = bvh[data2];
			(*it).state_stack |= 3u << state_stack_index;
			(*it).state_stack_index = state_stack_index + 2;
			(*it).current = data2;
			if (node2.is_leaf_and_parent & 1u) != 0u {
				(*it).leaf_base  = node2.data_1_and_2 & 0xFFFFu;
				(*it).leaf_count = (node2.data_1_and_2 >> 16u) & 0xFFFFu;
				(*it).leaf_i = 0u;
				return;
			}
			continue;
		}
		if (state == 2) {
			let node1 = bvh[data1];
			(*it).state_stack |= 3u << state_stack_index;
			(*it).state_stack_index = state_stack_index + 2u;
			(*it).current = data1;
			if (node1.is_leaf_and_parent & 1u) != 0u {
				(*it).leaf_base  = node1.data_1_and_2 & 0xFFFFu;
				(*it).leaf_count = (node1.data_1_and_2 >> 16u) & 0xFFFFu;
				(*it).leaf_i = 0u;
				return;
			}
			continue;
		}
    }
}

fn bvh_iter_new(ray_pos: vec3<f32>, ray_dir: vec3<f32>) -> BVHIter {
    var it: BVHIter;
    it.ray_pos    = ray_pos;
    it.inv_dir    = vec3<f32>(1.0) / ray_dir;
    it.current    = 0u;
    it.state_stack = 0;
	it.state_stack_index = 0;
    it.leaf_base  = 0u;
    it.leaf_count = 0u;
    it.leaf_i     = 0u;

    let node = bvh[0];
    let r0 = ray_aabb(ray_pos, it.inv_dir,
        vec3<f32>(node.min_x, node.min_y, node.min_z),
        vec3<f32>(node.max_x, node.max_y, node.max_z));

    if r0.x < 0.0 {
       	it.state_stack_index = 0xFFFFFFFFu;
    } else if (node.is_leaf_and_parent & 1u) != 0u {
        // Root is a leaf — set up item iteration immediately.
        it.leaf_base  = node.data_1_and_2 & 0xFFFFu;
        it.leaf_count = (node.data_1_and_2 >> 16u) & 0xFFFFu;
    }

    return it;
}

fn bvh_iter_next(it: ptr<function, BVHIter>, max_dist: f32) -> BVHHit {
    var miss: BVHHit;
    miss.valid              = false;
    miss.bvh_item_idx       = 0u;
    miss.dist               = 0.0;
    miss.aabb_internal_dist = 0.0;

    if (*it).state_stack_index == 0xFFFFFFFFu { return miss; }

    let ray_pos  = (*it).ray_pos;
    let inv = (*it).inv_dir;

    loop {
        // ── Test items in the current leaf ────────────────────────────────
        while (*it).leaf_i < (*it).leaf_count {
            let i = (*it).leaf_i;
            (*it).leaf_i += 1u;

            let item = bvh_items[(*it).leaf_base + i];
            let sz   = unpack4xU8(item.aabb_size);
            let imn  = vec3<f32>(item.min_x, item.min_y, item.min_z);
            let imx  = imn + vec3<f32>(f32(sz.x), f32(sz.y), f32(sz.z));
            let d    = ray_aabb(ray_pos, inv, imn, imx);

            if d.x >= 0.0 && d.x < max_dist {
                var hit: BVHHit;
                hit.valid              = true;
                hit.bvh_item_idx       = (*it).leaf_base + i;
                hit.dist               = d.x;
                hit.aabb_internal_dist = d.y - d.x;
                return hit;
            }
        }

        // ── Leaf exhausted — walk to the next one ─────────────────────────
        bvh_advance(it, max_dist);
        if (*it).state_stack_index == 0xFFFFFFFFu { break; }
    }

    return miss;
}
