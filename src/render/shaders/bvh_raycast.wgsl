// -- Node layout ---------------------------------------------------------------
//
// Each BVHNode is exactly 64 bytes (one cache line). All four fields are
// vec4<f32> so the struct is naturally 16-byte-aligned throughout.
//
// Integer values (child indices, flags) are stored in the .w component of
// each vec4 via bit-cast. This keeps the struct packed without padding and
// lets AABB min/max be extracted as .xyz swizzles — no per-component reads.
//
// +- internal node --------------------------------------------------------+
// │  c0_min_and_ref  .xyz = child-0 AABB min,  .w = bitcast<f32>(c0_ref)   │
// │  c0_max_and_ref2 .xyz = child-0 AABB max,  .w = bitcast<f32>(c1_ref)   │
// │  c1_min_and_flags.xyz = child-1 AABB min,  .w = bitcast<f32>(0)        │
// │  c1_max_pad      .xyz = child-1 AABB max,  .w = 0.0                    │
// +------------------------------------------------------------------------+
// +- leaf node ------------------------------------------------------------+
// │  c0_min_and_ref  .xyz = own AABB min,      .w = bitcast<f32>(base)     │
// │  c0_max_and_ref2 .xyz = own AABB max,      .w = bitcast<f32>(count)    │
// │  c1_min_and_flags       zeroed,            .w = bitcast<f32>(1) <-flag │
// │  c1_max_pad             zeroed                                         │
// +------------------------------------------------------------------------+
//
// Index 0 is a sentinel: c0 stores the world AABB, c0_ref points to the
// true root (index 1). bvh_iter_new reads only the sentinel.

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
@group(1) @binding(2) var<storage, read_write> bvh_item_hit_counts: array<atomic<u32>>;


// -- Ray–AABB intersection -----------------------------------------------------
//
// Returns vec2(t_enter, t_exit).
//   t_enter = 1e38          -> miss
//   t_enter = max(tmin, 0)  -> hit (0 when ray origin is inside the AABB)
//
// Using .xyz swizzles + component-wise min/max means this compiles to a
// handful of vector instructions with no branching on the AABB axes.

fn ray_aabb(rp: vec3<f32>, inv: vec3<f32>, mn: vec3<f32>, mx: vec3<f32>) -> vec2<f32> {
    let t1   = (mn - rp) * inv;
    let t2   = (mx - rp) * inv;
    let tmin = max(max(min(t1.x, t2.x), min(t1.y, t2.y)), min(t1.z, t2.z));
    let tmax = min(min(max(t1.x, t2.x), max(t1.y, t2.y)), max(t1.z, t2.z));
    if tmax < 0.0 || tmin > tmax { return vec2<f32>(1e38, 0.0); }
    return vec2<f32>(max(tmin, 0.0), tmax);
}

// -- BVH iterator -------------------------------------------------------------
//
// Stack-based nearest-first traversal.
//
// WHY A STACK INSTEAD OF A HEAP:
//   The previous implementation used a binary min-heap to guarantee strict
//   nearest-first ordering. On GPU, each heap push/pop is O(log n) with
//   irregular memory access patterns inside private/scratch memory, which
//   causes register pressure and potential spilling.
//
//   A plain LIFO stack with *ordered push* (far child pushed before near child
//   so the near child sits on top) achieves near-identical traversal order at
//   O(1) cost per entry. For a well-built SAH BVH the ordering is very close
//   to optimal; the rare reordering is negligible compared to the savings.
//
// WHY THE VOXEL DDA MAKES THIS ESPECIALLY GOOD:
//   The DDA is expensive, so the first hit returned to the caller will almost
//   always update max_dist to a tight bound. Combined with the ordered stack
//   all remaining candidates farther than that bound are culled immediately
//   on pop — the stack drains quickly after the first real hit.
//
// ITEM_FLAG ENCODING:
//   Items (leaf primitives) are pushed onto the same stack as nodes, using
//   bit 31 as a tag. When a leaf node is visited, its item AABBs are tested
//   and the hits are pushed. Because items are pushed after the leaf is
//   visited, they sit on top of the stack (LIFO), so they are returned before
//   any farther subtrees — preserving approximate nearest-first order.

const ITEM_FLAG:   u32 = 0x80000000u;
const STACK_DEPTH: i32 = 32;           // 32 * 8 B = 256 B private memory

struct StackEntry { idx: u32, dist: f32 }

struct BVHIter {
    ray_pos: vec3<f32>,
    inv_dir: vec3<f32>,
    stack:   array<StackEntry, 32>,
    top:     i32,
}

struct BVHHit {
    bvh_item_idx:       u32,
    dist:               f32,
    aabb_internal_dist: f32,
    valid:              bool,
}

fn bvh_push(it: ptr<function, BVHIter>, idx: u32, dist: f32) {
    if (*it).top >= STACK_DEPTH { return; }
    (*it).stack[(*it).top] = StackEntry(idx, dist);
    (*it).top += 1;
}

// -- bvh_iter_new -------------------------------------------------------------
//
// Reads only the sentinel node (index 0) for the initial world-AABB test,
// then pushes the true root (index 1) if the ray intersects.

fn bvh_iter_new(ray_pos: vec3<f32>, ray_dir: vec3<f32>) -> BVHIter {
    var it: BVHIter;
    it.ray_pos = ray_pos;
    it.inv_dir = vec3<f32>(1.0) / ray_dir;
    it.top     = 0;

    let s    = bvh[0]; // sentinel
    let d    = ray_aabb(ray_pos, it.inv_dir, s.c0_min_and_ref.xyz, s.c0_max_and_ref2.xyz);
    let ref0 = bitcast<u32>(s.c0_min_and_ref.w);
    if d.x < 1e38 && ref0 != 0u {
        bvh_push(&it, ref0, d.x);
    }
    return it;
}

// -- bvh_iter_next -------------------------------------------------------------
//
// Returns the next nearest BVH item hit, or miss (valid = false) when done.
// Pass the current max_dist (e.g. distance to the closest known hit) to enable
// aggressive culling of entries that can no longer improve the result.
//
// Calling convention: call bvh_iter_new once, then call bvh_iter_next in a
// loop, passing the tightest known max_dist each time to exploit the fact that
// each DDA result can shrink the search radius.

fn bvh_iter_next(it: ptr<function, BVHIter>, max_dist: f32) -> BVHHit {
    var miss: BVHHit;
    miss.valid = false;

    let rp  = (*it).ray_pos;
    let inv = (*it).inv_dir;

    loop {
        if (*it).top <= 0 { break; }
        (*it).top -= 1;
        let e = (*it).stack[(*it).top];

        // Cull entries that are now past the closest known hit.
        if e.dist >= max_dist { continue; }

        // -- Leaf item --------------------------------------------------------
        if (e.idx & ITEM_FLAG) != 0u {
            let ii   = e.idx ^ ITEM_FLAG;
            let item = bvh_items[ii];
            let sz   = unpack4xU8(item.aabb_size);
            let mn   = vec3<f32>(item.min_x, item.min_y, item.min_z);
            let mx   = mn + vec3<f32>(f32(sz.x), f32(sz.y), f32(sz.z));
            let d    = ray_aabb(rp, inv, mn, mx);
            // Re-test needed: max_dist may have tightened since we pushed this item.
            if d.x < max_dist {
				atomicAdd(&bvh_item_hit_counts[ii], 1u);
                return BVHHit(ii, d.x, d.y - d.x, true);
            }
            continue;
        }

        // -- BVH node ---------------------------------------------------------
        //
        // Single node read gives us everything we need:
        //   • Both children's AABBs  (.xyz swizzles on each vec4 field)
        //   • Both children's refs   (.w of c0_min and c0_max, via bitcast)
        //   • is_leaf flag           (.w of c1_min, via bitcast)
        let node  = bvh[e.idx];
        let flags = bitcast<u32>(node.c1_min_and_flags.w);
        let c0r   = bitcast<u32>(node.c0_min_and_ref.w);
        let c1r   = bitcast<u32>(node.c0_max_and_ref2.w);

        if (flags & 1u) != 0u {
            // -- Leaf: push intersecting items -----------------------------
            let base  = c0r;
            let count = c1r;
            for (var i = 0u; i < count; i++) {
                let ii   = base + i;
                let item = bvh_items[ii];
                let sz   = unpack4xU8(item.aabb_size);
                let mn   = vec3<f32>(item.min_x, item.min_y, item.min_z);
                let mx   = mn + vec3<f32>(f32(sz.x), f32(sz.y), f32(sz.z));
                let d    = ray_aabb(rp, inv, mn, mx);
                if d.x < max_dist { bvh_push(it, ITEM_FLAG | ii, d.x); }
            }
        } else {
            // -- Internal: test both children using AABBs already in node --
            //
            // No extra BVH reads needed — both AABBs came for free with the
            // parent read above. Push far child first so near child sits on
            // top of the stack (nearest-first traversal approximation).
            let r0 = ray_aabb(rp, inv, node.c0_min_and_ref.xyz,   node.c0_max_and_ref2.xyz);
            let r1 = ray_aabb(rp, inv, node.c1_min_and_flags.xyz, node.c1_max_pad.xyz);
            let h0 = r0.x < max_dist;
            let h1 = r1.x < max_dist;

            if h0 && h1 {
                // Push the farther child first; the nearer child will be
                // popped next, maintaining approximate depth order.
                if r0.x < r1.x {
                    bvh_push(it, c1r, r1.x); // far  -> pushed first
                    bvh_push(it, c0r, r0.x); // near -> on top
                } else {
                    bvh_push(it, c0r, r0.x);
                    bvh_push(it, c1r, r1.x);
                }
            } else if h0 { bvh_push(it, c0r, r0.x); }
              else if h1 { bvh_push(it, c1r, r1.x); }
        }
    }

    return miss;
}
