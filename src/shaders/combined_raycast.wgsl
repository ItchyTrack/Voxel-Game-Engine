// combined_raycast.wgsl
// Depends on: bvh_raycast.wgsl, dda_raycast.wgsl (included before this).
//
// Each BVHItem carries the grid's global world-space pose (pos + quat).
// The ray transform into local voxel space is a single pose inversion:
//   local = q_inv * (world - pos)

// ── Quaternion helpers ────────────────────────────────────────────────────────

fn quat_inv_rotate(q: vec4<f32>, v: vec3<f32>) -> vec3<f32> {
    let t = 2.0 * cross(q.xyz, v);
    return v - q.w * t + cross(q.xyz, t);
}

fn quat_rotate(q: vec4<f32>, v: vec3<f32>) -> vec3<f32> {
    let t = 2.0 * cross(q.xyz, v);
    return v + q.w * t + cross(q.xyz, t);
}

// ── Full raycast result ───────────────────────────────────────────────────────

struct RaycastHit {
    hit:          bool,
    bvh_item_idx: u32,
    voxel_value:  u32,
    world_normal: vec3<f32>,
    total_dist:   f32,
}

// ── Combined BVH + DDA raycast ────────────────────────────────────────────────
fn full_raycast(ray_pos: vec3<f32>, ray_dir: vec3<f32>, max_dist: f32) -> RaycastHit {
    var best: RaycastHit;
    best.hit        = false;
    best.total_dist = max_dist;

	var iter = bvh_iter_new(ray_pos, ray_dir);

    loop {
		let candidate = bvh_iter_next(&iter, best.total_dist);
		if (!candidate.valid) { break; }
        let bvh_dist = candidate.dist;

        // Early-out: candidates are sorted front-to-back
        if bvh_dist >= best.total_dist { break; }

        let item = bvh_items[candidate.bvh_item_idx];

        let pose_pos  = vec3<f32>(item.pos_x,  item.pos_y,  item.pos_z);
        let pose_quat = vec4<f32>(item.quat_x, item.quat_y, item.quat_z, item.quat_w);

        // ── KEY FIX: start ray at BVH entry point ─────────────────────────────
        let entry_world = ray_pos + ray_dir * bvh_dist;

        // Transform into local voxel space
        let local_pos = quat_inv_rotate(pose_quat, entry_world - pose_pos);
        let local_dir = quat_inv_rotate(pose_quat, ray_dir);

        // Remaining distance budget after reaching BVH hit
        let remaining = min(best.total_dist - bvh_dist, candidate.aabb_internal_dist);

        let dda = dda_raycast(local_pos, local_dir, remaining, item.item_index);

        if dda.hit {
            let total = bvh_dist + dda.dist;

            if total < best.total_dist {
                best.hit          = true;
                best.bvh_item_idx = candidate.bvh_item_idx;
                best.voxel_value  = dda.value;
                best.total_dist   = total;

                // Transform normal back to world space
                best.world_normal = quat_rotate(pose_quat, dda.normal);
            }
        }
    }

    return best;
}