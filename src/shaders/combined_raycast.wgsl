// Quaternion

fn quat_inv_rotate(q: vec4<f32>, v: vec3<f32>) -> vec3<f32> {
    let t = 2.0 * cross(q.xyz, v);
    return v - q.w * t + cross(q.xyz, t);
}

fn quat_rotate(q: vec4<f32>, v: vec3<f32>) -> vec3<f32> {
    let t = 2.0 * cross(q.xyz, v);
    return v + q.w * t + cross(q.xyz, t);
}

struct RaycastHit {
    hit:          bool,
    bvh_item_idx: u32,
    voxel_data_index: u32,
    voxel_index:   u32,
    world_normal: vec3<f32>,
    total_dist:   f32,
}

fn full_raycast(ray_pos: vec3<f32>, ray_dir: vec3<f32>, max_dist: f32) -> RaycastHit {
    var best: RaycastHit;
    best.hit = false;
    best.total_dist = max_dist;

    var iter = bvh_iter_new(ray_pos, ray_dir);

    loop {
        let candidate = bvh_iter_next(&iter, best.total_dist);
        if !candidate.valid { break; }

        let item      = bvh_items[candidate.bvh_item_idx];
        let pose_pos  = vec3<f32>(item.pos_x,  item.pos_y,  item.pos_z);
        let pose_quat = vec4<f32>(item.quat_x, item.quat_y, item.quat_z, item.quat_w);

        let entry_world = ray_pos + ray_dir * candidate.dist;
        let local_pos   = quat_inv_rotate(pose_quat, entry_world - pose_pos);
        let local_dir   = quat_inv_rotate(pose_quat, ray_dir);
        let remaining   = min(best.total_dist - candidate.dist, candidate.aabb_internal_dist);
        let dda         = dda_raycast(local_pos, local_dir, remaining, item.item_index);

        if dda.hit {
            let total = candidate.dist + dda.dist;
            if total < best.total_dist {
                best.hit          = true;
                best.bvh_item_idx = candidate.bvh_item_idx;
                best.voxel_data_index   = dda.voxel_data_index;
                best.voxel_index   = dda.voxel_index;
                best.total_dist   = total;
                best.world_normal = quat_rotate(pose_quat, dda.normal);
            }
        }
    }

    return best;
}
