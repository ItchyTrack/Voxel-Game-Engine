import package::bvh::beam_bvh_raycast::{ beam_bvh_raycast };
import package::bvh::bvh_raycast::{ BVHHit, BVHItem, BVHIter, bvh_items, bvh_iter_new, bvh_iter_next };
import package::dda::dda_raycast::{ DDAHit, dda_raycast };

fn quat_inv_rotate(q: vec4<f32>, v: vec3<f32>) -> vec3<f32> {
	let t = 2.0 * cross(q.xyz, v);
	return v - q.w * t + cross(q.xyz, t);
}

fn beam_raycast(ray_pos: vec3<f32>, ray_dir: vec3<f32>, expand_k: f32) -> f32 {
	let beam_min_dist = beam_bvh_raycast(ray_pos, vec3<f32>(1.0) / ray_dir, expand_k);
	if beam_min_dist >= 1e37 { return 1e38; }

	var best_total_dist = 1e38;
	var iter = bvh_iter_new(ray_pos + ray_dir * beam_min_dist, ray_dir);

	loop {
		let candidate = bvh_iter_next(&iter, best_total_dist);
		if !candidate.valid { break; }

		let item = bvh_items[candidate.bvh_item_idx];
		let transform_pos = vec3<f32>(item.pos_x, item.pos_y, item.pos_z);
		let transform_quat = vec4<f32>(item.quat_x, item.quat_y, item.quat_z, item.quat_w);

		let entry_world = ray_pos + ray_dir * (beam_min_dist + candidate.dist);
		let local_pos = quat_inv_rotate(transform_quat, entry_world - transform_pos) / item.scale;
		let local_dir = quat_inv_rotate(transform_quat, ray_dir);
		let remaining = min(best_total_dist - candidate.dist, candidate.aabb_internal_dist);

		let beam_radius_world = (beam_min_dist + candidate.dist) * expand_k;
		let dda_extra = max(beam_radius_world / max(item.scale, 1e-6), 1.0);
		let dda = dda_raycast(local_pos, local_dir, remaining / item.scale + dda_extra, item.item_index);

		if dda.normal != 0u {
			let total = beam_min_dist + candidate.dist + dda.dist * item.scale;
			if total < best_total_dist {
				best_total_dist = total;
			}
		}
	}

	return best_total_dist;
}
