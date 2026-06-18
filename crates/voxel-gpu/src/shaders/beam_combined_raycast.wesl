import package::bvh::beam_raycast::{ beam_bvh_iter_new, beam_bvh_iter_next };
import package::bvh::data::{ BVHItem, bvh_items };
import package::dda::beam_raycast::{ beam_dda_raycast };

fn quat_inv_rotate(q: vec4<f32>, v: vec3<f32>) -> vec3<f32> {
	let t = 2.0 * cross(q.xyz, v);
	return v - q.w * t + cross(q.xyz, t);
}

fn beam_raycast(ray_pos: vec3<f32>, ray_dir: vec3<f32>, expand_k: f32) -> f32 {
	var best_total_dist = 1e38;
	var iter = beam_bvh_iter_new(ray_pos, ray_dir, expand_k);

	loop {
		let candidate = beam_bvh_iter_next(&iter, best_total_dist);
		if !candidate.valid { break; }

		let item = bvh_items[candidate.bvh_item_idx];
		let transform_pos = vec3<f32>(item.pos_x, item.pos_y, item.pos_z);
		let transform_quat = vec4<f32>(item.quat_x, item.quat_y, item.quat_z, item.quat_w);

		let entry_world = ray_pos + ray_dir * candidate.dist;
		let local_pos = quat_inv_rotate(transform_quat, entry_world - transform_pos) / item.scale;
		let local_dir = quat_inv_rotate(transform_quat, ray_dir);
		let remaining = min(best_total_dist - candidate.dist, candidate.aabb_internal_dist);

		let dda_distance = beam_dda_raycast(
			local_pos,
			local_dir,
			remaining / item.scale,
			item.item_index,
			candidate.dist,
			expand_k,
			item.scale,
		);

		if dda_distance < 1e37 {
			let total = candidate.dist + dda_distance * item.scale;
			if total < best_total_dist {
				best_total_dist = total;
			}
		}
	}

	return best_total_dist;
}
