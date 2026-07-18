import package::bvh::data::{ bvh, bvh_items };
import package::helpers::raycast::{ ray_aabb };

const BEAM_BVH_RAYCAST_ITEM_FLAG:   u32 = 0x80000000u;
const BEAM_BVH_RAYCAST_STACK_DEPTH: i32 = 32;

struct BeamBVHRaycastStackEntry { idx: u32, dist: f32 }

struct BeamBVHRaycastIter {
	ray_pos: vec3<f32>,
	inv_dir: vec3<f32>,
	expand_k: f32,
	stack:   array<BeamBVHRaycastStackEntry, 32>,
	top:     i32,
}

struct BeamBVHRaycastHit {
	bvh_item_idx:       u32,
	dist:               f32,
	aabb_internal_dist: f32,
	valid:              bool,
}

fn beam_bvh_raycast_push(iter: ptr<function, BeamBVHRaycastIter>, idx: u32, dist: f32) {
	if (*iter).top >= BEAM_BVH_RAYCAST_STACK_DEPTH { return; }
	(*iter).stack[(*iter).top] = BeamBVHRaycastStackEntry(idx, dist);
	(*iter).top += 1;
}

fn beam_bvh_iter_new(ray_pos: vec3<f32>, ray_dir: vec3<f32>, expand_k: f32) -> BeamBVHRaycastIter {
	var iter: BeamBVHRaycastIter;
	iter.ray_pos = ray_pos;
	iter.inv_dir = vec3<f32>(1.0) / ray_dir;
	iter.expand_k = expand_k;
	iter.top = 0;

	let sentinel = bvh[0];
	let distances = ray_aabb(ray_pos, iter.inv_dir, sentinel.c0_min_and_ref.xyz, sentinel.c0_max_and_ref2.xyz);
	let root_ref = bitcast<u32>(sentinel.c0_min_and_ref.w);
	if distances.x < 1e38 && root_ref != 0u {
		beam_bvh_raycast_push(&iter, root_ref, distances.x);
	}
	return iter;
}

fn beam_bvh_iter_next(iter: ptr<function, BeamBVHRaycastIter>, max_dist: f32) -> BeamBVHRaycastHit {
	var miss: BeamBVHRaycastHit;
	miss.valid = false;

	let ray_pos = (*iter).ray_pos;
	let inv_dir = (*iter).inv_dir;
	let expand_k = (*iter).expand_k;

	loop {
		if (*iter).top <= 0 { break; }
		(*iter).top -= 1;
		let entry = (*iter).stack[(*iter).top];
		if entry.dist >= max_dist { continue; }

		if (entry.idx & BEAM_BVH_RAYCAST_ITEM_FLAG) != 0u {
			let item_index = entry.idx ^ BEAM_BVH_RAYCAST_ITEM_FLAG;
			let item = bvh_items[item_index];
			let size = unpack4xU8(item.aabb_size);
			let min_corner = vec3<f32>(item.min_x, item.min_y, item.min_z);
			let max_corner = min_corner + vec3<f32>(f32(size.x), f32(size.y), f32(size.z)) * item.scale;
			let center = (min_corner + max_corner) * 0.5;
			let depth = length(center - ray_pos);
			let expand = depth * expand_k;
			let distances = ray_aabb(ray_pos, inv_dir, min_corner - vec3<f32>(expand), max_corner + vec3<f32>(expand));
			if distances.x < max_dist {
				return BeamBVHRaycastHit(item_index, distances.x, distances.y - distances.x, true);
			}
			continue;
		}

		let node = bvh[entry.idx];
		let flags = bitcast<u32>(node.c1_min_and_flags.w);
		let child0_ref = bitcast<u32>(node.c0_min_and_ref.w);
		let child1_ref = bitcast<u32>(node.c0_max_and_ref2.w);

		if (flags & 1u) != 0u {
			let base = child0_ref;
			let count = child1_ref;
			for (var i = 0u; i < count; i++) {
				let item_index = base + i;
				let item = bvh_items[item_index];
				let size = unpack4xU8(item.aabb_size);
				let min_corner = vec3<f32>(item.min_x, item.min_y, item.min_z);
				let max_corner = min_corner + vec3<f32>(f32(size.x), f32(size.y), f32(size.z)) * item.scale;
				let center = (min_corner + max_corner) * 0.5;
				let depth = length(center - ray_pos);
				let expand = depth * expand_k;
				let distances = ray_aabb(ray_pos, inv_dir, min_corner - vec3<f32>(expand), max_corner + vec3<f32>(expand));
				if distances.x < max_dist { beam_bvh_raycast_push(iter, BEAM_BVH_RAYCAST_ITEM_FLAG | item_index, distances.x); }
			}
		} else {
			let child0_min = node.c0_min_and_ref.xyz;
			let child0_max = node.c0_max_and_ref2.xyz;
			let child0_center = (child0_min + child0_max) * 0.5;
			let child0_expand = length(child0_center - ray_pos) * expand_k;
			let child0_hit = ray_aabb(ray_pos, inv_dir, child0_min - vec3<f32>(child0_expand), child0_max + vec3<f32>(child0_expand));

			let child1_min = node.c1_min_and_flags.xyz;
			let child1_max = node.c1_max_pad.xyz;
			let child1_center = (child1_min + child1_max) * 0.5;
			let child1_expand = length(child1_center - ray_pos) * expand_k;
			let child1_hit = ray_aabb(ray_pos, inv_dir, child1_min - vec3<f32>(child1_expand), child1_max + vec3<f32>(child1_expand));

			let child0_hit_ok = child0_hit.x < max_dist;
			let child1_hit_ok = child1_hit.x < max_dist;

			if child0_hit_ok && child1_hit_ok {
				if child0_hit.x < child1_hit.x {
					beam_bvh_raycast_push(iter, child1_ref, child1_hit.x);
					beam_bvh_raycast_push(iter, child0_ref, child0_hit.x);
				} else {
					beam_bvh_raycast_push(iter, child0_ref, child0_hit.x);
					beam_bvh_raycast_push(iter, child1_ref, child1_hit.x);
				}
			} else if child0_hit_ok { beam_bvh_raycast_push(iter, child0_ref, child0_hit.x); }
			  else if child1_hit_ok { beam_bvh_raycast_push(iter, child1_ref, child1_hit.x); }
		}
	}

	return miss;
}

