import package::bvh::data::{ bvh, bvh_items };
import package::helpers::raycast::{ ray_aabb };

const BVH_RAYCAST_ITEM_FLAG:   u32 = 0x80000000u;
const BVH_RAYCAST_STACK_DEPTH: i32 = 32;

struct BVHRaycastStackEntry { idx: u32, dist: f32 }

struct BVHRaycastIter {
	ray_pos: vec3<f32>,
	inv_dir: vec3<f32>,
	stack:   array<BVHRaycastStackEntry, 32>,
	top:     i32,
}

struct BVHRaycastHit {
	bvh_item_idx:       u32,
	dist:               f32,
	aabb_internal_dist: f32,
	valid:              bool,
}

fn bvh_raycast_push(it: ptr<function, BVHRaycastIter>, idx: u32, dist: f32) {
	if (*it).top >= BVH_RAYCAST_STACK_DEPTH { return; }
	(*it).stack[(*it).top] = BVHRaycastStackEntry(idx, dist);
	(*it).top += 1;
}

fn bvh_iter_new(ray_pos: vec3<f32>, ray_dir: vec3<f32>) -> BVHRaycastIter {
	var iter: BVHRaycastIter;
	iter.ray_pos = ray_pos;
	iter.inv_dir = vec3<f32>(1.0) / ray_dir;
	iter.top     = 0;

	let sentinel = bvh[0];
	let distances = ray_aabb(ray_pos, iter.inv_dir, sentinel.c0_min_and_ref.xyz, sentinel.c0_max_and_ref2.xyz);
	let root_ref = bitcast<u32>(sentinel.c0_min_and_ref.w);
	if distances.x < 1e38 && root_ref != 0u {
		bvh_raycast_push(&iter, root_ref, distances.x);
	}
	return iter;
}

fn bvh_iter_next(iter: ptr<function, BVHRaycastIter>, max_dist: f32) -> BVHRaycastHit {
	var miss: BVHRaycastHit;
	miss.valid = false;

	let ray_pos = (*iter).ray_pos;
	let inv_dir = (*iter).inv_dir;

	loop {
		if (*iter).top <= 0 { break; }
		(*iter).top -= 1;
		let entry = (*iter).stack[(*iter).top];
		if entry.dist >= max_dist { continue; }

		if (entry.idx & BVH_RAYCAST_ITEM_FLAG) != 0u {
			let item_index = entry.idx ^ BVH_RAYCAST_ITEM_FLAG;
			let item = bvh_items[item_index];
			let size = unpack4xU8(item.aabb_size);
			let min_corner = vec3<f32>(item.min_x, item.min_y, item.min_z);
			let max_corner = min_corner + vec3<f32>(f32(size.x), f32(size.y), f32(size.z)) * item.scale;
			let distances = ray_aabb(ray_pos, inv_dir, min_corner, max_corner);
			if distances.x < max_dist {
				return BVHRaycastHit(item_index, distances.x, distances.y - distances.x, true);
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
				let distances = ray_aabb(ray_pos, inv_dir, min_corner, max_corner);
				if distances.x < max_dist { bvh_raycast_push(iter, BVH_RAYCAST_ITEM_FLAG | item_index, distances.x); }
			}
		} else {
			let child0_hit = ray_aabb(ray_pos, inv_dir, node.c0_min_and_ref.xyz,   node.c0_max_and_ref2.xyz);
			let child1_hit = ray_aabb(ray_pos, inv_dir, node.c1_min_and_flags.xyz, node.c1_max_pad.xyz);
			let child0_hit_ok = child0_hit.x < max_dist;
			let child1_hit_ok = child1_hit.x < max_dist;

			if child0_hit_ok && child1_hit_ok {
				if child0_hit.x < child1_hit.x {
					bvh_raycast_push(iter, child1_ref, child1_hit.x);
					bvh_raycast_push(iter, child0_ref, child0_hit.x);
				} else {
					bvh_raycast_push(iter, child0_ref, child0_hit.x);
					bvh_raycast_push(iter, child1_ref, child1_hit.x);
				}
			} else if child0_hit_ok { bvh_raycast_push(iter, child0_ref, child0_hit.x); }
			  else if child1_hit_ok { bvh_raycast_push(iter, child1_ref, child1_hit.x); }
		}
	}

	return miss;
}
