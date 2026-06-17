import package::bvh::bvh_raycast::{ BVHItem, BVHNode, bvh, bvh_items, bvh_item_hit_counts, ray_aabb };

const STACK_DEPTH: i32 = 32;

struct StackEntry { idx: u32, dist: f32 }

fn beam_bvh_raycast(ray_pos: vec3<f32>, inv_dir: vec3<f32>, expand_k: f32) -> f32 {
	var stack: array<StackEntry, 32>;
	var top: i32 = 0;
	var min_dist: f32 = 1e38;

	let sentinel = bvh[0];
	let root_ref = bitcast<u32>(sentinel.c0_min_and_ref.w);
	if root_ref != 0u {
		let d = ray_aabb(ray_pos, inv_dir,
						 sentinel.c0_min_and_ref.xyz,
						 sentinel.c0_max_and_ref2.xyz);
		if d.x < 1e38 {
			stack[0] = StackEntry(root_ref, d.x);
			top = 1;
		}
	}

	while top > 0 {
		top -= 1;
		let e = stack[top];
		if e.dist >= min_dist { continue; }

		let node  = bvh[e.idx];
		let flags = bitcast<u32>(node.c1_min_and_flags.w);
		let c0r   = bitcast<u32>(node.c0_min_and_ref.w);
		let c1r   = bitcast<u32>(node.c0_max_and_ref2.w);

		if (flags & 1u) != 0u {
			let base  = c0r;
			let count = c1r;
			for (var i = 0u; i < count; i++) {
				let item = bvh_items[base + i];
				let sz   = unpack4xU8(item.aabb_size);
				let mn   = vec3<f32>(item.min_x, item.min_y, item.min_z);
				let mx   = mn + vec3<f32>(f32(sz.x), f32(sz.y), f32(sz.z)) * item.scale;

				let center = (mn + mx) * 0.5;
				let depth  = length(center - ray_pos);
				let expand = depth * expand_k;
				let d = ray_aabb(ray_pos, inv_dir, mn - vec3<f32>(expand), mx + vec3<f32>(expand));
				if d.x < min_dist {
					min_dist = d.x;
					atomicAdd(&bvh_item_hit_counts[base + i], 1u);
				}
			}
		} else {
			let c0_mn  = node.c0_min_and_ref.xyz;
			let c0_mx  = node.c0_max_and_ref2.xyz;
			let c0_ctr = (c0_mn + c0_mx) * 0.5;
			let c0_exp = length(c0_ctr - ray_pos) * expand_k;
			let r0 = ray_aabb(ray_pos, inv_dir,
						  c0_mn - vec3<f32>(c0_exp),
						  c0_mx + vec3<f32>(c0_exp));

			let c1_mn  = node.c1_min_and_flags.xyz;
			let c1_mx  = node.c1_max_pad.xyz;
			let c1_ctr = (c1_mn + c1_mx) * 0.5;
			let c1_exp = length(c1_ctr - ray_pos) * expand_k;
			let r1 = ray_aabb(ray_pos, inv_dir,
						  c1_mn - vec3<f32>(c1_exp),
						  c1_mx + vec3<f32>(c1_exp));

			let h0 = r0.x < min_dist;
			let h1 = r1.x < min_dist;

			if h0 && h1 {
				if r0.x <= r1.x {
					if top < STACK_DEPTH { stack[top] = StackEntry(c1r, r1.x); top += 1; }
					if top < STACK_DEPTH { stack[top] = StackEntry(c0r, r0.x); top += 1; }
				} else {
					if top < STACK_DEPTH { stack[top] = StackEntry(c0r, r0.x); top += 1; }
					if top < STACK_DEPTH { stack[top] = StackEntry(c1r, r1.x); top += 1; }
				}
			} else if h0 {
				if top < STACK_DEPTH { stack[top] = StackEntry(c0r, r0.x); top += 1; }
			} else if h1 {
				if top < STACK_DEPTH { stack[top] = StackEntry(c1r, r1.x); top += 1; }
			}
		}
	}

	return min_dist;
}
