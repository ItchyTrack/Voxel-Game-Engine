#define FACE_GI_COMPUTE
#include "face_gi_common.slang"
#include "combined_raycast.slang"
#include "occlusion_raycast.slang"
#include "voxel_reader.slang"

struct ViewUniform {
	float4x4 clip_from_world;
	float4x4 unjittered_clip_from_world;
	float4x4 world_from_clip;
	float4x4 world_from_view;
	float4x4 view_from_world;
	float4x4 clip_from_view;
	float4x4 view_from_clip;
	float3 world_position;
	float exposure;
};
ConstantBuffer<ViewUniform> view : register(b0, space0);

struct RenderSettings {
	uint4 values;
};
ConstantBuffer<RenderSettings> render_settings : register(b1, space0);

bool gi_active() {
	return render_settings.values.z != 0u && gi_load(2u) == 0u;
}

uint gi_insert_face(uint reference, uint2 key) {
	let mask = gi_params.sizes.x - 1u;
	let start = gi_face_hash(key);
	for (uint probe = 0u; probe < GI_PROBE_LIMIT; probe++) {
		let slot = (start + probe) & mask;
		let address = gi_params.offsets.x + slot;
		// Slang takes (comparison, replacement) and returns the old value.
		let owner = gi_words[address].compareExchange(0u, reference + 1u);
		if (owner == 0u) {
			// WGSL lowers this to a weak CAS. A unique reference confirms success.
			if (gi_load(address) != reference + 1u) {
				gi_words[2u].or(8u);
				return GI_UNKNOWN;
			}
			let index = gi_words[0u].add(1u);
			if (index >= gi_params.sizes.x) {
				gi_words[2u].or(4u);
				return GI_UNKNOWN;
			}
			gi_words[gi_params.offsets.y + index].store(slot);
			return slot;
		}
		if (all(gi_reference_face(owner - 1u).xy == key)) { return slot; }
	}
	gi_words[2u].or(1u);
	return GI_UNKNOWN;
}

void gi_store_rgb(uint offset, float3 value) {
	gi_words[offset].store(asuint(value.x));
	gi_words[offset + 1u].store(asuint(value.y));
	gi_words[offset + 2u].store(asuint(value.z));
}

void gi_dispatch_args(uint offset, uint count) {
	gi_words[offset].store((count + 63u) / 64u);
	gi_words[offset + 1u].store(1u);
	gi_words[offset + 2u].store(1u);
}

[shader("compute")]
[numthreads(8, 4, 1)]
void discover_visible(uint3 id : SV_DispatchThreadID) {
	if (!gi_active()) { return; }
	let aa = render_settings.values.y != 0u;
	let size = gi_params.sizes.zw;
	let used_size = aa ? size : size - uint2(1u);
	if (any(id.xy >= used_size)) { return; }
	let data = intermediate_textured_read.Load(int3(int2(id.xy), 0));
	if (data.x == 0u) { return; }
	let reference_base = (id.y * size.x + id.x) * 3u;
	let hit_axis = (data.x >> 3u) & 3u;
	let occluded = (data.x >> 11u) & 7u;
	for (uint axis = 0u; axis < 3u; axis++) {
		if (aa ? (occluded & (1u << axis)) != 0u : axis != hit_axis) { continue; }
		gi_insert_face(reference_base + axis, uint2(gi_canonical_normal(data.x, axis), data.y));
	}
}

[shader("compute")]
[numthreads(1, 1, 1)]
void prepare_visible(uint3 id : SV_DispatchThreadID) {
	let count = gi_load(0u);
	let visible = min(count, gi_params.sizes.y);
	gi_words[1u].store(visible);
	if (count > gi_params.sizes.y) { gi_words[2u].or(2u); }
	let work = gi_active() ? visible : 0u;
	gi_dispatch_args(4u, work);
	gi_dispatch_args(7u, work * GI_SAMPLE_COUNT);
}

float3 gi_sample_direction(uint packed_normal, uint sample_index, float4 rotation) {
	let axis = (packed_normal >> 3u) & 3u;
	let negative = (packed_normal & (1u << axis)) != 0u;
	// Rotate a fixed cosine-weighted pattern using only the local face identity.
	let phase = float(gi_hash(packed_normal) & 0x00ffffffu) / 16777216.0;
	let reversed = ((sample_index & 1u) << 2u) | (sample_index & 2u) | ((sample_index & 4u) >> 2u);
	let u = (float(sample_index) + 0.5) / float(GI_SAMPLE_COUNT);
	let angle = 6.28318530718 * frac(float(reversed) / float(GI_SAMPLE_COUNT) + phase);
	float3 local_dir = float3(0.0);
	local_dir[axis] = (negative ? -1.0 : 1.0) * sqrt(1.0 - u);
	local_dir[(axis + 1u) % 3u] = sqrt(u) * cos(angle);
	local_dir[(axis + 2u) % 3u] = sqrt(u) * sin(angle);
	return normalize(Quat::rotate(rotation, local_dir));
}

[shader("compute")]
[numthreads(64, 1, 1)]
void trace_indirect(uint3 id : SV_DispatchThreadID) {
	if (!gi_active() || id.x >= gi_load(1u) * GI_SAMPLE_COUNT) { return; }
	let face = gi_list_face(id.x / GI_SAMPLE_COUNT);
	let geometry = gi_face_geometry(face);
	let direction = gi_sample_direction(face.x, id.x % GI_SAMPLE_COUNT, geometry.rotation);
	let hit = full_raycast(geometry.center + geometry.normal * geometry.bias, direction, 1e38);
	uint3 record = uint3(0u, GI_UNKNOWN, 0u);
	if (hit.unknown_dist < hit.total_dist) {
		gi_words[3u].add(1u);
	} else if (hit.normal == 0u) {
		record.y = GI_SKY;
		gi_words[14u].add(1u);
	} else {
		record = uint3(gi_canonical_normal(hit.normal, (hit.normal >> 3u) & 3u), hit.bvh_item_idx,
			hit.voxel_data_index | (hit.voxel_index << 16u));
		gi_words[13u].add(1u);
	}
	let offset = gi_params.offsets.z + id.x * 4u;
	gi_words[offset].store(record.x);
	gi_words[offset + 1u].store(record.y);
	gi_words[offset + 2u].store(record.z);
}

[shader("compute")]
[numthreads(64, 1, 1)]
void discover_contributors(uint3 id : SV_DispatchThreadID) {
	if (!gi_active() || id.x >= gi_load(1u) * GI_SAMPLE_COUNT) { return; }
	let offset = gi_params.offsets.z + id.x * 4u;
	let normal = gi_load(offset);
	let item = gi_load(offset + 1u);
	uint slot = item;
	if (normal != 0u) {
		slot = gi_insert_face(gi_primary_reference_count() + id.x, uint2(normal, item));
	}
	gi_words[offset + 3u].store(slot);
}

[shader("compute")]
[numthreads(1, 1, 1)]
void prepare_all(uint3 id : SV_DispatchThreadID) {
	gi_dispatch_args(10u, gi_active() ? min(gi_load(0u), gi_params.sizes.x) : 0u);
}

[shader("compute")]
[numthreads(64, 1, 1)]
void light_direct(uint3 id : SV_DispatchThreadID) {
	if (!gi_active() || id.x >= min(gi_load(0u), gi_params.sizes.x)) { return; }
	let slot = gi_load(gi_params.offsets.y + id.x);
	let face = gi_reference_face(gi_load(gi_params.offsets.x + slot) - 1u);
	let geometry = gi_face_geometry(face);
	let sun_dir = normalize(float3(0.5, 1.0, 0.2));
	float direct = 0.75 * max(dot(geometry.normal, sun_dir), 0.0);
	if (direct > 0.0 && render_settings.values.x != 0u) {
		if (occlusion_raycast_conservative(geometry.center + geometry.normal * geometry.bias, sun_dir, 1e38)) {
			direct = 0.0;
		}
	}
	let item = bvh_items[face.y];
	let albedo = voxel_sampler_color(item.voxel_type_id, item.aabb_size >> 24u, item.item_index_2,
		face.z & 0xffffu, face.z >> 16u, geometry.surface_pos).xyz;
	let offset = gi_params.offsets.w + slot * 7u;
	gi_store_rgb(offset, albedo * direct);
	gi_words[offset + 3u].store(asuint(direct));
	gi_store_rgb(offset + 4u, float3(0.0));
}

[shader("compute")]
[numthreads(64, 1, 1)]
void gather_indirect(uint3 id : SV_DispatchThreadID) {
	if (!gi_active() || id.x >= gi_load(1u)) { return; }
	float3 incoming = float3(0.0);
	for (uint sample = 0u; sample < GI_SAMPLE_COUNT; sample++) {
		let hit_offset = gi_params.offsets.z + (id.x * GI_SAMPLE_COUNT + sample) * 4u;
		let contributor = gi_load(hit_offset + 3u);
		if (contributor == GI_SKY) {
			incoming += float3(0.25);
		} else if (contributor < gi_params.sizes.x) {
			incoming += gi_load_rgb(gi_params.offsets.w + contributor * 7u);
		}
	}
	let slot = gi_load(gi_params.offsets.y + id.x);
	gi_store_rgb(gi_params.offsets.w + slot * 7u + 4u, incoming / float(GI_SAMPLE_COUNT));
}
