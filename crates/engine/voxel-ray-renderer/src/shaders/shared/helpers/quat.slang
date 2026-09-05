#pragma once

namespace Quat {

float3 rotate(float4 q, float3 v) {
	let t = 2.0 * cross(q.xyz, v);
	return v + q.w * t + cross(q.xyz, t);
}

float3 inv_rotate(float4 q, float3 v) {
	let t = 2.0 * cross(q.xyz, v);
	return v - q.w * t + cross(q.xyz, t);
}

}
