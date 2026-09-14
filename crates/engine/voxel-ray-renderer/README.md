# Per-face lighting prototype

Run `cargo run --release -p voxel-app`. **Debug → Graphics → Lighting** selects:

- **Nothing:** sun-normal shading with ambient fill, without shadow or GI passes.
- **Direct:** shadowed sunlight, computed once per visible voxel face, plus ambient fill.
- **1-bounce indirect:** shadowed sunlight plus one diffuse bounce and visible sky light.

Lighting defaults to Nothing. **GI rays per face** selects 8, 16, 32, or 64 rays, defaulting to 32. The ray-count dropdown is enabled only for indirect lighting. Direct and indirect modes include shadows; there is no separate shadows toggle. Nothing and Direct retain the old 0.25 ambient fill. GI uses sampled sky and bounced sunlight instead; the fill is never added to contributor radiance.

## Frame-local lighting

Each lit frame deduplicates visible faces, including those reconstructed by ray anti-aliasing. Direct mode shades that list and stops.

Indirect mode first traces cosine-weighted rays from visible face centers and discovers contributor faces. Only then does it compute direct lighting for the combined list and gather contributions back into visible faces. Repeated hits retain their sampling weight.

The hash table stores references to hit records from an earlier pass. Keys are voxel coordinates, BVH item, and face direction—not material addresses. This avoids publishing partly written keys.

Only the table and counters need clearing. Other data is overwritten before use. Buffers survive between frames, but lighting does not. Stats readback does not feed lighting or schedule GPU work. Large workloads use two-dimensional indirect dispatches.

## Limits

- Flat per-face lighting, no filtering or history. More rays reduce noise but do not add bounces or increase light strength.
- Genuine misses sample a constant sky. Contributors currently reflect sunlight only, not skylight.
- Known unavailable geometry contributes no indirect light and blocks shadow rays. Geometry absent from the render BVH cannot be detected.
- Rasterized and marching-renderer surfaces do not contribute to this lighting.
- At full face capacity, the arena uses about 104, 136, 200, or 328 MiB per view for 8, 16, 32, or 64 rays. Smaller device limits reduce face capacity. Buffers remain allocated when lighting is disabled; changing ray count rebuilds them.
- Capacity or bounded-hash-probe failure falls back to Direct with ambient fill for the whole frame. This can occur as the camera angle changes the face count. A warning appears outside the collapsed stats panel. Fallback samples shadows at the same face centers, including faces reconstructed by AA, but may repeat work per pixel.

**Face lighting stats** shows the last view's mode, ray count, face counts, secondary hits, sky misses, incomplete rays, memory, and overflow flags. Direct mode needs no indirect-hit records and can use the full face-table capacity.

## Checks

```sh
cargo test --release -p voxel-ray-renderer --lib
cargo run --release -p voxel-app --example face_gi_smoke
cargo run --release -p voxel-app --example face_gi_smoke -- --overflow
cargo run --release -p voxel-app --example face_gi_smoke -- --app
cargo run --release -p voxel-app --example face_gi_smoke -- --angles
```

The smoke example saves images under the system temporary directory. It checks sun-normal shading, ambient fill, all ray counts, color bleed, repeated frames, two-dimensional dispatches, AA, render-target resizing, and grid transforms. The overflow run deliberately limits visible-face capacity and checks that fallback matches Direct with ambient byte-for-byte, with and without AA. The `--app` run captures the normal application in all modes, including 64-ray GI with AA. The `--angles` diagnostic captures a camera sweep at 1686×948 and logs overflow flags.
