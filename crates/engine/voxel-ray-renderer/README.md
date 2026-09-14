# Per-face lighting prototype

Run `cargo run --release -p voxel-app`. **Debug → Graphics → Lighting** selects:

- **Nothing:** unlit material colors; no lighting passes.
- **Direct:** shadowed sunlight, computed once per visible voxel face.
- **1-bounce indirect:** the same direct lighting plus one diffuse bounce and visible sky light.

Lighting defaults to Nothing. **GI rays per face** selects 8, 16, 32, or 64 rays, defaulting to 32. The ray-count dropdown is enabled only for indirect lighting. Both lit modes include shadows; there is no separate shadows toggle or artificial ambient floor.

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
- Capacity or bounded-hash-probe failure falls back to direct-only lighting for the whole frame. Fallback samples shadows at the same face centers, including faces reconstructed by AA, but may repeat work per pixel.

**Face lighting stats** shows the last view's mode, ray count, face counts, secondary hits, sky misses, incomplete rays, memory, and overflow flags. Direct mode needs no indirect-hit records and can use the full face-table capacity.

## Checks

```sh
cargo test --release -p voxel-ray-renderer --lib
cargo run --release -p voxel-app --example face_gi_smoke
cargo run --release -p voxel-app --example face_gi_smoke -- --overflow
cargo run --release -p voxel-app --example face_gi_smoke -- --app
```

The smoke example saves images under the system temporary directory. It checks all lighting modes and ray counts, color bleed, repeated frames, two-dimensional dispatches, AA, render-target resizing, and grid transforms. The overflow run deliberately limits visible-face capacity and checks that fallback matches Direct byte-for-byte, with and without AA. The `--app` run captures the normal application in all modes, including 64-ray GI with AA.
