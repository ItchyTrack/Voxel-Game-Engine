# Frame-local face GI prototype

Run `cargo run --release -p voxel-app`. In **Debug → Graphics**, enable **shadows** and **face GI (prototype)**. GI is off by default; the shadows setting also controls contributor shadow rays.

Each frame:

1. Deduplicate visible voxel faces, including faces reconstructed by ray anti-aliasing.
2. Trace eight fixed, cosine-weighted rays from each face center.
3. Discover and deduplicate their contributor faces.
4. Compute direct lighting once per unique face.
5. Gather contributor lighting back into visible faces.

The hash table stores references to hit records from an earlier pass. Keys are voxel coordinates, BVH item, and face direction—not material addresses. This avoids publishing partly written keys. Repeated indirect hits retain their sampling weight.

Only the table and counters need clearing. Other data is overwritten before use. Buffers survive between frames, but lighting does not. Stats readback does not feed lighting or schedule GPU work.

## Limits

- One diffuse bounce, flat per-face lighting, no temporal or spatial filtering. Eight rays produce visible, stable noise.
- Genuine misses sample a constant sky. Known unavailable geometry contributes no indirect light and blocks direct shadow rays. Geometry absent from the render BVH cannot be detected.
- GI does not include rasterized or marching-renderer surfaces.
- The arena is capped at 104 MiB per view, with a smaller capacity on limited devices. It remains allocated while GI is disabled.
- Capacity or bounded-hash-probe failure falls back to the legacy direct/ambient renderer for the whole frame. No partial GI is displayed.

**Face GI stats** shows visible and total face counts, secondary hits, sky misses, incomplete rays, memory, and overflow flags. It reports the last rendered view.

## Checks

```sh
cargo test --release -p voxel-ray-renderer --lib
cargo run --release -p voxel-app --example face_gi_smoke
cargo run --release -p voxel-app --example face_gi_smoke -- --overflow
cargo run --release -p voxel-app --example face_gi_smoke -- --app
```

The smoke example uses a small colored room and saves images under the system temporary directory. It checks color bleed, repeated frames, GI on/off, anti-aliasing, render-target resizing, and grid translation/rotation/scale. The overflow run limits dispatch capacity and checks that fallback matches legacy rendering byte-for-byte. The `--app` run captures the normal application with GI on, with AA, and with GI off.
