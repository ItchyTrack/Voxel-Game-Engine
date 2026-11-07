# Optimization log

Each improvement is one commit and must beat the previous commit by at least 10% on a valuable
metric in an alternating A/B (`scripts/raybench.py ab`). Ideas that miss the bar or change the image
are recorded here so they can be revisited or approved by hand.

Noise: identical-binary GPU pass times vary about ±5% between runs. Treat smaller changes as noise.

## Committed

| commit | change | headline |
|---|---|---|
| ~~`3fe5a96`~~ | reuse the ray BVH and its GPU buffers while the tile set is unchanged (removed in `04a6ad5`: only helps static scenes) | extract -20%, default fps +8..15% |
| `858a2fe` | cache tile placements, drop redundant sort, Fx hashing | extract -22..30%, default fps +4..12% |
| next | async compute (streaming) pool 60% -> 25% of cores | tour fps +69%, 1% low 1.6 -> 7.3 fps, main 32 -> 22 ms; static warm-up +25% |
| `f2b4ebe` | incremental render-side tile table; work follows changes, not tile count | extract -21..-46% (swarm 7.7 -> 4.7 ms), fps +11..21% |

## Tried, not committed

- **Skip direction-feedback atomics already set** (load before `atomicOr`): within noise on every
  shot (-5..+4% GPU time). NVIDIA handles these atomics without contention cost.

- **Trusted shader modules** (`create_shader_module_trusted` with `ShaderRuntimeChecks::unchecked()`:
  no injected bounds checks or loop counters): GPU time -2..-13% on every shot (ray pass -5..-13%,
  roughly -7% on average), fps +1..+10%. Consistent, but below the bar, and it trades away bounds
  safety: a bad tree offset would read arbitrary memory instead of zeros. Needs your approval.

- **Cheaper BVH builders** (BVH is rebuilt every frame since the cache was removed). Same-run
  comparison on church, dynamic, sponza (fps / extract ms / ray-pass ms):
  - Morton split (leaves of 4): extract -30..-50%, but ray pass up to +26% (sponza full). Fps flat or worse.
  - HLBVH hybrid (12-bit Morton clusters joined by binned SAH): extract 3.6 -> 2.6 ms in church,
    4.7 -> 3.7 in swarm, 3.1 -> 1.8 in sponza; ray pass equal on church/swarm but +30% in sponza, so
    sponza full loses 15% fps. 15-bit clusters: no better quality, slower build.
  - SAH without parallel subtree builds: extract roughly doubles (3.6 -> 6.9 ms).
  The leaf grouping, not the top levels, is what sponza is sensitive to. Not committed.

- **Async BVH build** (`benchmarks/experiments/async-bvh-build.patch`): extraction hands the item
  list to a builder thread and the render world joins it in PrepareResources. The SAH builder's
  parallel halves move from the compute pool to scoped threads, because joining from a render
  system while the build needs the pool deadlocks. Extract -33..-62% everywhere, but here the render
  thread is the bottleneck (Xvfb present) and now waits 2-3 ms for the build, so fps -0..-7%
  (tour +3%, its 1% low +12%). Likely a win on a real display, where present is cheap and
  extract + main is the critical path. Needs your approval.

- **Streaming threads at nice 10** instead of a smaller pool: no effect at all, so the tour's main
  thread isn't starved of CPU; the cost scales with how fast tile results arrive.
- **mimalloc global allocator** (the main thread blocked in `free()` on memory allocated by tile
  builders): tour +10% avg fps with 30-45% run spread, other shots -5..+4%, peak RSS up to +10%.
  Not convincing.

## Parked for approval (would change the image)

(none yet)

## Environment notes

- Frame structure with pipelined rendering: frame ≈ extract + max(main, render thread). In the
  dynamic swarm (default settings) that is 7.6 + max(6.9, 19.7) ms. Extract blocks both worlds, so it
  is always on the critical path.
- The render thread's on-CPU time is ~60% the driver's present (Xvfb copy, ~9 ms per 1080p
  frame). It also runs main-world tasks (physics, parallel BVH build) while it waits, which delays
  its own work.
- Present goes through Xvfb (private desktopctl session), so the NVIDIA driver copies every frame
  to the CPU. In the stock profile that copy was ~21% of the main thread's CPU time. It counts in
  `frame_ms` but is not engine work.
- EXPERIMENTAL_RAY_QUERY makes vkCreateDevice fail on driver 550.163, which blocks hardware ray
  tracing through wgpu on this machine for now.
