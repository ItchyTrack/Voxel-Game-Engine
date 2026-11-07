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
| next | incremental render-side tile table; work follows changes, not tile count | extract -21..-46% (swarm 7.7 -> 4.7 ms), fps +11..21% |

## Tried, not committed

- **Skip direction-feedback atomics already set** (load before `atomicOr`): within noise on every
  shot (-5..+4% GPU time). NVIDIA handles these atomics without contention cost.

- **Trusted shader modules** (`create_shader_module_trusted` with `ShaderRuntimeChecks::unchecked()`:
  no injected bounds checks or loop counters): GPU time -2..-13% on every shot (ray pass -5..-13%,
  roughly -7% on average), fps +1..+10%. Consistent, but below the bar, and it trades away bounds
  safety: a bad tree offset would read arbitrary memory instead of zeros. Needs your approval.

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
