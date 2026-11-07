# Optimization log

Each improvement is one commit and must beat the previous commit by at least 10% on a valuable
metric in an alternating A/B (`scripts/raybench.py ab`). Ideas that miss the bar or change the image
are recorded here so they can be revisited or approved by hand.

Noise: identical-binary GPU pass times vary about ±5% between runs. Treat smaller changes as noise.

## Committed

| commit | change | headline |
|---|---|---|
| `3fe5a96` | reuse the ray BVH and its GPU buffers while the tile set is unchanged | extract -20%, default fps +8..15% |
| `858a2fe` | cache tile placements, drop redundant sort, Fx hashing | extract -22..30%, default fps +4..12% |

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

- Present goes through Xvfb (private desktopctl session), so the NVIDIA driver copies every frame
  to the CPU. In the stock profile that copy was ~21% of the main thread's CPU time. It counts in
  `frame_ms` but is not engine work.
- EXPERIMENTAL_RAY_QUERY makes vkCreateDevice fail on driver 550.163, which blocks hardware ray
  tracing through wgpu on this machine for now.
