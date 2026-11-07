# Flight benchmarks

`scripts/raybench.py` flies the camera through fixed scenes and records per-frame timings. The window
is real (1920x1080, vsync off), runs on the discrete GPU, and lives in a private `desktopctl`
session that the script creates and stops. Release build only:

```sh
cargo build --release
scripts/raybench.py run my-label                      # every scenario, default + full settings
scripts/raybench.py run my-label --scenarios church --settings full --repeats 3
scripts/raybench.py ab before target/bench-bin/old after target/release/voxel-app --repeats 2
scripts/raybench.py compare before after              # % change per metric, + is better
scripts/raybench.py images before after               # screenshot diff, frozen shots only
scripts/raybench.py exec --seconds 30 -- --host       # ad-hoc run, prints the log
```

`ab` alternates the two binaries inside every repeat, so thermal drift and background load hit
both. Laptop GPU clocks swing, so compare runs from the same `ab`, not results from different times.

## Scenarios

Defined in `crates/voxel-app/src/benchmark/scenarios.rs`.

| scenario | shots | what it stresses |
|---|---|---|
| church | `church_flyover`, `church_street` | wide LOD view of the village; dense near geometry with grazing rays |
| planet | `planet_orbit`, `planet_tree` | 1024 tile grids of the planet; fine foliage on the tree |
| castle | `castle_flyaround` | `res/castle.vox`, spawned at x=-6000 for this scenario only |
| sponza | `sponza_atrium` | ray-traced copy of `res/sponza.vox` at x=6000, enclosed space |
| tour | `streaming_tour` | live streaming and physics at real-time speed: stutter, load, memory |

Frozen shots (all except `tour`) warm streaming at the first camera key until it has been idle for
6 s, freeze the camera tile loader and physics, settle for 90 frames, then fly 480 frames at a fixed
1/60 s step. Every run renders the same frames, so ray tracing cost compares directly and
screenshots must match. The live tour moves by wall clock, so it measures what a player would see.

Settings: `default` is the shipped `GraphicsSettings` (shadows and ray AA off); `full` turns both on.

## Metrics

Per shot, median across repeats:

- `fps_avg`, `frame_ms_median`, `frame_ms_p95/p99/max`: wall time between frame starts.
- `fps_low_1pct`, `fps_low_0_1pct`: fps over the slowest 1% / 0.1% of frames.
- `stutter_pct`: frames over 2x the local (31-frame) median and at least 4 ms above it. `jitter_ms`: mean
  frame-to-frame change.
- `gpu_*_ms`: timestamp queries around the beam, ray, and coloring passes.
- `main_ms`: main-world schedule. `extract_ms`: voxel scene extraction. `readback_ms`: direction feedback
  readback, which blocks extraction.
- `peak_rss_mb` (VmHWM), `vram_peak_mib` (board-wide, nvidia-smi), `warm_seconds`, tile counts.

Results: `benchmarks/results/<label>.json`. Raw CSVs, logs, and screenshots: `target/bench-out/<label>/`.
