#!/usr/bin/env python3
"""Runs the voxel-app flight benchmarks in a private 1920x1080 desktopctl session.

  raybench.py run LABEL [--binary PATH] [--scenarios a,b] [--settings default,full] [--repeats N]
  raybench.py ab LABEL_A BIN_A LABEL_B BIN_B [--scenarios ...] [--settings ...] [--repeats N]
  raybench.py compare LABEL_A LABEL_B
  raybench.py show LABEL
  raybench.py images LABEL_A LABEL_B
  raybench.py exec [--binary PATH] [--seconds N] -- ARGS...   (ad-hoc run, prints the log tail)

Windowed, vsync off, RTX 2070. Results go to benchmarks/results/LABEL.json, raw CSVs and
screenshots to target/bench-out/LABEL/. `ab` alternates binaries so thermal drift hits both.
"""

import argparse
import json
import os
import statistics
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
RESULTS = ROOT / "benchmarks" / "results"
RAW = ROOT / "target" / "bench-out"
DEFAULT_BINARY = ROOT / "target" / "release" / "voxel-app"
SLANG_BIN = Path.home() / ".local" / "slang" / "bin"
WIDTH, HEIGHT = 1920, 1080
RUN_TIMEOUT_SECONDS = 1500
REQUIRED_ADAPTER = "NVIDIA"

ALL_SCENARIOS = "church,planet,castle,sponza,tour"
ALL_SETTINGS = "default,full"


def ctl(*args):
	out = subprocess.run(["desktopctl", *args], check=True, capture_output=True, text=True).stdout
	return json.loads(out)


class Desktop:
	"""A private session of our own, stopped afterwards as the machine's desktop policy requires."""

	def __enter__(self):
		self.id = ctl("create", "--width", str(WIDTH), "--height", str(HEIGHT))["id"]
		return self

	def __exit__(self, *exc):
		subprocess.run(["desktopctl", "stop", self.id], capture_output=True)

	def launch(self, script):
		return ctl("launch", self.id, "--", "bash", "-c", script)["pid"]


class GpuSampler(threading.Thread):
	"""Samples board-wide VRAM, clocks, utilization, power, and temperature once a second."""

	FIELDS = "memory.used,utilization.gpu,clocks.current.graphics,power.draw,temperature.gpu,clocks_event_reasons.hw_thermal_slowdown"

	def __init__(self):
		super().__init__(daemon=True)
		self.samples = []
		self.stop_flag = threading.Event()

	def run(self):
		while not self.stop_flag.wait(1.0):
			try:
				out = subprocess.run(
					["nvidia-smi", f"--query-gpu={self.FIELDS}", "--format=csv,noheader,nounits"],
					capture_output=True, text=True, timeout=5,
				).stdout.strip()
				parts = [part.strip() for part in out.split(",")]
				self.samples.append((float(parts[0]), float(parts[1]), float(parts[2]), float(parts[3]), float(parts[4]), parts[5] == "Active"))
			except (ValueError, IndexError, subprocess.SubprocessError):
				pass

	def summary(self):
		if not self.samples:
			return {}
		columns = list(zip(*self.samples))
		return {
			"vram_peak_mib": max(columns[0]),
			"gpu_util_mean": statistics.fmean(columns[1]),
			"gpu_clock_mean_mhz": statistics.fmean(columns[2]),
			"power_mean_w": statistics.fmean(columns[3]),
			"temp_peak_c": max(columns[4]),
			"thermal_slowdown_samples": sum(columns[5]),
		}


def parse_csv(path):
	meta = {}
	rows = []
	with open(path) as handle:
		first = handle.readline().lstrip("# ").split()
		meta = dict(item.split("=", 1) for item in first)
		header = handle.readline().strip().split(",")
		for line in handle:
			values = line.rstrip("\n").split(",")
			rows.append([float(value) if value else float("nan") for value in values])
	data = np.array(rows, dtype=float) if rows else np.zeros((0, len(header)))
	return meta, {name: data[:, index] for index, name in enumerate(header)}


def low_fps(frame_ms, fraction):
	count = max(1, int(len(frame_ms) * fraction))
	return 1000.0 / float(np.mean(np.sort(frame_ms)[-count:]))


def stutter(frame_ms):
	"""Frames more than twice the local median and at least 4 ms over it."""
	if len(frame_ms) < 3:
		return 0
	half = 15
	count = 0
	for index, value in enumerate(frame_ms):
		window = frame_ms[max(0, index - half) : index + half + 1]
		local = float(np.median(window))
		if value > 2.0 * local and value > local + 4.0:
			count += 1
	return count


def nan_stat(values, func):
	values = values[~np.isnan(values)]
	return float(func(values)) if len(values) else None


def summarize(meta, columns):
	frame = columns["frame_ms"][1:]
	if not len(frame):
		return {"meta": meta, "frames": 0}
	gpu = columns["gpu_beam_ms"] + columns["gpu_ray_ms"] + columns["gpu_color_ms"]
	stutters = stutter(frame)
	return {
		"meta": meta,
		"frames": len(frame),
		"fps_avg": 1000.0 / float(np.mean(frame)),
		"frame_ms_median": float(np.median(frame)),
		"frame_ms_p95": float(np.percentile(frame, 95)),
		"frame_ms_p99": float(np.percentile(frame, 99)),
		"frame_ms_max": float(np.max(frame)),
		"fps_low_1pct": low_fps(frame, 0.01),
		"fps_low_0_1pct": low_fps(frame, 0.001),
		"jitter_ms": float(np.mean(np.abs(np.diff(frame)))) if len(frame) > 1 else 0.0,
		"stutter_frames": stutters,
		"stutter_pct": 100.0 * stutters / len(frame),
		"main_ms_median": float(np.median(columns["main_ms"])),
		"readback_ms_median": nan_stat(columns["readback_ms"], np.median),
		"extract_ms_median": nan_stat(columns["extract_ms"], np.median),
		"gpu_total_ms_median": nan_stat(gpu, np.median),
		"gpu_total_ms_p99": nan_stat(gpu, lambda values: np.percentile(values, 99)),
		"gpu_beam_ms_median": nan_stat(columns["gpu_beam_ms"], np.median),
		"gpu_ray_ms_median": nan_stat(columns["gpu_ray_ms"], np.median),
		"gpu_color_ms_median": nan_stat(columns["gpu_color_ms"], np.median),
		"rss_mb_max": float(np.max(columns["rss_kb"])) / 1024.0,
		"peak_rss_mb": float(meta.get("peak_rss_kb", 0)) / 1024.0,
		"tile_tree_mb": float(columns["tree_bytes"][-1]) / 1e6,
		"tile_voxel_mb": float(columns["voxel_bytes"][-1]) / 1e6,
		"bvh_kb": float(columns["bvh_bytes"][-1] + columns["bvh_item_bytes"][-1]) / 1e3,
		"loaded_tiles": int(columns["loaded_tiles"][-1]),
		"pending_tiles_max": int(np.max(columns["pending_tiles"])),
		"warm_seconds": float(meta.get("warm_seconds", 0)),
	}


def validate(meta, source):
	problems = []
	if REQUIRED_ADAPTER not in meta.get("adapter", ""):
		problems.append(f"adapter {meta.get('adapter')}")
	if meta.get("window") != f"{WIDTH}x{HEIGHT}":
		problems.append(f"window {meta.get('window')}")
	if meta.get("present") != "Immediate":
		problems.append(f"present mode {meta.get('present')}")
	if problems:
		raise SystemExit(f"invalid benchmark run {source}: {', '.join(problems)}")


def launch_script(binary, args, log, status, extra_env=""):
	env = (
		f"export PATH={SLANG_BIN}:$PATH WGPU_ADAPTER_NAME={REQUIRED_ADAPTER} "
		f"VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json RUST_LOG=warn,voxel_app=info {extra_env}"
	)
	return f"{env}; cd {ROOT}; {binary} {args} > {log} 2>&1; echo $? > {status}"


def wait_status(status, timeout, kill_pattern):
	start = time.monotonic()
	while not status.exists():
		if time.monotonic() - start > timeout:
			subprocess.run(["pkill", "-f", kill_pattern])
			time.sleep(2)
			return None
		time.sleep(0.5)
	return status.read_text().strip()


def exec_adhoc(binary, seconds, args, env, prefix=""):
	out = RAW / "adhoc"
	out.mkdir(parents=True, exist_ok=True)
	log, status = out / "run.log", out / "exit_status"
	status.unlink(missing_ok=True)
	with Desktop() as desktop:
		desktop.launch(launch_script(f"timeout {seconds} {prefix} {binary}", " ".join(args), log, status, env))
		code = wait_status(status, seconds + 30, str(binary))
	print(f"exit status: {code}")
	print(log.read_text()[-6000:] if log.exists() else "(no log)")


def run_once(desktop, binary, scenario, settings, out_dir):
	out_dir.mkdir(parents=True, exist_ok=True)
	for stale in out_dir.glob("*.csv"):
		stale.unlink()
	log = out_dir / "run.log"
	status = out_dir / "exit_status"
	status.unlink(missing_ok=True)
	args = f"--benchmark {scenario} --bench-settings {settings} --bench-out {out_dir}"
	sampler = GpuSampler()
	sampler.start()
	desktop.launch(launch_script(binary, args, log, status))
	code = wait_status(status, RUN_TIMEOUT_SECONDS, f"--bench-out {out_dir}")
	sampler.stop_flag.set()
	sampler.join()
	if code is None:
		raise SystemExit(f"timed out: {scenario}/{settings}; see {log}")
	if code != "0":
		tail = log.read_text()[-3000:] if log.exists() else ""
		raise SystemExit(f"{binary} exited with {code} for {scenario}/{settings}:\n{tail}")
	shots = {}
	for csv in sorted(out_dir.glob("*.csv")):
		meta, columns = parse_csv(csv)
		validate(meta, csv)
		shots[csv.stem] = {**summarize(meta, columns), **sampler.summary()}
	if not shots:
		raise SystemExit(f"no shot output for {scenario}/{settings}; see {log}")
	return shots


def result_path(label):
	return RESULTS / f"{label}.json"


def load(label):
	return json.loads(result_path(label).read_text())


def save(label, runs, binary):
	RESULTS.mkdir(parents=True, exist_ok=True)
	commit = subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=ROOT, capture_output=True, text=True).stdout.strip()
	result_path(label).write_text(json.dumps({"label": label, "commit": commit, "binary": str(binary), "runs": runs}, indent=1))


def run_matrix(jobs, scenarios, settings, repeats):
	"""jobs: (label, binary). Alternates between jobs within every repeat."""
	collected = {label: {} for label, _ in jobs}
	with Desktop() as desktop:
		for repeat in range(repeats):
			for scenario in scenarios:
				for setting in settings:
					for label, binary in jobs:
						out_dir = RAW / label / f"{scenario}.{setting}.r{repeat}"
						print(f"[{time.strftime('%H:%M:%S')}] {label} {scenario}/{setting} repeat {repeat + 1}/{repeats}", flush=True)
						shots = run_once(desktop, binary, scenario, setting, out_dir)
						for shot, stats in shots.items():
							collected[label].setdefault(f"{scenario}/{shot}/{setting}", []).append(stats)
	for label, binary in jobs:
		save(label, collected[label], binary)


def median_stats(repeats):
	merged = {}
	for key, value in repeats[0].items():
		if isinstance(value, (int, float)) and not isinstance(value, bool):
			values = [run[key] for run in repeats if run.get(key) is not None]
			merged[key] = float(np.median(values)) if values else None
		else:
			merged[key] = value
	return merged


ROW_METRICS = [
	("fps_avg", "avg fps", True),
	("frame_ms_median", "median ms", False),
	("frame_ms_p99", "p99 ms", False),
	("fps_low_1pct", "1% low fps", True),
	("fps_low_0_1pct", "0.1% low fps", True),
	("stutter_pct", "stutter %", False),
	("jitter_ms", "jitter ms", False),
	("gpu_total_ms_median", "gpu ms", False),
	("gpu_beam_ms_median", "beam ms", False),
	("gpu_ray_ms_median", "ray ms", False),
	("gpu_color_ms_median", "color ms", False),
	("main_ms_median", "main ms", False),
	("readback_ms_median", "readback ms", False),
	("extract_ms_median", "extract ms", False),
	("peak_rss_mb", "peak RSS MB", False),
	("vram_peak_mib", "VRAM MiB", False),
	("warm_seconds", "warm s", False),
]


def show(label):
	data = load(label)
	print(f"# {label} @ {data['commit']}")
	for key, repeats in sorted(data["runs"].items()):
		stats = median_stats(repeats)
		print(f"\n{key}  ({len(repeats)} repeats, {int(stats['frames'])} frames, {int(stats['loaded_tiles'])} tiles)")
		for metric, name, _ in ROW_METRICS:
			value = stats.get(metric)
			print(f"  {name:14} {value:10.3f}" if value is not None else f"  {name:14}          -")


def compare(label_a, label_b):
	a, b = load(label_a), load(label_b)
	print(f"# {label_a} ({a['commit']}) -> {label_b} ({b['commit']}); + is better")
	best = 0.0
	for key in sorted(set(a["runs"]) & set(b["runs"])):
		stats_a, stats_b = median_stats(a["runs"][key]), median_stats(b["runs"][key])
		print(f"\n{key}")
		for metric, name, higher_better in ROW_METRICS:
			value_a, value_b = stats_a.get(metric), stats_b.get(metric)
			if value_a is None or value_b is None or value_a == 0:
				continue
			change = (value_b / value_a - 1.0) * 100.0
			improvement = change if higher_better else -change
			# Spread between repeats, so noise isn't mistaken for a win.
			spread = 0.0
			if len(a["runs"][key]) > 1:
				values = [run[metric] for run in a["runs"][key] if run.get(metric) is not None]
				spread = 100.0 * (max(values) - min(values)) / abs(value_a) if values and value_a else 0.0
			best = max(best, improvement)
			print(f"  {name:14} {value_a:10.3f} -> {value_b:10.3f}  {improvement:+7.1f}%  (A spread {spread:.1f}%)")
	print(f"\nbest improvement: {best:+.1f}%")


def images(label_a, label_b):
	from PIL import Image

	worst = 0.0
	for image_a in sorted((RAW / label_a).glob("*/*.png")):
		image_b = RAW / label_b / image_a.relative_to(RAW / label_a)
		if not image_b.exists():
			print(f"missing {image_b}")
			worst = float("inf")
			continue
		pixels_a = np.asarray(Image.open(image_a).convert("RGB"), dtype=np.int16)
		pixels_b = np.asarray(Image.open(image_b).convert("RGB"), dtype=np.int16)
		diff = np.abs(pixels_a - pixels_b)
		changed = float(np.mean(np.any(diff > 8, axis=2))) * 100.0
		worst = max(worst, changed)
		print(f"{image_a.relative_to(RAW / label_a)}: {changed:.4f}% pixels differ by >8/255, max {int(diff.max())}")
	print(f"worst: {worst:.4f}%")


def main():
	parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
	commands = parser.add_subparsers(dest="command", required=True)

	def add_matrix(sub):
		sub.add_argument("--scenarios", default=ALL_SCENARIOS)
		sub.add_argument("--settings", default=ALL_SETTINGS)
		sub.add_argument("--repeats", type=int, default=1)

	run = commands.add_parser("run")
	run.add_argument("label")
	run.add_argument("--binary", default=str(DEFAULT_BINARY))
	add_matrix(run)
	ab = commands.add_parser("ab")
	for name in ("label_a", "bin_a", "label_b", "bin_b"):
		ab.add_argument(name)
	add_matrix(ab)
	cmp_parser = commands.add_parser("compare")
	cmp_parser.add_argument("label_a")
	cmp_parser.add_argument("label_b")
	show_parser = commands.add_parser("show")
	show_parser.add_argument("label")
	images_parser = commands.add_parser("images")
	images_parser.add_argument("label_a")
	images_parser.add_argument("label_b")
	exec_parser = commands.add_parser("exec")
	exec_parser.add_argument("--binary", default=str(DEFAULT_BINARY))
	exec_parser.add_argument("--seconds", type=int, default=60)
	exec_parser.add_argument("--env", default="")
	exec_parser.add_argument("--prefix", default="", help="command to wrap the binary with, e.g. perf record")
	exec_parser.add_argument("args", nargs=argparse.REMAINDER)
	args = parser.parse_args()

	if args.command == "run":
		run_matrix([(args.label, Path(args.binary).resolve())], args.scenarios.split(","), args.settings.split(","), args.repeats)
		show(args.label)
	elif args.command == "ab":
		jobs = [(args.label_a, Path(args.bin_a).resolve()), (args.label_b, Path(args.bin_b).resolve())]
		run_matrix(jobs, args.scenarios.split(","), args.settings.split(","), args.repeats)
		compare(args.label_a, args.label_b)
	elif args.command == "compare":
		compare(args.label_a, args.label_b)
	elif args.command == "show":
		show(args.label)
	elif args.command == "images":
		images(args.label_a, args.label_b)
	elif args.command == "exec":
		extra = args.args[1:] if args.args[:1] == ["--"] else args.args
		exec_adhoc(Path(args.binary).resolve(), args.seconds, extra, args.env, args.prefix)


if __name__ == "__main__":
	sys.exit(main())
