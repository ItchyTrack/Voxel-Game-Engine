#!/usr/bin/env python3

import argparse
import datetime as dt
from pathlib import Path
import subprocess
import sys

BENCHES = ["grid_tree", "splat_voxels"]
SUITES = {
	"all": [("grid_tree", None), ("splat_voxels", None)],
	"grid_tree": [("grid_tree", None)],
	"region_transfer": [("grid_tree", "grid_tree/region_transfer")],
	"splat_voxels": [("splat_voxels", None)],
	"splat_blocking": [("splat_voxels", "grid/splat_voxels_blocking")],
}


def run(cmd):
	print("$", " ".join(cmd), flush=True)
	subprocess.run(cmd, check=True)


parser = argparse.ArgumentParser()
parser.add_argument("--bench", action="append", choices=BENCHES, help="Criterion bench target to run")
parser.add_argument("--suite", action="append", choices=SUITES, help="Named benchmark suite/filter to run")
parser.add_argument("--filter", help="Criterion filter; overrides suite filters when provided")
parser.add_argument("--baseline")
args = parser.parse_args()

baseline = args.baseline or dt.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
if args.bench:
	bench_runs = [(bench, args.filter) for bench in args.bench]
else:
	bench_runs = []
	for suite in args.suite or ["all"]:
		bench_runs.extend(SUITES[suite])
	if args.filter:
		bench_runs = [(bench, args.filter) for bench, _ in bench_runs]

for bench, criterion_filter in bench_runs:
	cmd = ["cargo", "bench", "-p", "voxel-data", "--bench", bench, "--", "--save-baseline", baseline]
	if criterion_filter:
		cmd.append(criterion_filter)
	run(cmd)

report = Path(__file__).with_name("benchmark_report.py")
if report.exists():
	run([sys.executable, str(report)])

print(f"\nSaved Criterion baseline: {baseline}")
print("Open Criterion report: target/criterion/report/index.html")
print("Open summary doc: target/bench-reports/criterion/benchmark.md")
