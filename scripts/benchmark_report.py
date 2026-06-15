#!/usr/bin/env python3

import csv
import json
import math
import re
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CRITERION = ROOT / "target" / "criterion"
OUT = ROOT / "target" / "bench-reports" / "criterion"
CSV = OUT / "benchmark_history.csv"
MD = OUT / "benchmark.md"
OVERVIEW = OUT / "benchmark_avg_ms.svg"
CHART_DIR = OUT / "avg-ms"
STAMP = re.compile(r"^\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2}$")


def safe_name(s: str) -> str:
	return re.sub(r"[^A-Za-z0-9_.-]+", "_", s)


def read_rows() -> list[dict[str, str]]:
	rows = []
	for path in CRITERION.glob("**/estimates.json"):
		stamp = path.parent.name
		if not STAMP.match(stamp):
			continue
		bench = "/".join(path.relative_to(CRITERION).parts[:-2])
		data = json.loads(path.read_text())
		mean_ns = float(data["mean"]["point_estimate"])
		rows.append({"timestamp": stamp, "benchmark": bench, "mean_ns": f"{mean_ns:.3f}", "mean_ms": f"{mean_ns / 1_000_000.0:.9f}"})
	return sorted(rows, key=lambda r: (r["timestamp"], r["benchmark"]))


def write_csv(rows: list[dict[str, str]]) -> None:
	OUT.mkdir(parents=True, exist_ok=True)
	with CSV.open("w", newline="") as f:
		w = csv.DictWriter(f, fieldnames=["timestamp", "benchmark", "mean_ns", "mean_ms"])
		w.writeheader()
		w.writerows(rows)


def geometric_mean(values: list[float]) -> float:
	values = [v for v in values if v > 0]
	if not values:
		return 0.0
	return math.exp(sum(math.log(v) for v in values) / len(values))


def grouped(rows: list[dict[str, str]]):
	by_timestamp = defaultdict(list)
	by_benchmark = defaultdict(list)
	for row in rows:
		by_timestamp[row["timestamp"]].append(row)
		by_benchmark[row["benchmark"]].append(row)
	return by_timestamp, by_benchmark


def write_line_chart(path: Path, title: str, points: list[tuple[str, float]], unit: str = "ms", width: int = 520, height: int = 150) -> None:
	path.parent.mkdir(parents=True, exist_ok=True)
	if not points:
		path.write_text("")
		return
	vals = [v for _, v in points]
	ymin, ymax = min(vals), max(vals)
	pad = max((ymax - ymin) * 0.15, ymax * 0.03, 0.000001)
	ymin = max(0.0, ymin - pad)
	ymax += pad
	left, right, top, bottom = 54, 10, 24, 30

	def x(i: int) -> float:
		if len(points) == 1:
			return (width + left - right) / 2
		return left + i / (len(points) - 1) * (width - left - right)

	def y(v: float) -> float:
		return height - bottom - (v - ymin) / (ymax - ymin or 1.0) * (height - top - bottom)

	poly = " ".join(f"{x(i):.1f},{y(v):.1f}" for i, (_, v) in enumerate(points))
	circles = "\n".join(f'<circle cx="{x(i):.1f}" cy="{y(v):.1f}" r="2.5" fill="#1d4ed8"/>' for i, (_, v) in enumerate(points))
	path.write_text("\n".join([
		f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
		'<rect width="100%" height="100%" rx="5" fill="white" stroke="#ddd"/>',
		f'<text x="{left}" y="16" font-size="12" font-family="sans-serif" fill="#222">{title}</text>',
		f'<text x="4" y="{top+4}" font-size="10" font-family="sans-serif" fill="#555">{ymax:.4g}</text>',
		f'<text x="4" y="{height-bottom+4}" font-size="10" font-family="sans-serif" fill="#555">{ymin:.4g}</text>',
		f'<line x1="{left}" y1="{height-bottom}" x2="{width-right}" y2="{height-bottom}" stroke="#ddd"/>',
		f'<polyline points="{poly}" fill="none" stroke="#2563eb" stroke-width="1.8"/>',
		circles,
		f'<text x="{width-right}" y="16" text-anchor="end" font-size="11" font-family="sans-serif" fill="#333">{vals[-1]:.4g} {unit}</text>',
		f'<text x="{left}" y="{height-8}" font-size="9" font-family="sans-serif" fill="#666">{points[0][0]}</text>',
		f'<text x="{width-right}" y="{height-8}" text-anchor="end" font-size="9" font-family="sans-serif" fill="#666">{points[-1][0]}</text>',
		'</svg>',
	]))


def write_reports(rows: list[dict[str, str]]) -> None:
	by_timestamp, by_benchmark = grouped(rows)
	summary = []
	for timestamp, rs in sorted(by_timestamp.items()):
		values = [float(r["mean_ms"]) for r in rs]
		summary.append((timestamp, sum(values) / len(values), geometric_mean(values), len(values)))

	write_line_chart(OVERVIEW, "average benchmark mean", [(t, avg) for t, avg, _, _ in summary])

	chart_for = {}
	CHART_DIR.mkdir(parents=True, exist_ok=True)
	for benchmark, rs in sorted(by_benchmark.items()):
		points = [(r["timestamp"], float(r["mean_ms"])) for r in sorted(rs, key=lambda r: r["timestamp"])]
		name = f"{safe_name(benchmark)}.svg"
		write_line_chart(CHART_DIR / name, benchmark, points)
		chart_for[benchmark] = f"avg-ms/{name}"

	latest = max(by_timestamp) if by_timestamp else ""
	latest_rows = sorted(by_timestamp.get(latest, []), key=lambda r: r["benchmark"])
	previous_timestamps = sorted(by_timestamp)[:-1]
	previous = previous_timestamps[-1] if previous_timestamps else ""
	previous_by_bench = {r["benchmark"]: r for r in by_timestamp.get(previous, [])}

	lines = [
		"# Criterion benchmark history",
		"",
		f"Source: `{CRITERION.relative_to(ROOT)}`  ",
		f"CSV: [`{CSV.name}`]({CSV.name})  ",
		f"Latest timestamp: `{latest}`",
		"",
		f"![Average benchmark mean]({OVERVIEW.name})",
		"",
		"## Runs",
		"",
		"| timestamp | benchmarks | arithmetic avg ms | geometric avg ms |",
		"|---|---:|---:|---:|",
	]
	for timestamp, avg, gmean, count in summary:
		lines.append(f"| `{timestamp}` | {count} | {avg:.6f} | {gmean:.6f} |")

	lines += [
		"",
		"## Latest benchmark means",
		"",
		"| benchmark | history | latest mean ms | previous mean ms | latest/previous |",
		"|---|---|---:|---:|---:|",
	]
	for r in latest_rows:
		bench = r["benchmark"]
		latest_ms = float(r["mean_ms"])
		prev = previous_by_bench.get(bench)
		if prev:
			prev_ms = float(prev["mean_ms"])
			ratio = latest_ms / prev_ms if prev_ms else 0.0
			prev_s = f"{prev_ms:.6f}"
			ratio_s = f"{ratio:.3f}x"
		else:
			prev_s = ""
			ratio_s = ""
		lines.append(f"| `{bench}` | ![]({chart_for[bench]}) | {latest_ms:.6f} | {prev_s} | {ratio_s} |")
	MD.write_text("\n".join(lines) + "\n")


def main() -> None:
	rows = read_rows()
	if not rows:
		raise SystemExit(f"No timestamp baselines found under {CRITERION}. Run scripts/benchmark.py first.")
	write_csv(rows)
	write_reports(rows)
	print(f"Wrote {MD}")
	print(f"Wrote {CSV}")


if __name__ == "__main__":
	main()
