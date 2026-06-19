#!/usr/bin/env python3
import argparse
import os
import shutil
import subprocess
import sys
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
TARGET_WASM = ROOT / "target/wasm32-unknown-unknown/release/bevy_web_worker.wasm"
PKG_DIR = ROOT / "pkg"
OUT_NAME = "bevy_web_worker"


class CoiRequestHandler(SimpleHTTPRequestHandler):
	def end_headers(self):
		self.send_header("Cross-Origin-Opener-Policy", "same-origin")
		self.send_header("Cross-Origin-Embedder-Policy", "require-corp")
		self.send_header("Cross-Origin-Resource-Policy", "cross-origin")
		super().end_headers()

	def guess_type(self, path):
		if path.endswith(".wasm"):
			return "application/wasm"
		return super().guess_type(path)


def run(command, env=None):
	print("+", " ".join(command))
	subprocess.run(command, cwd=ROOT, env=env, check=True)


def ensure_tool(name):
	if shutil.which(name) is None:
		print(f"Missing required tool: {name}", file=sys.stderr)
		sys.exit(1)


def build():
	ensure_tool("cargo")
	ensure_tool("wasm-bindgen")

	run([
		"cargo",
		"build",
		"-p",
		"bevy-web-worker",
		"--lib",
		"--target",
		"wasm32-unknown-unknown",
		"--release",
		"-Z",
		"build-std=std,panic_abort",
	])

	PKG_DIR.mkdir(exist_ok=True)
	run([
		"wasm-bindgen",
		str(TARGET_WASM),
		"--out-dir",
		str(PKG_DIR),
		"--target",
		"web",
		"--out-name",
		OUT_NAME,
	])


def serve(port):
	os.chdir(ROOT)
	server = ThreadingHTTPServer(("127.0.0.1", port), CoiRequestHandler)
	base = f"http://127.0.0.1:{port}"
	entry = f"{base}/crates/bevy-web-worker/web/index.html"
	print(f"Serving {ROOT} at {base}")
	print(f"Open {entry}")
	print("COOP/COEP enabled for threaded wasm")
	try:
		server.serve_forever()
	except KeyboardInterrupt:
		print("\nShutting down server")
	finally:
		server.server_close()


def main():
	parser = argparse.ArgumentParser(description="Build voxel-app for threaded wasm and serve it locally.")
	parser.add_argument("--port", type=int, default=8000)
	parser.add_argument("--no-build", action="store_true")
	args = parser.parse_args()

	if not args.no_build:
		build()

	serve(args.port)


if __name__ == "__main__":
	main()
