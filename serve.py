#!/usr/bin/env python3
"""Serve the bridge-inspection dashboard.

    python serve.py --bag ../data/run_20261024_123507 [--port 8099]

Standard-library HTTP server (same style as dashboard/server/app.py). Serves:
  /                      the dashboard UI (web/index.html)
  /web/<file>            static UI assets + the exported data bundle (web/data/)
  /frames/<NNNN.png>     inspection-frame stills from the bag (the "video feed")

It expects export_web.py to have been run first (writes web/data/). If the
bundle is missing it still serves the UI with an on-screen hint.
"""
import argparse
import json
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote, urlparse

HERE = Path(__file__).resolve().parent
WEB = HERE / "web"
REVIEWS = HERE / "out" / "reviews"      # sign-off records (gitignored with out/)

sys.path.insert(0, str(HERE))
from pipeline import review  # noqa: E402


def _result():
    p = WEB / "data" / "result.json"
    return json.loads(p.read_text(encoding="utf-8")) if p.exists() else None

TYPES = {".html": "text/html; charset=utf-8", ".js": "text/javascript; charset=utf-8",
         ".css": "text/css; charset=utf-8", ".json": "application/json",
         ".bin": "application/octet-stream", ".png": "image/png", ".ico": "image/x-icon",
         ".mp4": "video/mp4"}


def make_handler(frames_dir: Path):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *a):
            pass  # quiet

        def _send(self, code, body, ctype="application/octet-stream"):
            if isinstance(body, str):
                body = body.encode()
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            if self.command != "HEAD":
                self.wfile.write(body)

        def do_GET(self):
            path = unquote(urlparse(self.path).path)
            if path in ("/", "/index.html"):
                return self._send(200, (WEB / "index.html").read_bytes(), TYPES[".html"])
            if path.startswith("/web/"):
                f = (WEB / path[len("/web/"):]).resolve()
                if WEB in f.parents and f.is_file():
                    if f.suffix == ".mp4":
                        return self._send_range(f, TYPES[".mp4"])
                    return self._send(200, f.read_bytes(), TYPES.get(f.suffix, "application/octet-stream"))
                return self._send(404, b"not found", "text/plain")
            if path.startswith("/frames/"):
                name = path[len("/frames/"):]
                # Prefer the annotated copy (masks/boxes drawn) when it exists.
                annotated = (WEB / "data" / "annotated" / name).resolve()
                if (WEB / "data" / "annotated") in annotated.parents and annotated.is_file():
                    return self._send(200, annotated.read_bytes(), TYPES[".png"])
                f = (frames_dir / name).resolve()
                if frames_dir in f.parents and f.is_file() and f.suffix == ".png":
                    return self._send(200, f.read_bytes(), TYPES[".png"])
                return self._send(404, b"frame not found", "text/plain")
            if path.startswith("/raw/"):  # the un-annotated original frame
                f = (frames_dir / path[len("/raw/"):]).resolve()
                if frames_dir in f.parents and f.is_file() and f.suffix == ".png":
                    return self._send(200, f.read_bytes(), TYPES[".png"])
                return self._send(404, b"frame not found", "text/plain")
            if path == "/api/review":
                res = _result()
                if res is None:
                    return self._send(404, b'{"error":"no result exported"}', TYPES[".json"])
                return self._send(200, json.dumps(review.load(REVIEWS, res)), TYPES[".json"])
            if path == "/favicon.ico":
                return self._send(204, b"", TYPES[".ico"])
            return self._send(404, b"not found", "text/plain")

        def do_POST(self):
            if unquote(urlparse(self.path).path) != "/api/review":
                return self._send(404, b"not found", "text/plain")
            res = _result()
            if res is None:
                return self._send(404, b'{"error":"no result exported"}', TYPES[".json"])
            try:
                n = min(int(self.headers.get("Content-Length", "0")), 10_000)
                body = json.loads(self.rfile.read(n) or b"{}")
                rec = review.apply(REVIEWS, res, str(body.get("action", "")),
                                   reviewer=str(body.get("reviewer", "")), note=str(body.get("note", "")),
                                   expected_hash=str(body.get("report_hash", "")))
            except (ValueError, review.ReviewError) as e:
                return self._send(400, json.dumps({"error": str(e)}), TYPES[".json"])
            return self._send(200, json.dumps(rec), TYPES[".json"])

        def _send_range(self, fpath, ctype):
            """Serve a file with HTTP Range support (needed for <video> seeking)."""
            import os
            size = os.path.getsize(fpath)
            rng = self.headers.get("Range", "")
            start, end = 0, size - 1
            if rng.startswith("bytes="):
                a, _, b = rng[len("bytes="):].partition("-")
                if a:
                    start = int(a)
                if b:
                    end = int(b)
                end = min(end, size - 1)
            length = end - start + 1
            with open(fpath, "rb") as fh:
                fh.seek(start)
                data = fh.read(length)
            code = 206 if rng else 200
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(length))
            self.send_header("Accept-Ranges", "bytes")
            if rng:
                self.send_header("Content-Range", f"bytes {start}-{end}/{size}")
            self.end_headers()
            if self.command != "HEAD":
                self.wfile.write(data)

        do_HEAD = do_GET

    return Handler


def main() -> int:
    ap = argparse.ArgumentParser(description="Serve the bridge-inspection dashboard")
    ap.add_argument("--bag", required=True, help="bag dir (for the inspection_frames stills)")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8099)
    args = ap.parse_args()

    frames_dir = (Path(args.bag) / "inspection_frames").resolve()
    if not (WEB / "data" / "result.json").exists():
        print("warning: web/data/result.json missing - run export_web.py first", file=sys.stderr)

    httpd = ThreadingHTTPServer((args.host, args.port), make_handler(frames_dir))
    httpd.daemon_threads = True
    shown = "localhost" if args.host in ("0.0.0.0", "") else args.host
    print(f"Hyperion dashboard on http://{shown}:{args.port}")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
