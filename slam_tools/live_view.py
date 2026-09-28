#!/usr/bin/env python3
"""Live mapping preview: watch the current RealSense mapping camera feed in a
browser (MJPEG stream).

Does not touch the camera or the pipeline at all -- it purely polls the rgb/
folder of a run's output directory and turns the newest image into a
multipart MJPEG stream. Fully independent of the mapping run itself: won't
contend for the camera, won't affect mapping performance, and a remote
browser can just open a URL to watch.

Usage:
    python3 live_view.py outputs_malong/<run_name> [--port 8090] [--fps 5]

Once running, open in a browser:
    http://<Thor's LAN IP>:<port>/

Copied from ~/fungi/live_view.py, comments translated to English; behavior
unchanged. Note: the run directory this points at (outputs_malong/<run_name>)
physically lives under ~/fungi, not here -- see run_slam_oneshot.sh's header
comment for why.
"""
import argparse
import glob
import os
import socket
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

BOUNDARY = "frame"


def find_latest(rgb_dir, pattern="*_rgb.png"):
    files = glob.glob(os.path.join(rgb_dir, pattern))
    if not files:
        return None
    return max(files, key=os.path.getmtime)


def lan_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        s.close()


class Handler(BaseHTTPRequestHandler):
    rgb_dir = None
    pattern = "*_rgb.png"
    ctype = "image/png"
    fps = 5.0

    def log_message(self, fmt, *args):
        pass  # keep quiet, don't spam the terminal

    def do_GET(self):
        if self.path != "/":
            self.send_response(404)
            self.end_headers()
            return
        self.send_response(200)
        self.send_header("Age", "0")
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Pragma", "no-cache")
        self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={BOUNDARY}")
        self.end_headers()

        last_sent = None
        period = 1.0 / self.fps
        try:
            while True:
                path = find_latest(self.rgb_dir, self.pattern)
                if path and path != last_sent:
                    try:
                        with open(path, "rb") as f:
                            data = f.read()
                    except OSError:
                        # file may still be mid-write, skip this tick
                        time.sleep(period)
                        continue
                    self.wfile.write(f"--{BOUNDARY}\r\n".encode())
                    self.wfile.write(f"Content-Type: {self.ctype}\r\n".encode())
                    self.wfile.write(f"Content-Length: {len(data)}\r\n\r\n".encode())
                    self.wfile.write(data)
                    self.wfile.write(b"\r\n")
                    last_sent = path
                time.sleep(period)
        except (BrokenPipeError, ConnectionResetError):
            pass  # the viewer closed the tab, that's fine


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    ap.add_argument("run_dir", help="outputs_malong/<run_name> (the folder containing rgb/)")
    ap.add_argument("--port", type=int, default=8090)
    ap.add_argument("--fps", type=float, default=5.0,
                     help="refresh rate, default 5 (only for human viewing, doesn't need to match the mapping fps)")
    ap.add_argument("--mask", action="store_true",
                    help="stream mask_viz/ instead: [camera | pixels removed by dynamic masking, in red]. "
                         "Frames appear once per submap, after the masker has run")
    args = ap.parse_args()

    rgb_dir = os.path.join(args.run_dir, "mask_viz" if args.mask else "rgb")
    Handler.rgb_dir = rgb_dir
    if args.mask:
        Handler.pattern, Handler.ctype = "*.jpg", "image/jpeg"
    Handler.fps = args.fps

    print(f"[live_view] waiting for {rgb_dir} to appear (only exists once mapping starts collecting frames) ...")
    while not os.path.isdir(rgb_dir):
        time.sleep(1)

    ip = lan_ip()
    try:
        server = ThreadingHTTPServer(("0.0.0.0", args.port), Handler)
    except OSError as e:
        print(f"[live_view] ❌ could not listen on port {args.port}: {e}")
        print(f"[live_view]    probably a previous live_view.py wasn't cleaned up and is still holding this port, "
              f"check `pgrep -af live_view.py`, or rerun with a different --port")
        raise SystemExit(1)

    print(f"[live_view] ready, open in a browser: http://{ip}:{args.port}/")
    print("[live_view] Ctrl+C to stop")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
