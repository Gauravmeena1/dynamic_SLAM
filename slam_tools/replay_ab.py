#!/usr/bin/env python3
"""Replay a captured run through the LIVE server (/add_frame), the same path the
gateway uses -- so it exercises server_api.py:148 -> FusionMaSlam.
Usage: replay_ab.py <recording> <out_name> [--limit N] [--fps F]
  <recording> is a folder name under $KACHAKA_TOOLS_DIR/replay/ (rgb/ + depth/, *_rgb.png / *_depth.png in mm)
  --fps F  post frames at F per second, like a live robot (default: as fast as possible)
  output -> $KACHAKA_TOOLS_DIR/outputs_malong/<out_name>  (mask_viz/ = removed pixels)
  The SLAM server must be running (slam_tools/run_t1_server.sh --mode rgb+depth+intr --backend ma).
  K below is the mapping D435 at 640x480 (848x480 centre-cropped); change it for another camera."""
import sys, time, json, glob, os
import urllib.request


def _load_repo_env():
    """Read the repo's .env (KEY=VALUE lines) without overriding variables already set."""
    p = os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", ".env")
    if os.path.isfile(p):
        for line in open(p):
            line = line.strip()
            if line and not line.startswith("#") and "=" in line:
                k, v = line.split("=", 1)
                os.environ.setdefault(k.strip(), os.path.expandvars(v.strip().strip("'\"")))


_load_repo_env()

if len(sys.argv) < 3 or sys.argv[1] in ("-h", "--help"):
    print(__doc__); sys.exit(0 if len(sys.argv) > 1 else 2)
TOOLS = os.environ.get("KACHAKA_TOOLS_DIR", "/opt/kachaka/fungi")

API = "http://localhost:3636"
K = [[612.6372680664062, 0.0, 312.4703369140625],
     [0.0, 613.3155517578125, 235.73280334472656],
     [0.0, 0.0, 1.0]]

def post(path, payload=None, timeout=1800):
    data = json.dumps(payload or {}).encode()
    r = urllib.request.Request(API + path, data=data,
                               headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(r, timeout=timeout) as f:
        return json.loads(f.read().decode())

def get(path, timeout=30):
    with urllib.request.urlopen(API + path, timeout=timeout) as f:
        return json.loads(f.read().decode())

src, out_name = sys.argv[1], sys.argv[2]
limit = int(sys.argv[sys.argv.index("--limit")+1]) if "--limit" in sys.argv else None
fps = float(sys.argv[sys.argv.index("--fps")+1]) if "--fps" in sys.argv else None
HOST = f"{TOOLS}/replay/{src}"
CT   = f"/fungi/replay/{src}"                     # same tree as seen inside the container
rgb = sorted(glob.glob(f"{HOST}/rgb/*_rgb.png"))
if limit: rgb = rgb[:limit]
print(f"replaying {len(rgb)} frames from {src} -> {out_name}")

print(post("/start_session", {"output_uri": f"file:///fungi/outputs_malong/{out_name}",
                              "save_on_stop": True, "intrinsics": K, "depth_scale": 1000.0}))
print(post("/set_intrinsics", {"K": K, "depth_scale": 1000.0}))

t0 = time.time(); kept = 0
for i, f in enumerate(rgb):
    stem = os.path.basename(f)[:-8]                       # strip _rgb.png
    r = post("/add_frame", {"image_path": f"{CT}/rgb/{stem}_rgb.png",
                            "depth_path": f"{CT}/depth/{stem}_depth.png"})
    kept += 1 if r.get("kept") else 0
    if fps: time.sleep(1.0 / fps)
    if (i+1) % 100 == 0:
        st = get("/status")
        print(f"  {i+1}/{len(rgb)}  kept={kept}  queue={st['queue_size']}  "
              f"splats={st['num_splats']}  {time.time()-t0:.0f}s")
print(f"all frames posted in {time.time()-t0:.0f}s; draining + finalizing ...")
t1 = time.time()
print(post("/stop", {}, timeout=3600))
print(f"finalize took {time.time()-t1:.0f}s; TOTAL {time.time()-t0:.0f}s")
print(json.dumps(get("/status"), indent=1))
