#!/usr/bin/env bash
# install.sh — add the dynamic-masking hook to a ma-long streaming-server checkout.
#
#   slam_integration/install.sh /path/to/ma-long-server [/path/to/workspace]
#
#   /path/to/ma-long-server  host path of the SLAM server repo
#   /path/to/workspace       optional: KACHAKA_TOOLS_DIR (the folder mounted at /fungi); also patches
#                            the camera gateway there: COLLECT_EXTRA passthrough (parked recordings)
#                            and the RealSense 640x480 fallback for cameras on USB 2
#
# 1. links src/ma_slam/fusion_solver.py -> this folder's fusion_solver.py (relative link, so it also
#    resolves inside a container that mounts both repos under one common parent folder)
# 2. applies server_api.patch (the --dynamic_mask flag + masker wiring + live mask view)
#    and viz.patch (viewer layout: 3D map | camera with removed pixels)
# Safe to run twice: already-applied patches are detected and skipped.
set -euo pipefail
HERE="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
SRV="${1:?usage: install.sh /path/to/ma-long-server}"
[ -f "$SRV/src/ma_slam/solver.py" ] || { echo "$SRV does not look like a ma-long server checkout"; exit 1; }

dst="$SRV/src/ma_slam/fusion_solver.py"
rel="$(python3 -c 'import os,sys; print(os.path.relpath(sys.argv[1], os.path.dirname(sys.argv[2])))' "$HERE/fusion_solver.py" "$dst")"
if [ -e "$dst" ] && [ ! -L "$dst" ]; then
    mv "$dst" "$dst.bak.$(date +%Y%m%d%H%M%S)"; echo "kept the previous fusion_solver.py as a .bak"
fi
ln -sfn "$rel" "$dst"; echo "✓ linked $dst -> $rel"

apply() {   # apply DIR PATCHNAME — idempotent
    if patch -d "$1" -p1 --dry-run -R -s -f < "$HERE/$2.patch" >/dev/null 2>&1; then
        echo "✓ $2.patch already applied"
    elif patch -d "$1" -p1 --dry-run -s -f < "$HERE/$2.patch" >/dev/null 2>&1; then
        patch -d "$1" -p1 -s < "$HERE/$2.patch"; echo "✓ applied $2.patch"
    else
        echo "✗ $2.patch does not apply cleanly to $1 — apply it by hand"; exit 1
    fi
}
apply "$SRV" server_api
apply "$SRV" viz
if [ -n "${2:-}" ]; then
    WS="$2"
    [ -f "$WS/gateway_slam/src/gateway_node_robot.py" ] || { echo "$WS has no gateway_slam/src"; exit 1; }
    [ -f "$WS/semantic_slam/src/ma-long/src/ma_slam_stream/realsense.py" ] \
        || { echo "$WS/semantic_slam/src/ma-long/src/ma_slam_stream/realsense.py missing (the gateway's tuned capture needs it)"; exit 1; }
    apply "$WS" gateway_collect_extra
    apply "$WS" realsense_usb2_fallback
fi
echo "done. Start the server with slam_tools/run_t1_server.sh (it passes --dynamic_mask)."
