#!/bin/bash
# T3 — main control panel (see ~/fungi/README_VER2.md §2). Keys: 1 = start,
# 2 = stop (safe to press any time, does a full finalize+deploy), q = quit
# (⚠ q does NOT stop the run, press 2 first).
#   ./run_t3_main.sh             # default folder: replay a folder (main_system_with_streamer.py)
#   ./run_t3_main.sh folder --ros-args -p root_dir:=/fungi/test_run_EDB1/rgb \
#                          -p output_uri:=file:///fungi/outputs/my_test -p publish_rate:=1.0
#   ./run_t3_main.sh robot       # live robot control (main_system.py); pair with run_t2_gateway.sh robot
#
# Copied from ~/fungi/run_t3_main.sh — controls the gateway container ($KACHAKA_SLAM_CONTAINER in .env).
MODE="${1:-folder}"
[ $# -gt 0 ] && shift
case "$MODE" in
    robot)  NODE=main_system.py ;;
    folder) NODE=main_system_with_streamer.py ;;
    *) echo "usage: $0 [folder|robot] [--ros-args -p key:=value ...]"; exit 1 ;;
esac
echo "[T3] scenario=$MODE -> $NODE"
. "$(dirname "$(readlink -f "$0")")/_env.sh"
docker start "$GW_C" >/dev/null 2>&1
exec docker exec -it "$GW_C" bash -c \
    "source /opt/ros/humble/setup.bash && source /gateway_slam_ws/install/setup.bash \
     && cd /fungi/gateway_slam/src && python3 $NODE $*"
