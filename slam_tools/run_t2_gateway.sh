#!/bin/bash
# T2 — Gateway (see ~/fungi/README_VER2.md §2). Scenario must be picked explicitly:
#   ./run_t2_gateway.sh           # default folder: replay a folder (gateway_node.py, no hardware)
#   ./run_t2_gateway.sh folder    # same as above
#   ./run_t2_gateway.sh robot     # ⚠ LIVE ROBOT: gateway_node_robot.py — a goal will open the
#                                  #   camera and move the robot!
# Pairing rule: folder <-> run_t3_main.sh folder; robot <-> run_t3_main.sh robot
# (mismatching gives 0 frames and can move real hardware unexpectedly)
#
# Copied from ~/fungi/run_t2_gateway.sh — controls the gateway container ($KACHAKA_SLAM_CONTAINER in .env).
MODE="${1:-folder}"
case "$MODE" in
    robot)  NODE=gateway_node_robot.py ;;
    folder) NODE=gateway_node.py ;;
    *) echo "usage: $0 [folder|robot]"; exit 1 ;;
esac
echo "[T2] scenario=$MODE -> $NODE  (wait for '=== SLAM Control Node Started ===')"
. "$(dirname "$(readlink -f "$0")")/_env.sh"
docker start "$GW_C" >/dev/null 2>&1
exec docker exec -it "$GW_C" bash -c \
    "source /opt/ros/humble/setup.bash && source /gateway_slam_ws/install/setup.bash \
     && cd /fungi/gateway_slam/src && python3 $NODE"
