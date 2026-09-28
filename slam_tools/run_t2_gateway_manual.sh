#!/bin/bash
# T2 — Gateway manual mode: always runs the live robot gateway_node_robot.py --manual --duration 120
# ⚠ LIVE ROBOT: a goal will open the camera and move the robot! Pair with run_t3_main.sh robot.
#
# Copied from ~/fungi/run_t2_gateway_manual.sh — controls the gateway container ($KACHAKA_SLAM_CONTAINER in .env).
echo "[T2] manual robot -> gateway_node_robot.py --manual --duration 120  (wait for '=== SLAM Control Node Started ===')"
. "$(dirname "$(readlink -f "$0")")/_env.sh"
docker start "$GW_C" >/dev/null 2>&1
exec docker exec -it "$GW_C" bash -c \
    "source /opt/ros/humble/setup.bash && source /gateway_slam_ws/install/setup.bash \
     && cd /fungi/gateway_slam/src && python3 gateway_node_robot.py --manual --duration 120"
