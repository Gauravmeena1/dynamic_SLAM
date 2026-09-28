#!/usr/bin/env bash
# SLAM runner for ~/kachaka_mapping/run_bridge_oneshot.sh (KACHAKA_SLAM_RUNNER in its .env).
# The bridge drives the robot itself and ends collection by pressing Enter, so
# record with no time limit (--duration 0) instead of the manual-mode default 120 s.
exec "$(dirname "$(readlink -f "$0")")/run_slam_oneshot.sh" "$@" --duration 0
