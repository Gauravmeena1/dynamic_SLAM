#!/usr/bin/env bash
# make_containers.sh — create the SLAM (AI) container and the camera-gateway container.
#
#   slam_tools/make_containers.sh
#
# Everything comes from .env (see .env.example):
#   KACHAKA_AI_CONTAINER / KACHAKA_AI_IMAGE           SLAM server + masking (GPU)
#   KACHAKA_SLAM_CONTAINER / KACHAKA_GATEWAY_IMAGE    RealSense -> ROS 2 gateway
#   KACHAKA_TOOLS_DIR      host workspace, mounted at /fungi in both
#   KACHAKA_CODE_DIR       host folder holding ma-long-server/ and this repo (dynamic_SLAM/), mounted at
#                          KACHAKA_CODE_DIR_CT in the AI container
#   KACHAKA_WEIGHTS_DIR    host ma-long tree that holds src/weights and vendor/weights (read-only)
#
# An existing container is removed first only if it is stopped (running ones are left alone).
set -euo pipefail
. "$(dirname "$(readlink -f "$0")")/_env.sh"
need() { [ -n "${!1:-}" ] || { echo "set $1 in .env"; exit 2; }; }
for v in KACHAKA_AI_IMAGE KACHAKA_GATEWAY_IMAGE KACHAKA_CODE_DIR KACHAKA_CODE_DIR_CT KACHAKA_WEIGHTS_DIR; do need $v; done
for img in "$KACHAKA_AI_IMAGE" "$KACHAKA_GATEWAY_IMAGE"; do
    docker image inspect "$img" >/dev/null || { echo "image $img not found — stop"; exit 1; }
done
[ -d "$KACHAKA_WEIGHTS_DIR/vendor/weights" ] && [ -d "$KACHAKA_WEIGHTS_DIR/src/weights" ] \
    || { echo "weights not found under $KACHAKA_WEIGHTS_DIR — stop"; exit 1; }
[ -d "$FUNGI_HOST/gateway_slam/src" ] || { echo "workspace $FUNGI_HOST has no gateway_slam/src — stop"; exit 1; }

remove_if_stopped() {
    docker inspect "$1" >/dev/null 2>&1 || return 0
    [ "$(docker inspect -f '{{.State.Running}}' "$1")" = false ] || { echo "$1 is running — stop it first"; exit 1; }
    docker rm "$1" >/dev/null
}
ENVS=(-e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e ROS_DOMAIN_ID=0 -e ROS_LOCALHOST_ONLY=0)

remove_if_stopped "$SLAM_C"
docker create -it --name "$SLAM_C" --network host --privileged --runtime nvidia --gpus all "${ENVS[@]}" \
  -w /slam_ws --entrypoint /bin/bash \
  -v "$KACHAKA_CODE_DIR:$KACHAKA_CODE_DIR_CT" \
  -v "$KACHAKA_WEIGHTS_DIR/vendor/weights:$SERVER_CT/vendor/weights:ro" \
  -v "$KACHAKA_WEIGHTS_DIR/src/weights:$SERVER_CT/src/weights:ro" \
  -v /dev:/dev -v "$FUNGI_HOST:/fungi" \
  "$KACHAKA_AI_IMAGE" >/dev/null
echo "✓ $SLAM_C created"

if docker inspect "$GW_C" >/dev/null 2>&1; then
    echo "$GW_C already exists — left as is"
else
    docker create -it --name "$GW_C" --network host --privileged --runtime nvidia --gpus all "${ENVS[@]}" \
      -w /gateway_slam_ws --entrypoint /ros_entrypoint.sh \
      -v /dev:/dev -v "$FUNGI_HOST:/fungi" \
      "$KACHAKA_GATEWAY_IMAGE" bash >/dev/null
    echo "✓ $GW_C created"
fi

echo; echo "mounts:"
for c in "$SLAM_C" "$GW_C"; do
    echo "  $c"; docker inspect -f '{{range .Mounts}}    {{.Source}} -> {{.Destination}} {{if not .RW}}(ro){{end}}{{"\n"}}{{end}}' "$c"
done
