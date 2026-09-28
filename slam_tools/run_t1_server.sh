#!/bin/bash
# T1 — SLAM server (ma-long streaming server) WITH dynamic-object masking.
#
#   ./run_t1_server.sh [extra server_api args, e.g. --mode rgb+depth+intr --backend ma]
#
# Runs inside $KACHAKA_AI_CONTAINER, from $KACHAKA_SERVER_DIR_CT (set both in .env). The server must
# have slam_integration/ installed (see slam_integration/install.sh).
#
#   --dynamic_mask        our masking (YOLOv9e-seg + FlowSeek, see dynamic_masking/)
#   DYNAMIC_GEO_GATE      anchor (default) = optical-flow blobs only count next to a YOLO-detected
#                         movable object; none = previous behaviour (motion channel alone can remove)
#   --no_deploy           skips the semantic export; add --semantic --sem_chunk_size 4 and drop
#                         --no_deploy for a full semantic map (needs a lot more GPU memory)
#
# Remove --dynamic_mask for a no-masking baseline.
. "$(dirname "$(readlink -f "$0")")/_env.sh"
docker start "$SLAM_C" >/dev/null 2>&1
docker exec "$SLAM_C" bash -c "pkill -9 -f ma_slam_stream.server_api 2>/dev/null; true" >/dev/null 2>&1
echo "======================================================================"
echo " Server : $SERVER_CT   (container $SLAM_C)"
echo " MASKING: ON  (--dynamic_mask, geo gate: ${DYNAMIC_GEO_GATE:-anchor})"
echo " Viewer : http://localhost:9090/?url=rerun%2Bhttp%3A%2F%2Flocalhost%3A9876%2Fproxy"
echo "======================================================================"
exec docker exec -it -e DYNAMIC_GEO_GATE="${DYNAMIC_GEO_GATE:-anchor}" "$SLAM_C" bash -c \
  "cd $SERVER_CT && PYTHONPATH=src CUDA_VISIBLE_DEVICES=0 \
   python -m ma_slam_stream.server_api \
     --depth_max 5 --submap_size 16 --keyframe_disparity 0 \
     --viz web --no_deploy --dynamic_mask \
     --default_out /fungi/outputs_malong $*"
