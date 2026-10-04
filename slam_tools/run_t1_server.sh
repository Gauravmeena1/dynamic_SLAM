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
#   DYNAMIC_PERSON_DILATE_PX / DYNAMIC_PERSON_BOX_FILL / DYNAMIC_BRIDGE_MAX_GAP / DYNAMIC_BOX_FILL_DOWN / DYNAMIC_EDGE_RING_PX
#                         person-pixel leftovers (defaults 5 / 1 / 3 / 0.6 / 12; 0 / 0 / 1 / 0 / 0 = before 2026-10-01)
#   DYNAMIC_CARVE=1       (default) after the map is written, drop points that other frames only ever
#                         see on removed (dynamic) pixels -> carved_pcd.ply; 0 = off
#   SEMANTIC=1            also build the semantic instance map + deploy files (--semantic
#                         --sem_chunk_size 4, no --no_deploy); needs ~20 GB more GPU memory.
#                         Default (unset): geometry + masking only (--no_deploy).
#
# Remove --dynamic_mask for a no-masking baseline.
. "$(dirname "$(readlink -f "$0")")/_env.sh"
docker start "$SLAM_C" >/dev/null 2>&1
docker exec "$SLAM_C" bash -c "pkill -9 -f ma_slam_stream.server_api 2>/dev/null; true" >/dev/null 2>&1
echo "======================================================================"
echo " Server : $SERVER_CT   (container $SLAM_C)"
if [ "${SEMANTIC:-0}" = 1 ]; then SEM_ARGS="--semantic --sem_chunk_size 4"; else SEM_ARGS="--no_deploy"; fi
echo " MASKING: ON  (--dynamic_mask, geo gate: ${DYNAMIC_GEO_GATE:-anchor})"
echo " SEMANTIC: $([ "${SEMANTIC:-0}" = 1 ] && echo "ON (instances + deploy)" || echo "off (--no_deploy)")"
echo " Viewer : http://localhost:9090/?url=rerun%2Bhttp%3A%2F%2Flocalhost%3A9876%2Fproxy"
echo "======================================================================"
exec docker exec -it -e DYNAMIC_GEO_GATE="${DYNAMIC_GEO_GATE:-anchor}" \
  -e DYNAMIC_PERSON_DILATE_PX="${DYNAMIC_PERSON_DILATE_PX:-5}" -e DYNAMIC_PERSON_BOX_FILL="${DYNAMIC_PERSON_BOX_FILL:-1}" \
  -e DYNAMIC_BRIDGE_MAX_GAP="${DYNAMIC_BRIDGE_MAX_GAP:-3}" -e DYNAMIC_BOX_FILL_DOWN="${DYNAMIC_BOX_FILL_DOWN:-0.6}" \
  -e DYNAMIC_EDGE_RING_PX="${DYNAMIC_EDGE_RING_PX:-12}" -e DYNAMIC_CARVE="${DYNAMIC_CARVE:-1}" "$SLAM_C" bash -c \
  "cd $SERVER_CT && PYTHONPATH=src CUDA_VISIBLE_DEVICES=0 \
   python -m ma_slam_stream.server_api \
     --depth_max 5 --submap_size 16 --keyframe_disparity 0 \
     --viz web $SEM_ARGS --dynamic_mask \
     --default_out /fungi/outputs_malong $*"
