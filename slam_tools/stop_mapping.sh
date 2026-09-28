#!/bin/bash
# Safely shut down mapping, regardless of how the previous run was interrupted
# (Ctrl+C not caught, terminal closed, the arm lost power and took the
# terminal down with it, etc.). Cleans up everything needed so the next
# preflight check passes again.
#
# Usage: ./stop_mapping.sh
#
# Copied from ~/fungi/stop_mapping.sh, comments translated to English;
# behavior unchanged. Talks to the same shared containers (zealous_agnesi /
# $GW_C) as fungi's copy.
set -uo pipefail
. "$(dirname "$(readlink -f "$0")")/_env.sh"

echo "========================================"
echo "[1/3] AI node (:3636) clean shutdown"
echo "========================================"
if curl -s -m 3 http://localhost:3636/status >/dev/null 2>&1; then
  STATUS=$(curl -s -m 3 http://localhost:3636/status)
  echo "current status: $STATUS"
  if echo "$STATUS" | grep -q '"is_mapping":true' && echo "$STATUS" | grep -q '"is_done":false'; then
    echo "still stuck 'mapping', sending /stop ..."
    # /stop needs to compress/downsample the point cloud, which can take a
    # while; a curl timeout does not mean it failed -- checking /status
    # afterward is the only reliable signal.
    curl -s -X POST -m 60 http://localhost:3636/stop >/dev/null 2>&1
    echo "sending /cleanup ..."
    curl -s -X POST -m 30 http://localhost:3636/cleanup >/dev/null 2>&1
    sleep 2
    NEW_STATUS=$(curl -s -m 3 http://localhost:3636/status 2>/dev/null || echo "")
    if echo "$NEW_STATUS" | grep -q '"is_mapping":false'; then
      echo "✅ confirmed AI node finished shutting down (is_mapping:false)"
    else
      echo "⚠️ /status still shows is_mapping:true, might still be processing, rerun ./stop_mapping.sh in a bit to confirm"
      echo "   current status: $NEW_STATUS"
    fi
  else
    echo "✅ AI node is not stuck mapping, nothing to do"
  fi
else
  echo "AI node (:3636) not responding, the SLAM server was probably not running, skipping"
fi

echo
echo "========================================"
echo "[2/3] leftover gateway_node / main_system inside $GW_C"
echo "========================================"
if docker ps --format '{{.Names}}' | grep -qx "$GW_C"; then
  LEFT=$(docker exec "$GW_C" ps -ef | grep -E '[g]ateway_node|[m]ain_system' || true)
  if [ -n "$LEFT" ]; then
    echo "found leftovers:"
    echo "$LEFT"
    docker exec "$GW_C" bash -c "pkill -TERM -f '[g]ateway_node' ; pkill -TERM -f '[m]ain_system'" 2>/dev/null
    sleep 2
    STILL=$(docker exec "$GW_C" ps -ef | grep -E '[g]ateway_node|[m]ain_system' || true)
    if [ -n "$STILL" ]; then
      echo "SIGTERM didn't clean it up, switching to SIGKILL ..."
      echo "$STILL" | awk '{print $2}' | while read -r pid; do
        docker exec "$GW_C" kill -KILL "$pid" 2>/dev/null
      done
      sleep 1
    fi
    FINAL=$(docker exec "$GW_C" ps -ef | grep -E '[g]ateway_node|[m]ain_system' || true)
    if [ -n "$FINAL" ]; then
      echo "❌ still couldn't clean it up, check manually:"
      echo "$FINAL"
    else
      echo "✅ cleaned up"
    fi
  else
    echo "✅ no leftovers"
  fi
else
  echo "$GW_C is not running, skipping"
fi

echo
echo "========================================"
echo "[3/3] live_view.py (live preview)"
echo "========================================"
if pgrep -f "live_view\.py" >/dev/null; then
  pkill -f "live_view\.py"
  echo "✅ stopped"
else
  echo "✅ not running"
fi

echo
echo "===== cleanup done, recommended to run ~/preflight_check.sh to confirm ====="
echo "(this script does not touch the pose2d logger -- if this was just this one mapping run"
echo " being interrupted and you're about to rerun, you usually don't need to stop it)"
