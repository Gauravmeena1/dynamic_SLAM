# slam_tools/_env.sh — sourced by every slam_tools script. Loads the repo's .env (never committed)
# and sets the names below. Machine-specific values live in .env only; see .env.example.
#
#   KACHAKA_TOOLS_DIR      host workspace mounted at /fungi in both containers (gateway code, outputs)
#   KACHAKA_SLAM_CONTAINER camera/ROS 2 gateway container      (default gateway_slam)
#   KACHAKA_AI_CONTAINER   SLAM server + masking container      (default slam_node)
#   KACHAKA_SERVER_DIR_CT  SLAM server repo, path INSIDE the AI container (default /workspace/ma-long-server)
#   KACHAKA_ROBOTIC_DIR    Kachaka bridge project (robot mode only)
REPO="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/.." && pwd)"
[ -f "$REPO/.env" ] && { set -a; . "$REPO/.env"; set +a; }
FUNGI_HOST="${KACHAKA_TOOLS_DIR:-/opt/kachaka/fungi}"
GW_C="${KACHAKA_SLAM_CONTAINER:-gateway_slam}"
SLAM_C="${KACHAKA_AI_CONTAINER:-slam_node}"
SERVER_CT="${KACHAKA_SERVER_DIR_CT:-/workspace/ma-long-server}"
ROBOTIC_HOST="${KACHAKA_ROBOTIC_DIR:-/opt/kachaka/robotic}"
