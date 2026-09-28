#!/usr/bin/env bash
# run_slam_oneshot.sh — one-click 3D semantic mapping (fungi V2 / ma-long)
#
#   ./run_slam_oneshot.sh robot            # live: Kachaka follows traj.csv + RealSense
#   ./run_slam_oneshot.sh manual           # live: just collect RGB-D, robot doesn't self-drive (you push it), default 120s
#   ./run_slam_oneshot.sh folder           # offline: replay an image folder, touches no hardware
#   ./run_slam_oneshot.sh robot --preflight-only    # only run the preflight checks, don't start
#
# This replaces the manual three-terminal flow of run_t1_server.sh /
# run_t2_gateway.sh / run_t3_main.sh: opens one tmux session (windows
# T1/T2/T3), waits for each stage to actually be ready before moving to the
# next, presses 1 automatically to start, polls until final_result, and
# prints the resulting artifacts.
#
# You can always `tmux attach -t slam` to see the three windows' raw output.
#
# ⚠️ The one mistake you cannot recover from is "killing T1 before it
#    finalizes" -- the map sitting in memory disappears instantly. That's why
#    this script's Ctrl+C sends POST /stop to finalize cleanly, never a kill.
#
# ── Where things live ─────────────────────────────────────────────────────
# The two containers it drives ($KACHAKA_AI_CONTAINER = SLAM server + masking, $KACHAKA_SLAM_CONTAINER
# = ROS 2 camera gateway) both mount $KACHAKA_TOOLS_DIR at /fungi, so captured frames, point clouds
# and mask_viz/ land in $KACHAKA_TOOLS_DIR/outputs_malong/<run_name> on the host. All names and
# paths come from the repo's .env (see slam_tools/_env.sh and .env.example).
set -uo pipefail

# ── constants ─────────────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"   # run_t1/t2/t3 live here
# FUNGI_HOST (workspace mounted at /fungi), ROBOTIC_HOST (robot mode only), SLAM_C (AI container),
# GW_C (gateway container) come from .env via _env.sh
. "$SCRIPT_DIR/_env.sh"
BRIDGE_C=ros2_bridge-ros2_bridge-1    # Kachaka gRPC -> ROS 2 bridge
API=http://localhost:3636
SESSION=slam
MIN_FREE_GB=30                        # one run's rgb/depth frames can eat several GB

# ── args ──────────────────────────────────────────────────────────────────
MODE_ARG="${1:-}"; [ $# -gt 0 ] && shift
RUN_NAME=""; OUT_URI=""; DURATION=120; ROOT_DIR="/fungi/test_run_EDB1/rgb"
PUBLISH_RATE=1.0; SLAM_MODE=""; SLAM_BACKEND=""; MAX_MINUTES=60
ASSUME_YES=0; PREFLIGHT_ONLY=0; SKIP_BRIDGE=0; KEEP_TMUX=1

usage() {
    sed -n '2,17p' "$0" | sed 's/^# \{0,1\}//'
    cat <<'EOF'

Options:
  --name NAME          run name (default run_YYYYmmdd_HHMMSS, also sets the output dir and deploy filenames)
  --out  URI           output location (default file:///fungi/outputs_malong/<name>)
  --duration SEC       manual mode collection duration in seconds (default 120)
  --root DIR           folder mode's source directory (default /fungi/test_run_EDB1/rgb)
  --rate N             folder mode replay fps (default 1.0, close to live-robot pace)
  --mode M             override SLAM mode: rgb+depth+intr | rgb+intr | rgb
  --backend B          override backend: ma | da3   (da3 only accepts rgb, passing depth is a hard error)
  --max-minutes N      auto-finalize after N minutes (default 60, safety net against runaway runs)
  --no-bridge          don't manage the Kachaka bridge automatically (you already started it)
  --preflight-only     only run checks
  -y                   skip all confirmations
EOF
}

while [ $# -gt 0 ]; do
    case "$1" in
        --name)         RUN_NAME="$2"; shift 2 ;;
        --out)          OUT_URI="$2"; shift 2 ;;
        --duration)     DURATION="$2"; shift 2 ;;
        --root)         ROOT_DIR="$2"; shift 2 ;;
        --rate)         PUBLISH_RATE="$2"; shift 2 ;;
        --mode)         SLAM_MODE="$2"; shift 2 ;;
        --backend)      SLAM_BACKEND="$2"; shift 2 ;;
        --max-minutes)  MAX_MINUTES="$2"; shift 2 ;;
        --no-bridge)    SKIP_BRIDGE=1; shift ;;
        --preflight-only) PREFLIGHT_ONLY=1; shift ;;
        -y|--yes)       ASSUME_YES=1; shift ;;
        -h|--help)      usage; exit 0 ;;
        *) echo "unknown argument: $1"; usage; exit 2 ;;
    esac
done

case "$MODE_ARG" in
    robot|manual|folder) ;;
    -h|--help) usage; exit 0 ;;
    "") usage; exit 2 ;;
    *)  echo "scenario must be robot / manual / folder"; exit 2 ;;
esac

# Default mode: live robot uses depth (tightest geometry, see fungi's
# README_VER2 §4 "rgb-only quality has a ceiling"); folder mode has no depth,
# so it must use rgb + da3.
if [ -z "$SLAM_MODE" ]; then
    [ "$MODE_ARG" = "folder" ] && SLAM_MODE="rgb" || SLAM_MODE="rgb+depth+intr"
fi
if [ -z "$SLAM_BACKEND" ]; then
    [ "$MODE_ARG" = "folder" ] && SLAM_BACKEND="da3" || SLAM_BACKEND="ma"
fi
if [ "$SLAM_BACKEND" = "da3" ] && [[ "$SLAM_MODE" == *depth* ]]; then
    echo "❌ backend da3 does not support depth (server_api.py:534 will ap.error). Use --backend ma instead"
    exit 2
fi

[ -z "$RUN_NAME" ] && RUN_NAME="run_$(date +%Y%m%d_%H%M%S)"
[ -z "$OUT_URI" ]  && OUT_URI="file:///fungi/outputs_malong/${RUN_NAME}"
OUT_DIR_CT="${OUT_URI#file://}"                       # path inside the container
OUT_DIR_HOST="${OUT_DIR_CT/\/fungi/$FUNGI_HOST}"      # corresponding host path

# ── small helpers ────────────────────────────────────────────────────────
C_OK=$'\033[32m'; C_WARN=$'\033[33m'; C_ERR=$'\033[31m'; C_DIM=$'\033[2m'; C_R=$'\033[0m'
ok()   { echo "${C_OK}  ✓${C_R} $*"; }
warn() { echo "${C_WARN}  !${C_R} $*"; }
die()  { echo "${C_ERR}  ✗${C_R} $*"; echo; echo "${C_ERR}preflight failed, nothing was started.${C_R}"; exit 1; }
step() { echo; echo "${C_DIM}────────────────────────────────────────────────────────${C_R}"; echo "▶ $*"; }

confirm() {   # confirm "message"
    [ "$ASSUME_YES" = 1 ] && return 0
    read -r -p "$1 [y/N] " a; [[ "$a" =~ ^[Yy]$ ]]
}

api()      { curl -s -m 5 "$API/status" 2>/dev/null; }
api_field(){ api | jq -r "$1 // empty" 2>/dev/null; }

gw_ros()   { docker exec "$GW_C" bash -c "source /opt/ros/humble/setup.bash \
             && source /gateway_slam_ws/install/setup.bash && $1" 2>/dev/null; }

# wait_until "description" timeout_seconds condition_command...
wait_until() {
    local msg="$1" timeout="$2"; shift 2
    local t0=$SECONDS
    printf '  … %s' "$msg"
    while ! "$@" >/dev/null 2>&1; do
        if [ $((SECONDS - t0)) -ge "$timeout" ]; then
            printf '\r'; echo "${C_ERR}  ✗${C_R} $msg — timed out after ${timeout}s"
            return 1
        fi
        printf '.'
        sleep 2
    done
    printf '\r'; ok "$msg ($((SECONDS - t0))s)"
    return 0
}

# ══════════════════════════════════════════════════════════════════════════
# 1. PREFLIGHT — checks and cleanup only, does not start SLAM
# ══════════════════════════════════════════════════════════════════════════
step "Preflight (scenario = ${MODE_ARG}, mode = ${SLAM_MODE} / ${SLAM_BACKEND})"

# --- 1.1 host basics ---
command -v docker >/dev/null || die "docker not found"
command -v tmux   >/dev/null || die "tmux not found"
command -v jq     >/dev/null || die "jq not found"

free_gb=$(df -BG --output=avail "$FUNGI_HOST" | tail -1 | tr -dc '0-9')
if [ "${free_gb:-0}" -lt "$MIN_FREE_GB" ]; then
    warn "only ${free_gb}G free on disk (recommend >${MIN_FREE_GB}G) -- rgb/depth frames can eat several GB"
    confirm "continue anyway?" || exit 1
else
    ok "disk free: ${free_gb}G"
fi

if [ -e "$OUT_DIR_HOST" ]; then
    warn "output directory already exists: $OUT_DIR_HOST"
    confirm "overwrite?" || die "pick a different --name and rerun"
fi

# --- 1.2 container / image ---
for c in "$SLAM_C" "$GW_C"; do
    docker inspect "$c" >/dev/null 2>&1 || die "container '$c' does not exist. See ~/fungi/README_VER2.md §1 for the four docker scenarios"
done
docker start "$SLAM_C" >/dev/null 2>&1
docker start "$GW_C"   >/dev/null 2>&1
ok "containers started: $SLAM_C / $GW_C"

# ROS_DOMAIN_ID must be 0 (a container once got stuck baked with =42, took hours to find)
gw_domain=$(docker exec "$GW_C" printenv ROS_DOMAIN_ID 2>/dev/null)
[ "${gw_domain:-0}" = "0" ] || die "gateway's ROS_DOMAIN_ID=${gw_domain} (must be 0). Fix: $FUNGI_HOST/recreate_gateway_container.sh"
ok "gateway ROS_DOMAIN_ID=0"

# --- 1.3 weights ---
[ -d "$FUNGI_HOST/semantic_slam/src/ma-long/vendor/weights" ] \
    || die "vendor/weights not found (OpenSe3r semantic weights)"
[ -f "$FUNGI_HOST/semantic_slam/src/ma-long/src/weights/dino_salad.ckpt" ] \
    || die "dino_salad.ckpt not found (SALAD loop closure)"
ok "weights in place"

# --- 1.4 camera (robot / manual) ---
# ⚠️ Grepping lsusb alone is not enough (bitten by this on 2026-08-13): even
#    with "a" RealSense plugged in, it might be a different camera than
#    expected. The collector pins a specific serial via
#    cfg.enable_device(CAMERA_SERIAL) (replay_and_collect_ros2.py:522) -- a
#    serial mismatch gives "No device connected", which then waits out the
#    server's 90s intrinsics timeout before failing -- burning a full cold
#    start loading 14G of weights for nothing. So this enumerates devices
#    inside the container via pyrealsense2 directly and cross-checks serial +
#    USB version.
if [ "$MODE_ARG" != "folder" ]; then
    lsusb 2>/dev/null | grep -qi "RealSense" || die "no RealSense visible on USB"

    cam_serial=$(grep -oP '^CAMERA_SERIAL\s*=\s*\K[0-9]+' \
        "$FUNGI_HOST/gateway_slam/src/gateway_node_robot.py" 2>/dev/null | head -1)

    # Enumerate: each line is "serial usb_version". If the camera is held by
    # another process, enumeration can come back empty, hence the timeout.
    cam_list=$(timeout 30 docker exec "$GW_C" python3 -c '
import pyrealsense2 as rs
for d in rs.context().query_devices():
    try:
        print(d.get_info(rs.camera_info.serial_number),
              d.get_info(rs.camera_info.usb_type_descriptor))
    except Exception:
        pass
' 2>/dev/null)

    if [ -z "$cam_list" ]; then
        # enumeration failing doesn't necessarily mean no camera (could be held
        # by something else / pyrealsense misbehaving) -- fall back to the old
        # behavior but say so clearly
        warn "could not enumerate any RealSense inside the container (held by another process? just plugged in and not settled yet?)"
        warn "  check yourself: docker exec $GW_C python3 -c 'import pyrealsense2 as rs; print(len(rs.context().query_devices()))'"
        confirm "continue anyway? (a serial mismatch will only fail after 90s)" || exit 1
    else
        echo "$cam_list" | while read -r s u; do echo "    ${C_DIM}· $s  (USB $u)${C_R}"; done

        if [ -z "$cam_serial" ]; then
            warn "gateway CAMERA_SERIAL=None -> using the first camera found"
            cam_usb=$(echo "$cam_list" | head -1 | awk '{print $2}')
        else
            cam_usb=$(echo "$cam_list" | awk -v s="$cam_serial" '$1==s {print $2; exit}')
            if [ -z "$cam_usb" ]; then
                echo "${C_ERR}  ✗ gateway's configured camera ${cam_serial} is not online${C_R}"
                echo "    two ways to fix:"
                echo "      (a) plug in the camera with serial ${cam_serial} (recommended, no code change)"
                echo "      (b) edit CAMERA_SERIAL in $FUNGI_HOST/gateway_slam/src/gateway_node_robot.py:33"
                die "camera serial mismatch"
            fi
            ok "configured camera ${cam_serial} is online"
        fi

        # The D435's usable stream combinations are very limited on USB2, start()
        # falls all the way back to 640x480@15, which visibly hurts
        # reconstruction quality -- caught here so you switch ports before the
        # run, not after.
        case "${cam_usb:-}" in
            3.*) ok "USB ${cam_usb}" ;;
            "")  warn "could not read the USB version (can be empty if the camera has never been opened)" ;;
            *)   warn "camera is on USB ${cam_usb} -- not USB3! Depth quality will be noticeably worse"
                 warn "  switch to Thor's blue USB3 port (and the cable too -- a USB2 cable in a USB3 port is still 2.x)"
                 confirm "run with USB ${cam_usb} anyway?" || exit 1 ;;
        esac
    fi
fi

# --- 1.5 Kachaka + bridge (only needed for robot; manual never touches ROS/odom) ---
if [ "$MODE_ARG" = "robot" ]; then
    # ⚠️ Deliberately not calling find_kachaka.sh: its first step,
    #    `getent hosts kachaka-XXXX.local`, hangs indefinitely on this machine
    #    (Kachaka doesn't answer mDNS, and the nss lookup doesn't fail, it just
    #    hangs, so it never reaches its own port-scan fallback). This does it
    #    directly instead: try the cached .env value first, then scan the subnet.
    probe_k() { timeout 2 bash -c "echo > /dev/tcp/$1/26400" 2>/dev/null; }
    kip=$(grep -oP '^KACHAKA_IP=\K.*' "$ROBOTIC_HOST/.env" 2>/dev/null)

    if [ -n "$kip" ] && probe_k "$kip"; then
        ok "Kachaka = ${kip}:26400 (from .env)"
    else
        [ -n "$kip" ] && warn "${kip} from .env is unreachable (DHCP drifted?), scanning the subnet"
        subnet=$(ip -4 -o addr show scope global | awk '{print $4}' | head -1 | cut -d/ -f1 | cut -d. -f1-3)
        [ -n "$subnet" ] || die "could not determine local subnet"
        # Firing 254 concurrent connections at once on wifi causes them to
        # contend with each other, even a genuinely open port can time out
        # (observed intermittently). Batch 32 at a time + 3s timeout + two passes.
        scan_subnet() {
            local sf; sf=$(mktemp)
            local i b
            for b in $(seq 1 32 254); do   # seq FIRST INCREMENT LAST
                for i in $(seq "$b" $((b + 31))); do
                    [ "$i" -le 254 ] || break
                    ( timeout 3 bash -c "echo > /dev/tcp/${subnet}.$i/26400" 2>/dev/null \
                        && echo "${subnet}.$i" ) &
                done >>"$sf" 2>/dev/null
                wait
            done
            head -1 "$sf"; rm -f "$sf"
        }
        for attempt in 1 2; do
            echo "  … scanning ${subnet}.0/24 port 26400 (pass ${attempt})"
            kip=$(scan_subnet)
            [ -n "$kip" ] && break
        done
        [ -n "$kip" ] || die "no host with port 26400 open on this subnet -- is the robot powered on? same router as Thor (SSID ACM)?"
        printf 'KACHAKA_IP=%s\nKACHAKA_GRPC_PORT=26400\n' "$kip" > "$ROBOTIC_HOST/.env"
        ok "Kachaka = ${kip}:26400 (found by scan, written back to .env)"
    fi

    if [ "$SKIP_BRIDGE" = 0 ]; then
        if ! docker ps --format '{{.Names}}' | grep -qx "$BRIDGE_C"; then
            echo "  … bridge is not running, starting it (start_kachaka_bridge.sh -d)"
            ( cd "$ROBOTIC_HOST" && timeout 180 ./start_kachaka_bridge.sh -d ) >/dev/null 2>&1 \
                || die "start_kachaka_bridge.sh failed or timed out. Run it manually to see the error:
      cd $ROBOTIC_HOST && ./start_kachaka_bridge.sh"
            sleep 12
        fi
    fi

    # The topic name existing doesn't mean anyone is publishing -- must check
    # the publisher count (see fungi's README §4 troubleshooting step 1).
    pub=$(gw_ros "ros2 topic info /kachaka/odometry/odometry --verbose --no-daemon" \
          | awk '/Publisher count/{print $3; exit}')
    if [ "${pub:-0}" -ge 1 ] 2>/dev/null; then
        ok "odometry has a publisher (count=$pub)"
    else
        die "/kachaka/odometry/odometry has no publisher -> no odometry, the follower will never send cmd_vel,
      the robot won't move, the view will be static, and the LK gate will drop every frame.
      Fix: docker restart $BRIDGE_C (wait 15s) and rerun this script"
    fi

    [ -f "$FUNGI_HOST/traj.csv" ] || die "$FUNGI_HOST/traj.csv not found"
    ok "trajectory traj.csv ($(( $(wc -l < "$FUNGI_HOST/traj.csv") - 1 )) points)"
fi

# --- 1.6 folder mode's source directory ---
if [ "$MODE_ARG" = "folder" ]; then
    src_host="${ROOT_DIR/\/fungi/$FUNGI_HOST}"
    [ -d "$src_host" ] || die "source folder does not exist: $src_host"
    nimg=$(find "$src_host" -maxdepth 2 -type f \
           \( -iname '*.jpg' -o -iname '*.png' -o -iname '*.jpeg' -o -iname '*.bmp' -o -iname '*.webp' \) | wc -l)
    [ "$nimg" -gt 0 ] || die "$src_host has no images in it"
    ok "source folder $ROOT_DIR ($nimg images)"
fi

# --- 1.7 clean up leftovers from a previous run ---
# ⚠️ tmux kill-session only kills the host-side `docker exec` client -- the
#    processes inside the container survive. If a previous run's
#    gateway_node/streamer wasn't cleaned up, the next run ends up with two
#    sets of processes publishing frames at once -- observed frame count
#    literally doubling (74 -> 148), the resulting map is two runs stacked on
#    top of each other. Must kill inside the container.
# ⚠️ Patterns must always be written as [x]xx: `pkill -f 'python3 gateway_node'`
#    matches its own bash -c wrapper (the wrapper's cmdline contains that exact
#    string), so the wrapper gets killed first and the intended pkill never runs.
tmux kill-session -t "$SESSION" 2>/dev/null
docker exec "$SLAM_C" bash -c "pkill -9 -f '[m]a_slam_stream.server_api'; true" >/dev/null 2>&1
docker exec "$GW_C"   bash -c "pkill -9 -f '[p]ython3 gateway_node';       \
                               pkill -9 -f '[p]ython3 main_system';        \
                               pkill -9 -f '[r]eplay_and_collect'; true"   >/dev/null 2>&1
sleep 2
leftover=$(docker exec "$GW_C" bash -c \
    "pgrep -cf '[p]ython3 (gateway_node|main_system)|[r]eplay_and_collect'" 2>/dev/null)
[ "${leftover:-0}" = "0" ] || die "still $leftover leftover process(es) inside the container that couldn't be killed, check manually:
      docker exec $GW_C ps -ef | grep -E 'gateway_node|main_system'"
ok "leftovers cleared (old server / gateway / streamer / collector / tmux session)"

echo
echo "  output -> $OUT_DIR_HOST"
if [ "$PREFLIGHT_ONLY" = 1 ]; then echo; ok "preflight passed (--preflight-only, nothing started)"; exit 0; fi

# the one step that can't be automated
if [ "$MODE_ARG" = "robot" ]; then
    echo
    echo "${C_WARN}  The robot must already be positioned near traj.csv's starting point, facing the right way, not docked/e-stopped.${C_R}"
    confirm "in position, start?" || { echo "cancelled."; exit 0; }
fi

# ══════════════════════════════════════════════════════════════════════════
# 2. START T1 / T2 / T3
# ══════════════════════════════════════════════════════════════════════════
tmux new-session -d -s "$SESSION" -n T1
tmux set-option -t "$SESSION" remain-on-exit on >/dev/null 2>&1

step "T1 — AI server ($SLAM_MODE / $SLAM_BACKEND)"
tmux send-keys -t "$SESSION:T1" \
    "cd $SCRIPT_DIR && ./run_t1_server.sh --mode $SLAM_MODE --backend $SLAM_BACKEND" C-m
echo "  ${C_DIM}cold start loads 14G of weights, a few minutes is normal${C_R}"
wait_until "waiting for :3636 to be ready" 900 bash -c "curl -s -m 3 $API/status | jq -e '.mode' >/dev/null" \
    || { echo "  check T1's screen: tmux attach -t $SESSION"; exit 1; }

# mode mismatch check -- this is what guards against the "viewer is completely
# empty / Stop Failed" trap
got_mode=$(api_field .mode); got_backend=$(api_field .backend)
if [ "$got_mode" != "$SLAM_MODE" ] || [ "$got_backend" != "$SLAM_BACKEND" ]; then
    echo "${C_ERR}  ✗ server came up but mode doesn't match: wanted ${SLAM_MODE}/${SLAM_BACKEND}, got ${got_mode}/${got_backend}${C_R}"
    echo "    (whichever block is currently active in run_server_thor.sh, e.g. rgb-only + da3)"
    exit 1
fi
ok "server ready: mode=$got_mode backend=$got_backend"

curl -s -m 10 -X POST "$API/cleanup" >/dev/null 2>&1   # clear any leftover session, otherwise the goal gets rejected

step "T2 — Gateway"
case "$MODE_ARG" in
    robot)  T2CMD="./run_t2_gateway.sh robot" ;;
    manual) T2CMD="./run_t2_gateway_manual.sh" ;;   # built-in --manual --duration 120
    folder) T2CMD="./run_t2_gateway.sh folder" ;;
esac
# for a custom duration in manual mode, bypass the wrapper script and call
# gateway_node_robot.py directly
if [ "$MODE_ARG" = "manual" ] && [ "$DURATION" != "120" ]; then
    T2CMD="docker exec -it -e COLLECT_EXTRA=\"${COLLECT_EXTRA:-}\" $GW_C bash -c 'source /opt/ros/humble/setup.bash && source /gateway_slam_ws/install/setup.bash && cd /fungi/gateway_slam/src && python3 gateway_node_robot.py --manual --duration $DURATION'"
fi
tmux new-window -t "$SESSION" -n T2
tmux send-keys -t "$SESSION:T2" "cd $SCRIPT_DIR && $T2CMD" C-m
wait_until "waiting for the run_slam action server" 120 \
    bash -c "docker exec $GW_C bash -c 'source /opt/ros/humble/setup.bash && source /gateway_slam_ws/install/setup.bash && ros2 service list --no-daemon' 2>/dev/null | grep -q start_inference_system" \
    || { echo "  check T2's screen: tmux attach -t $SESSION"; exit 1; }

step "T3 — control panel"
if [ "$MODE_ARG" = "folder" ]; then
    T3CMD="./run_t3_main.sh folder --ros-args -p root_dir:=$ROOT_DIR -p output_uri:=$OUT_URI -p publish_rate:=$PUBLISH_RATE"
else
    T3CMD="./run_t3_main.sh robot --ros-args -p output_uri:=$OUT_URI"
fi
tmux new-window -t "$SESSION" -n T3
tmux send-keys -t "$SESSION:T3" "cd $SCRIPT_DIR && $T3CMD" C-m
wait_until "waiting for the control panel" 90 \
    bash -c "tmux capture-pane -p -t $SESSION:T3 | grep -q 'Enter command'" \
    || { echo "  check T3's screen: tmux attach -t $SESSION"; exit 1; }

tmux send-keys -t "$SESSION:T3" "1" C-m
ok "sent START (equivalent to a human pressing 1)"

# ══════════════════════════════════════════════════════════════════════════
# 3. MONITOR UNTIL FINALIZATION IS DONE
# ══════════════════════════════════════════════════════════════════════════
STOP_SENT=0
send_stop() {
    [ "$STOP_SENT" = 1 ] && return
    STOP_SENT=1
    echo; echo "${C_WARN}▶ sending STOP, finalizing (flush + semantic export + deploy, 1-2 minutes)${C_R}"
    tmux send-keys -t "$SESSION:T3" "2" C-m 2>/dev/null
    ( curl -s -m 900 -X POST "$API/stop" >/dev/null 2>&1 ) &   # safety net in case T3 didn't get the keypress
}
on_int() {
    if [ "$STOP_SENT" = 0 ]; then
        echo; echo "${C_WARN}Ctrl+C received -- will NOT kill T1 (that would destroy the map), finalizing normally instead.${C_R}"
        send_stop
    else
        echo; echo "${C_WARN}already finalizing. Wait a bit; if you really need to leave, T1 is still alive, later you can run:${C_R}"
        echo "  curl -s -m 900 -X POST $API/stop"
        exit 130
    fi
}
trap on_int INT TERM

step "monitoring (tmux attach -t $SESSION to see the raw screens)"
# ⚠️ Cannot rely solely on /status.final_result: after finalizing, the
#    gateway calls /cleanup in the background
#    (gateway_node_robot.py:_trigger_cleanup_background), after which /status
#    falls back to its "no session" defaults: final_result=false /
#    num_splats=0 / map_uri="". Trusting only that field spins forever.
#    So completion uses a three-way check instead: final_result / deploy
#    artifacts on disk / session having disappeared.
DEPLOY_HOST="$OUT_DIR_HOST/deploy"
deploy_done() { [ -d "$DEPLOY_HOST" ] && [ "$(ls -1 "$DEPLOY_HOST" 2>/dev/null | wc -l)" -ge "$DEPLOY_N" ]; }

DEPLOY_N=15   # number of files that make deploy/ complete (also used as the finalize progress denominator)

# ⚠️ "still collecting frames" and "already finalizing" look almost identical
#    in /status (is_mapping is true in both) -- watching only that makes the
#    1-2 minute finalize step look stuck (a user has asked "why isn't it
#    stopping past the duration"). The real boundary is only visible on T2's
#    screen, so read that instead.
t2_pane() { tmux capture-pane -p -J -t "$SESSION:T2" 2>/dev/null; }

t0=$SECONDS; last=""; saw_mapping=0; seen_pts=0; seen_uri=""; tty_out=0
phase="collect"; announced=0; fin_t0=$SECONDS; cap_start_ms=""; frames=""
[ -t 1 ] && tty_out=1
while true; do
    st=$(api)
    if [ -z "$st" ]; then
        deploy_done && { echo; ok "server no longer responding, but deploy artifacts are complete -> treating as done"; break; }
        warn "temporarily can't reach :3636"
        sleep 5; continue
    fi
    final=$(echo "$st"   | jq -r '.final_result // false')
    mapping=$(echo "$st" | jq -r '.is_mapping // false')
    pts=$(echo "$st"     | jq -r '.num_splats // 0')
    q=$(echo "$st"       | jq -r '.queue_size // 0')
    uri=$(echo "$st"     | jq -r '.map_uri // empty')
    err=$(echo "$st"     | jq -r '.error // empty')

    [ "$mapping" = "true" ] && saw_mapping=1
    [ "$pts" != "0" ] && seen_pts="$pts"
    [ -n "$uri" ] && seen_uri="$uri"

    [ -n "$err" ] && { echo; echo "${C_ERR}  ✗ AI node reported an error: $err${C_R}"; echo "  full traceback is on T1's screen"; send_stop; }

    # ── progress line ────────────────────────────────────────────────────
    if [ "$MODE_ARG" = "folder" ]; then
        line=$(printf "  %5ds | points %-10s | queue %-4s" "$((SECONDS - t0))" "$pts" "$q")
    else
        pane=$(t2_pane)
        # read the frame number straight from the collector's "[Captured] 0221_1786632314350"
        frames=$(echo "$pane" | grep -oP '\[Captured\] \K[0-9]+' | tail -1)

        # collection start time = the epoch ms embedded in the first frame id
        # (more accurate than a ROS timestamp: the ~10s of camera init isn't
        # counted against --duration). Only extracted once, and needs the full
        # scrollback to find it.
        if [ -z "$cap_start_ms" ]; then
            cap_start_ms=$(tmux capture-pane -p -J -S - -t "$SESSION:T2" 2>/dev/null \
                | grep -oP '\[Captured\] [0-9]+_\K[0-9]{13}' | head -1)
        fi

        if [ "$phase" = "collect" ] \
           && echo "$pane" | grep -qE 'Capture duration reached|Collector exited safely|Collector completed'; then
            phase="finalize"
        fi

        if [ "$phase" = "collect" ]; then
            if [ -n "$cap_start_ms" ] && [ "$MODE_ARG" = "manual" ] && [ "$DURATION" -gt 0 ]; then
                left=$(( DURATION - ( $(date +%s) - cap_start_ms / 1000 ) ))
                [ "$left" -lt 0 ] && left=0
                head=$(printf "collecting %4ds left" "$left")
            else
                head=$(printf "collecting %5ds " "$((SECONDS - t0))")
            fi
            line=$(printf "  %s | frame %-5s | points %-9s | queue %-3s" \
                   "$head" "${frames:-0}" "$pts" "$q")
        else
            if [ "$announced" = 0 ]; then
                announced=1; fin_t0=$SECONDS
                echo
                echo "${C_OK}▶ frame collection finished (${frames:-?} frames) -- the robot can stop now${C_R}"
                echo "${C_DIM}  now finalizing: flush -> semantic export -> deploy, about 1-2 minutes.${C_R}"
                echo "${C_DIM}  no point-count movement during this stage is normal, don't Ctrl+C and don't kill T1.${C_R}"
            fi
            nd=$(ls -1 "$DEPLOY_HOST" 2>/dev/null | wc -l)
            line=$(printf "  finalizing %4ds | deploy %2s/%s | exporting, do not kill T1" \
                   "$((SECONDS - fin_t0))" "$nd" "$DEPLOY_N")
        fi
    fi

    if [ "$line" != "$last" ]; then
        last="$line"
        [ "$tty_out" = 1 ] && printf '\r%-78s' "$line" || { [ $(( (SECONDS-t0) % 60 )) -lt 5 ] && echo "$line"; }
    fi

    [ "$final" = "true" ] && { echo; ok "AI node reported final_result"; break; }
    deploy_done            && { echo; ok "deploy artifacts complete (15 files)"; break; }
    # session was wiped by /cleanup: mapping happened, everything is now back to zero
    if [ "$saw_mapping" = 1 ] && [ "$mapping" = "false" ] && [ "$pts" = "0" ] && [ -z "$uri" ]; then
        sleep 20   # give deploy time to land on disk
        deploy_done && { echo; ok "session cleared, deploy artifacts complete"; break; }
        echo; warn "session was cleared but no deploy artifacts appeared -- check T1's screen"; break
    fi

    if [ "$STOP_SENT" = 0 ] && [ $((SECONDS - t0)) -ge $((MAX_MINUTES * 60)) ]; then
        echo; warn "exceeded --max-minutes ${MAX_MINUTES}, finalizing automatically"
        send_stop
    fi
    # Enter = finish collection normally (run_bridge_oneshot.sh sends Enter at 6.5
    # when navigation is done). Same path as Ctrl+C: POST /stop, never a kill.
    if [ "$STOP_SENT" = 0 ] && [ -t 0 ]; then
        if read -r -t 5 _; then
            echo; warn "Enter received -- finishing collection (normal finalize, not a kill)"
            send_stop
        fi
    else
        sleep 5
    fi
done

# ══════════════════════════════════════════════════════════════════════════
# 4. RESULTS
# ══════════════════════════════════════════════════════════════════════════
step "done"
# /status has usually been wiped by /cleanup by this point, so use the values recorded while monitoring
ok "map_uri = ${seen_uri:-$OUT_DIR_CT}"
ok "points  = ${seen_pts}"

deploy_host="$DEPLOY_HOST"
if [ -d "$deploy_host" ]; then
    n=$(ls "$deploy_host" | wc -l)
    [ "$n" -ge 15 ] && ok "deploy/ has $n files (expected 15)" \
                    || warn "deploy/ only has $n files (expected 15) -- check T1's screen"
else
    warn "$deploy_host not found"
fi

if [ -f "$OUT_DIR_HOST/run_stats.txt" ]; then
    echo
    grep -E '^(backend|mode|n_frames|n_submaps|loops_accepted|fps_end_to_end|seconds|peak_vram_gb):' \
        "$OUT_DIR_HOST/run_stats.txt" | sed 's/^/    /'
fi

# the container runs as root, so artifacts land root:root on the host
if [ -d "$OUT_DIR_HOST" ] && [ "$(stat -c %U "$OUT_DIR_HOST")" = "root" ]; then
    warn "artifacts are owned by root. To fix ownership: sudo chown -R $USER:$USER $OUT_DIR_HOST"
fi

cat <<EOF

  artifacts: ${OUT_DIR_HOST}
    combined_pcd.ply / camera_poses.txt / run_stats.txt
    deploy/robot_deploy_${RUN_NAME}_*.{json,ply,png}

  T1 is still alive (the model stays resident), you can run the next run directly without reloading.
  full shutdown: tmux kill-session -t ${SESSION}
EOF
