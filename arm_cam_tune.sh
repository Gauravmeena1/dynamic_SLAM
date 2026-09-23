#!/usr/bin/env bash
# arm_cam_tune.sh — 調手臂「建圖收合姿態」用：送關節姿態 → 抓建圖相機一幀 → 看圖，
# 反覆到手臂支架不再入鏡為止。2026-09-11 起因：3D 點雲裡的白色拖線是手臂上的白色支架
# 從畫面底部入鏡、跟著相機軌跡被重建出來。
#
#   ./arm_cam_tune.sh status                 # 手臂目前關節角（rad）與跟收合姿態的差
#   ./arm_cam_tune.sh snap [label]           # 抓一幀 → $KACHAKA_PREVIEW_DIR/armcheck_<label>_<時間>.png
#   ./arm_cam_tune.sh stow                   # 送目前腳本用的建圖收合姿態（讀 ~/ops/run_slam_oneshot.sh）
#   ./arm_cam_tune.sh j5 <rad>               # 收合姿態但 joint5 改成 <rad>，送出後自動抓一幀
#   ./arm_cam_tune.sh j4 <rad> / j6 <rad>    # 同上，改別軸
#   ./arm_cam_tune.sh pose j1 j2 j3 j4 j5 j6 [gripper]   # 任意姿態（會檢查 URDF 範圍），送出後抓一幀
#   ./arm_cam_tune.sh live [port]            # 瀏覽器即時看：http://HOST_IP:<port>/（預設 8091）
#   ./arm_cam_tune.sh live-stop
#
# ⚠️ stow / j5 / pose 會讓手臂真的動：人手離開手臂、周圍淨空、站在急停可及處再下。
# ⚠️ snap / live 跟 SLAM 的 collector 搶同一台相機（由 KACHAKA_CAMERA_SERIAL 設定），只能在兩趟之間用。
#    建圖進行中要看畫面請用 $KACHAKA_TOOLS_DIR/live_view.py <run名>（只讀 run 目錄，不碰相機）。
#
# 找到乾淨的姿態後，改 ~/ops/run_slam_oneshot.sh 裡的 MAPPING_STOW_POSE（fungi 原版不可寫），
# 這支的 stow 子指令就會跟著用新值。
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
[ -f "$HERE/.env" ] && { set -a; source "$HERE/.env"; set +a; }
SNAP_PY="$HERE/cam_snap.py"
GW="${KACHAKA_SLAM_CONTAINER:-gateway_slam}"
MM="${KACHAKA_ARM_CONTAINER:-mm_container}"
VIZ="${KACHAKA_PREVIEW_DIR:-$HERE/artifacts/previews}"
case "$VIZ" in /*) ;; *) VIZ="$HERE/$VIZ" ;; esac
SLAM_SH="${KACHAKA_SLAM_RUNNER:-$HOME/ops/run_slam_oneshot.sh}"
THOR_IP=$(ip -4 -o addr show scope global 2>/dev/null | awk '$2 ~ /^wl/ {print $4}' | cut -d/ -f1 | head -1)
THOR_IP="${KACHAKA_HOST_IP:-${THOR_IP:-HOST_IP}}"

C_OK=$'\033[32m'; C_WARN=$'\033[33m'; C_ERR=$'\033[31m'; C_DIM=$'\033[2m'; C_R=$'\033[0m'
ok()   { echo "${C_OK}  ✓${C_R} $*"; }
warn() { echo "${C_WARN}  !${C_R} $*"; }
die()  { echo "${C_ERR}  ✗${C_R} $*"; exit 1; }

usage() {
    cat <<'EOF'
arm_cam_tune.sh — 建圖相機與手臂姿態工具 / Mapping-camera and arm-pose utility

Usage:
  ./arm_cam_tune.sh status
  ./arm_cam_tune.sh snap [label]
  ./arm_cam_tune.sh stow
  ./arm_cam_tune.sh j4|j5|j6 <rad>
  ./arm_cam_tune.sh pose j1 j2 j3 j4 j5 j6 [gripper]
  ./arm_cam_tune.sh live [port]
  ./arm_cam_tune.sh live-stop

Safety: stow, j4/j5/j6, and pose move the arm. Keep clear and stay near E-stop.
安全：stow、j4/j5/j6、pose 會移動手臂；保持淨空並站在急停旁。
EOF
}

# URDF 範圍（/workspace/piper_ros/install/piper_description/.../piper_description.urdf，2026-09-11 讀出）
# 注意：目前收合姿態的 joint2=-0.0316、joint3=+0.0445 其實都在範圍外，驅動會夾到 0 —— 這就是
# 「斷電垂下」的極限位置，讀回來永遠是 0.0，誤差 0.045 rad 是正常的。
LIM_LO=(-2.618  0.000 -2.967 -1.745 -1.220 -2.0944 0.00)
LIM_HI=( 2.618  3.140  0.000  1.745  1.220  2.0944 0.10)
NAMES=(joint1 joint2 joint3 joint4 joint5 joint6 gripper)

stow_pose() {  # 從 ~/ops 的修正版讀，跟建圖流程用同一組數字
    local s
    s=$(grep -oP 'MAPPING_STOW_POSE="\[\K[^\]]+' "$SLAM_SH" 2>/dev/null | head -1)
    [ -n "$s" ] || s="0.004483108, -0.031591084, 0.044534532, 0.023165632, 0.195494908, 0.0, 0.0"
    echo "$s" | tr -d ' '
}

ros_mm() { timeout 30 docker exec "$MM" bash -c "source /opt/ros/humble/setup.bash && source /workspace/piper_ros/install/setup.bash && $*" 2>/dev/null; }

read_fb() {  # 印 7 個數字（空白分隔）
    ros_mm "timeout 8 ros2 topic echo /joint_states_feedback --field position --once" \
        | grep -oP -- '-?\d+\.\d+(?:e[+-]?\d+)?|-?\d+' | head -7 | tr '\n' ' '
}

collector_running() { timeout 10 docker exec "$GW" bash -c "ps -eo args | grep -q '[r]eplay_and_collect'" 2>/dev/null; }
live_running() { timeout 10 docker exec "$GW" bash -c "pgrep -f '[c]am_snap.py live' >/dev/null" 2>/dev/null; }
ensure_gateway_running() {
    docker inspect "$GW" >/dev/null 2>&1 || die "找不到 container $GW"
    if [ "$(docker inspect -f "{{.State.Running}}" "$GW" 2>/dev/null)" != "true" ]; then
        warn "$GW 沒有執行，正在啟動"
        docker start "$GW" >/dev/null || die "啟動 $GW 失敗"
        local i
        for i in $(seq 1 20); do
            [ "$(docker inspect -f "{{.State.Running}}" "$GW" 2>/dev/null)" = "true" ] && {
                ok "$GW 已啟動"
                return 0
            }
            sleep 0.5
        done
        die "$GW 啟動後未進入 running 狀態"
    fi
}

guard_camera() {
    collector_running && die "SLAM collector 正在跑、相機被它用著 —— 建圖中請用 fungi/live_view.py 看畫面"
    live_running && die "即時串流還開著（同一台相機，一次只能一個人用）—— 先 $0 live-stop，或直接在瀏覽器看"
}

do_snap() {
    ensure_gateway_running
    guard_camera
    local label="${1:-now}" out
    mkdir -p "$VIZ"
    out="$VIZ/armcheck_${label}_$(date +%H%M%S).png"
    timeout 75 docker exec -i -e KACHAKA_CAMERA_SERIAL="${KACHAKA_CAMERA_SERIAL:-}" "$GW" timeout 60 python3 -u - snap /tmp/armcheck.png < "$SNAP_PY" \
        || die "抓幀失敗（相機被別人開著？插在 USB2？）"
    docker cp "$GW:/tmp/armcheck.png" "$out" >/dev/null || die "docker cp 失敗"
    ok "存到 $out"
    echo "${C_DIM}      紅線 = 畫面 90% 高度；支架露出來的通常在紅線以下的左下/正下方${C_R}"
}

send_pose() {  # $1 = "a,b,c,d,e,f,g"
    local IFS=','; local -a p=($1); unset IFS
    [ "${#p[@]}" = 7 ] || die "要 7 個數字（joint1..joint6, gripper），拿到 ${#p[@]} 個"
    local i bad=0
    for i in 0 1 2 3 4 5 6; do
        awk -v v="${p[$i]}" -v lo="${LIM_LO[$i]}" -v hi="${LIM_HI[$i]}" 'BEGIN{exit !(v<lo-0.05 || v>hi+0.05)}' \
            && { warn "${NAMES[$i]}=${p[$i]} 超出 URDF 範圍 [${LIM_LO[$i]}, ${LIM_HI[$i]}]"; bad=1; }
    done
    [ "$bad" = 0 ] || die "有關節超出範圍，不送。要硬送請自己改 LIM_*"
    collector_running && die "SLAM collector 正在跑 —— 建圖中動手臂會毀掉這趟，不送"
    echo "  送出 [${p[*]}]（手臂會動，4 秒 min-jerk）"
    ros_mm "ros2 topic pub --once /joint_states sensor_msgs/msg/JointState \
        \"{name: [joint1,joint2,joint3,joint4,joint5,joint6,gripper], position: [$(IFS=','; echo "${p[*]}")]}\"" >/dev/null \
        || die "送指令失敗（mm_container 沒在跑？DDS 額度被吃光？先跑 preflight.sh）"
    sleep 4
    local fb; fb=$(read_fb)
    [ -n "$fb" ] || die "讀不到 /joint_states_feedback"
    local -a f=($fb); local err
    err=$(for i in 0 1 2 3 4 5 6; do awk -v a="${f[$i]:-0}" -v b="${p[$i]}" 'BEGIN{d=a-b; if(d<0)d=-d; print d}'; done | sort -g | tail -1)
    printf "  讀回 [%s]\n" "$(printf '%.4f ' "${f[@]}")"
    awk -v e="$err" 'BEGIN{exit !(e<0.08)}' && ok "到位（最大誤差 ${err} rad）" || warn "最大誤差 ${err} rad > 0.08 —— 可能卡住或被範圍夾住，目視確認"
}

cmd="${1:-}"; shift || true
case "$cmd" in
    -h|--help|help) usage ;;
    status)
        echo "收合姿態（$SLAM_SH）: [$(stow_pose)]"
        fb=$(read_fb); [ -n "$fb" ] || die "讀不到 /joint_states_feedback（手臂沒通電？DDS 額度？→ preflight.sh 第 6/7 項）"
        echo "目前關節角:           [$(echo "$fb" | tr ' ' ',' | sed 's/,$//')]"
        collector_running && warn "SLAM collector 正在跑（相機被占用）" || ok "相機空著，可以 snap / live"
        ;;
    snap)      do_snap "${1:-now}" ;;
    stow)      send_pose "$(stow_pose)"; do_snap stow ;;
    j4|j5|j6)
        [ -n "${1:-}" ] || die "用法：$0 $cmd <rad>"
        idx=$(( ${cmd#j} - 1 ))
        IFS=',' read -r -a p <<< "$(stow_pose)"; p[$idx]="$1"
        send_pose "$(IFS=','; echo "${p[*]}")"; do_snap "${cmd}_${1}"
        ;;
    pose)
        [ $# -ge 6 ] || die "用法：$0 pose j1 j2 j3 j4 j5 j6 [gripper]"
        g="${7:-0.0}"
        send_pose "$1,$2,$3,$4,$5,$6,$g"; do_snap pose
        ;;
    live)
        ensure_gateway_running
        collector_running && die "SLAM collector 正在跑 —— 建圖中請用 fungi/live_view.py"
        port="${1:-8091}"
        docker cp "$SNAP_PY" "$GW:/tmp/cam_snap.py" >/dev/null || die "docker cp 失敗"
        docker exec "$GW" bash -c "pkill -f '[c]am_snap.py live' 2>/dev/null; true"
        docker exec -d -e KACHAKA_CAMERA_SERIAL="${KACHAKA_CAMERA_SERIAL:-}" "$GW" bash -c "python3 -u /tmp/cam_snap.py live $port > /tmp/cam_live.log 2>&1"
        sleep 2
        docker exec "$GW" bash -c "pgrep -f '[c]am_snap.py live' >/dev/null" \
            || { docker exec "$GW" cat /tmp/cam_live.log; die "串流沒起來（見上）"; }
        ok "瀏覽器開 http://${THOR_IP}:${port}/   （結束：$0 live-stop）"
        echo "${C_DIM}      開著串流時可以另開終端下 j5/pose，畫面會即時跟著變${C_R}"
        ;;
    live-stop)
        docker exec "$GW" bash -c "pkill -f '[c]am_snap.py live' 2>/dev/null; true"; ok "已停"
        ;;
    *) usage; exit 2 ;;
esac
