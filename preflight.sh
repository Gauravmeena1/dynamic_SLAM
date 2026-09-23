#!/usr/bin/env bash
# preflight.sh — 開跑前把「已知會讓整趟白跑」的坑一次查完。
#
#   ./preflight.sh                      # 只查環境
#   ./preflight.sh --name ec129_0911    # 另外查這個 run 名會不會覆蓋既有資料
#
# 每一條都是實際踩過的（見 KNOWN_ISSUES.md）。FAIL 會印出當下就能貼的修法。
# 全程唯讀，不碰任何硬體、不動機器人 —— 出發前在座位上先跑一次就好。
# （TODO：等目前這趟跑完，把它接進 run_bridge_oneshot.sh 的 1/7 自動執行。
#   執行中的 bash 腳本不能改，會從錯的偏移繼續讀。）
set -uo pipefail

WORK="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
[ -f "$WORK/.env" ] && { set -a; source "$WORK/.env"; set +a; }
TOOLS="${KACHAKA_TOOLS_DIR:-/opt/kachaka/fungi}"
KAPI="${KACHAKA_API_PATH:-/opt/kachaka/kachaka-api/python}"
PY="${KACHAKA_PYTHON:-python3}"
ROBOT_ENV="${KACHAKA_ROBOT_ENV_FILE:-/opt/kachaka/robotic/.env}"
SLAM_SH="${KACHAKA_SLAM_RUNNER:-$HOME/ops/run_slam_oneshot.sh}"
T1_SH="${KACHAKA_T1_RUNNER:-$HOME/ops/run_t1_server_fixed.sh}"
GW="${KACHAKA_SLAM_CONTAINER:-gateway_slam}"
MM="${KACHAKA_ARM_CONTAINER:-mm_container}"
NAME=""
PLAN_ONLY=0
usage() {
    cat <<'EOF'
preflight.sh — 現場唯讀前置檢查 / Read-only field preflight

Usage:
  ./preflight.sh [--name RUN_NAME] [--plan-only]

Options:
  --name RUN_NAME  檢查 run 名是否會覆蓋資料 / Check for an existing run name
  --plan-only      只檢查離線規劃需求 / Check only offline planning requirements
  -h, --help       顯示說明 / Show this help

This command does not move the robot or arm.
此指令不會移動機器人或手臂。
EOF
}
while [ $# -gt 0 ]; do
    case "$1" in
        --name) NAME="${2:-}"; shift 2 ;;
        --plan-only) PLAN_ONLY=1; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "未知參數 / Unknown option: $1" >&2; usage >&2; exit 2 ;;
    esac
done

C_OK=$'\033[32m'; C_WARN=$'\033[33m'; C_ERR=$'\033[31m'; C_DIM=$'\033[2m'; C_R=$'\033[0m'
FAIL=0
ok()   { echo "${C_OK}  ✓${C_R} $*"; }
warn() { echo "${C_WARN}  !${C_R} $*"; }
bad()  { echo "${C_ERR}  ✗${C_R} $*"; FAIL=1; }
fix()  { echo "${C_DIM}      修法：$*${C_R}"; }

echo "── 1. 本地工具鏈 ──────────────────────────────────────"
for f in make_bridge_traj.py drive_waypoints.py preview_traj.py run_bridge_oneshot.sh; do
    [ -f "$WORK/$f" ] || { bad "缺 $WORK/$f"; fix "從 $TOOLS 重新複製並照 README 改路徑"; }
done
if [ "$PLAN_ONLY" = 0 ]; then
    [ -x "$SLAM_SH" ] \
        && ok "SLAM 修正版 ~/ops/run_slam_oneshot.sh 在" \
        || { bad "缺 ~/ops/run_slam_oneshot.sh（T1 會炸在 dynamic_tracker）"; fix "見 KNOWN_ISSUES #6"; }
    [ -x "$T1_SH" ] \
        || { bad "缺 ~/ops/run_t1_server_fixed.sh"; fix "請檢查場地 T1 launcher / Check the site T1 launcher (MAP-08)"; }
fi

echo "── 2. kachaka_api 能不能 import ───────────────────────"
if PYTHONPATH="$KAPI" "$PY" -c "import kachaka_api" 2>/dev/null; then
    ok "kachaka_api OK（PYTHONPATH=$KAPI）"
else
    bad "import kachaka_api 失敗"; fix "確認 $KAPI 存在且可讀"
fi

if [ "$PLAN_ONLY" = 1 ]; then
    warn "--plan-only：略過 SLAM、磁碟、container、手臂、logger 與 run 產物檢查"
    echo
    [ "$FAIL" = 0 ] && echo "${C_OK}規劃前置檢查通過${C_R}" || echo "${C_ERR}有 FAIL，先修完再規劃${C_R}"
    exit "$FAIL"
fi

echo "── 3. ma-long 部署樹（T1 會不會一啟動就死）────────────"
HOOKS="$TOOLS/semantic_slam/src/ma-long/src/ma_slam_semantic/hooks.py"
if grep -q "dynamic_tracker" "$HOOKS" 2>/dev/null; then
    if [ -f "$HOME/ops/ma-long-fix/ma_slam_semantic/hooks.py" ]; then
        warn "acm 部署樹的 hooks.py 仍是壞的（import dynamic_tracker），但會被 ~/ops 影子套件遮蔽"
    else
        bad "acm 部署樹的 hooks.py 壞掉，且找不到 ~/ops/ma-long-fix/ 的修好版"
    fi
else
    ok "acm 部署樹的 hooks.py 看起來已經修好了（可以考慮不再走影子套件）"
fi

echo "── 4. 磁碟 ────────────────────────────────────────────"
free_gb=$(df -BG --output=avail "$TOOLS" | tail -1 | tr -dc '0-9')
if [ "${free_gb:-0}" -ge 50 ]; then ok "可用 ${free_gb}G"
elif [ "${free_gb:-0}" -ge 30 ]; then warn "只剩 ${free_gb}G（腳本門檻 30G，一趟吃數 GB）"
else bad "只剩 ${free_gb}G，低於門檻 30G"; fix "清 $TOOLS/outputs_malong 裡不要的 run"; fi

echo "── 5. container ───────────────────────────────────────"
for c in zealous_agnesi "$GW"; do
    docker inspect "$c" >/dev/null 2>&1 \
        && ok "$c 存在（Exited 沒關係，腳本會 docker start）" \
        || { bad "container $c 不存在"; fix "見 README_VER2 §1"; }
done
mm_running=0
if [ "$(docker inspect -f '{{.State.Running}}' "$MM" 2>/dev/null)" = "true" ]; then
    mm_running=1
    ok "$MM 在跑"
else
    warn "$MM 沒在跑（正式流程會自己 docker start；本次略過手臂 topic 檢查）"
fi
echo "${C_DIM}      註：manual 模式不需要 ros2_bridge，那個 container Exited 是正常的${C_R}"

echo "── 5b. 建圖相機有沒有被 arm_cam_tune.sh 的即時串流占住 ──"
if timeout 10 docker exec "$GW" bash -c "pgrep -f '[c]am_snap.py live' >/dev/null" 2>/dev/null; then
    bad "cam_snap.py live 還開著 —— SLAM collector 會 Device or resource busy 開不了相機"
    fix "$WORK/arm_cam_tune.sh live-stop"
else
    ok "沒有即時串流占用相機"
fi

echo "── 6. 卡住的 ros2 CLI（會吃光 DDS participant）────────"
stuck=$(timeout 20 docker exec "$MM" bash -c \
        "ps -eo etimes,args | awk '/[r]os2 topic/ && \$1 > 60 {print}'" 2>/dev/null)
if [ -z "$stuck" ]; then
    ok "沒有卡住的 ros2 topic CLI"
else
    bad "有卡住的 ros2 topic CLI —— 手臂檢查會假性逾時 120s"
    echo "$stuck" | sed 's/^/        /'
    fix "docker exec "$MM" pkill -f \"ros2 topic\""
fi

echo "── 7. 手臂（CAN + 節點）───────────────────────────────"
if [ "$mm_running" = 0 ]; then
    warn "$MM 未執行，略過 /joint_states_feedback；由 run_slam_oneshot 啟動後再驗"
else
fb=$(timeout 30 docker exec "$MM" bash -c \
     'source /opt/ros/humble/setup.bash && source /workspace/piper_ros/install/setup.bash && \
      timeout 10 ros2 topic echo /joint_states_feedback --field position --once' 2>&1 | head -1)
case "$fb" in
    array*) ok "手臂在回報 /joint_states_feedback" ;;
    *"free participant index"*) bad "DDS participant 被佔滿（見第 6 項）"
                                fix "docker exec "$MM" pkill -f \"ros2 topic\"" ;;
    *) bad "讀不到 /joint_states_feedback：${fb:0:80}"
       fix "先看 ip -s -d link show can_piper 的 RX 有沒有在漲；沒漲才是真的 CAN/電源問題（ARM-15）" ;;
esac
fi

echo "── 8. 殘留程序（上一趟失敗留下的）─────────────────────"
# 注意：一趟正在跑的時候本來就會有一支 logger（run_bridge_oneshot 的 4/7 開的）。
# 只有「沒有進行中的一趟卻還有 logger」或「同時有兩支以上」才是殘留 —— 兩支會對
# 同一個 csv 重複寫入，時間戳會成對重複。
stale=$(pgrep -af "log_kachaka_pose2d" | grep -v "preflight\|claude" || true)
n_log=$(printf '%s' "$stale" | grep -c . || true)
run_active=0; tmux has-session -t bridge_oneshot 2>/dev/null && run_active=1
if [ "$n_log" = 0 ]; then
    ok "沒有殘留的 pose2d logger"
elif [ "$run_active" = 1 ] && [ "$n_log" = 1 ]; then
    ok "有一趟正在進行，logger 1 支（正常，別殺它）"
else
    bad "pose2d logger $n_log 支 / 進行中的 run: $run_active —— 多的是上一趟 die 之後沒回收的"
    echo "$stale" | sed 's/^/        /'
    fix "確認沒有正在跑的一趟後：pkill -f log_kachaka_pose2d; pkill -f logger_keepalive.sh"
    fix "有正在跑的一趟就只 kill 掉開始時間較舊的那支 PID"
fi

echo "── 9. Kachaka ─────────────────────────────────────────"
endpoint="${KACHAKA_ENDPOINT:-KACHAKA_IP:26400}"
kip="${endpoint%%:*}"
if [ -f "$ROBOT_ENV" ]; then
    env_kip=$(grep -oP '^KACHAKA_IP=\K.*' "$ROBOT_ENV" 2>/dev/null)
    [ -n "$env_kip" ] && kip="$env_kip"
fi
target="$kip:${endpoint##*:}"
if timeout 3 bash -c "echo > /dev/tcp/$kip/${endpoint##*:}" 2>/dev/null; then
    ok "Kachaka $kip:${endpoint##*:} 通"
    PYTHONPATH="$KAPI" "$PY" - "$target" <<'EOF'
import sys
import kachaka_api
c = kachaka_api.KachakaApiClient(target=sys.argv[1])
cur = c.get_current_map_id()
print(f"  \033[32m✓\033[0m 目前地圖 {next((m.name for m in c.get_map_list() if m.id==cur),'?')} ({cur[:8]})"
      "  ← 必須就是你要對齊的那張")
b = c.get_battery_info()[0]
print(("  \033[32m✓\033[0m " if b > 30 else "  \033[33m!\033[0m ") + f"電量 {b:.0f}%")
s = c.get_moving_shelf_id()
print(f"  \033[32m✓\033[0m 載貨 {s!r}" + ("（載著架子 → 一定要走 move_to_pose，本腳本就是）" if s else ""))
EOF
else
    bad "連不上 Kachaka $kip:${endpoint##*:}"; fix "開機了嗎？IP 漂了嗎？~/robotic/find_kachaka.sh"
fi

if [ -n "$NAME" ]; then
    echo "── 10. run 名衝突 ────────────────────────────────────"
    d="$TOOLS/outputs_malong/$NAME"
    n=$(ls -1 "$d/deploy" 2>/dev/null | wc -l)
    if [ "$n" -ge 1 ]; then
        bad "$d 已經有 $n 個 deploy 檔 —— 重跑同名會覆蓋掉那趟資料（-y 不會問你）"
        fix "換一個 --name，或確認舊的不要了再手動刪"
    else
        ok "run 名 $NAME 沒有既有資料"
    fi
fi

echo
[ "$FAIL" = 0 ] && echo "${C_OK}全部通過${C_R}" || echo "${C_ERR}有 FAIL，先修完再跑${C_R}"
exit "$FAIL"
