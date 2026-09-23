#!/usr/bin/env bash
# run_bridge_oneshot.sh — 一鍵橋 run（救沒有 pose2d 的舊 3D run）
#
#   ./run_bridge_oneshot.sh --name bridge_20260826 --venue map4_20260826 \
#                           --old-run run_20260825
#   ./run_bridge_oneshot.sh --name b1 --venue v1 --plan-only   # 只規劃＋出圖，不動機器人
#
# 把 ROBOT_MANUAL §4.3「忘了開 logger 怎麼救」那一整段串起來：
#   匯出 2D 圖 → 產生蛇行路徑 → 出預覽圖 → 開 pose2d logger → 開 SLAM 收資料
#   → 用 move_to_pose 導航走完 → 收尾 → 印出 alignment 指令
#
# ⚠️ 為什麼不是直接 `run_slam_oneshot.sh robot`（2026-08-26 踩過，OPEN_ISSUES [KCK-04]）：
#    robot 模式靠 cmd_vel 原始速度驅動，而 Kachaka **載著家具時拒絕 cmd_vel 的旋轉**，
#    follower 會卡在「原地轉向」直到逾時，整趟靜默報廢。所以這裡拆成兩支並行：
#    collector 用 manual 模式（完全不碰機器人），驅動另外交給 move_to_pose 導航 API。
#
# ⚠️ 這支會讓機器人實際移動。真的下 --go 之前會停下來讓你看預覽圖並確認。
set -uo pipefail

# ── 執行環境（可由 .env 覆寫；見 .env.example）──────────────────────
WORK="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
[ -f "$WORK/.env" ] && { set -a; source "$WORK/.env"; set +a; }
resolve_from_work() {
    case "$1" in /*) printf '%s\n' "$1" ;; *) printf '%s/%s\n' "$WORK" "$1" ;; esac
}
TOOLS="${KACHAKA_TOOLS_DIR:-/opt/kachaka/fungi}"
KAPI="${KACHAKA_API_PATH:-/opt/kachaka/kachaka-api/python}"
PY="${KACHAKA_PYTHON:-python3}"
export PYTHONPATH="$KAPI${PYTHONPATH:+:$PYTHONPATH}"
ROBOT_ENV="${KACHAKA_ROBOT_ENV_FILE:-/opt/kachaka/robotic/.env}"
OUTPUT_DIR="$(resolve_from_work "${KACHAKA_OUTPUT_DIR:-artifacts/runs}")"
PREVIEW_DIR="$(resolve_from_work "${KACHAKA_PREVIEW_DIR:-artifacts/previews}")"
SESSION=bridge_oneshot
KACHAKA_DEFAULT="${KACHAKA_ENDPOINT:-KACHAKA_IP:26400}"
API="${KACHAKA_AI_API_URL:-http://localhost:3636}"

NAME=""; VENUE=""; OLD_RUN=""; SPACING=0.6; CLEARANCE=0.40; PATTERN=boustrophedon
# 2026-09-16 新地圖實測：0.25/0.8 產生 32 點；0.4/1.0 只要約 23 點，
# 最短點距反而更大、最大朝向差更小。每個 move_to_pose 都會停車收斂，所以採後者。
GOAL_SPACING=0.4; MIN_STEP=1.0; MIN_GOAL_DISTANCE=0.15
# 低於約 135° 的硬拆分可能在彎道插入公分級補點；145° 搭配抵達方向 yaw
# 可保留平滑曲線，又會拒絕真正的大幅跳變。
MAX_YAW_STEP_DEG=145; SETTLE=0.5; GOAL_TIMEOUT=30
ENDPOINT="$KACHAKA_DEFAULT"; PLAN_ONLY=0; ASSUME_YES=0; REUSE_MAP=0; DO_ALIGN=0

usage() {
    sed -n '2,17p' "$0" | sed 's/^# \{0,1\}//'
    cat <<'EOF'

必要參數 / Required:
  --name NAME        Run 名稱；controls output/alignment names
  --venue VENUE      場地名稱；controls kachaka_2d_<VENUE>.png/.yaml

選填 / Optional:
  --old-run RUN      要對齊的舊 3D run / Old 3D run to align after collection
  --spacing M        掃描線間距 / Sweep-line spacing (default: 0.6 m)
  --pattern P        boustrophedon（蛇行，預設 / default）| spiral（螺旋 / spiral）
  --min-clearance M  最小淨空 / Minimum goal clearance (default: 0.40 m)
  --goal-spacing M   導航目標間距 / Navigation-goal spacing (default: 0.4 m)
  --min-step M       直線段合併間距 / Straight-segment merge distance (default: 1.0 m)
  --min-goal-distance M  相鄰目標最小距離 / Minimum adjacent-goal distance (default: 0.15 m)
  --max-yaw-step-deg DEG  最大 yaw 變化 / Maximum yaw step (default: 145 deg)
  --settle SEC       到位停留時間 / Camera settling time (default: 0.5 s)
  --goal-timeout SEC 單點 timeout / Per-goal timeout (default: 30 s)
  --endpoint IP:PORT Kachaka gRPC endpoint（default: KACHAKA_ENDPOINT from .env）
  --reuse-map        沿用並驗證既有地圖 / Reuse and verify an existing map export
  --align            收集後執行 alignment / Run alignment after collection
  --plan-only        只規劃，不動硬體 / Plan and preview without hardware motion
  -y, --yes          略過確認 / Skip confirmation (not recommended)
EOF
}

while [ $# -gt 0 ]; do
    case "$1" in
        --name)          NAME="$2"; shift 2 ;;
        --venue)         VENUE="$2"; shift 2 ;;
        --old-run)       OLD_RUN="$2"; shift 2 ;;
        --spacing)       SPACING="$2"; shift 2 ;;
        --pattern)       PATTERN="$2"; shift 2 ;;
        --min-clearance) CLEARANCE="$2"; shift 2 ;;
        --goal-spacing)  GOAL_SPACING="$2"; shift 2 ;;
        --min-step)      MIN_STEP="$2"; shift 2 ;;
        --min-goal-distance) MIN_GOAL_DISTANCE="$2"; shift 2 ;;
        --max-yaw-step-deg) MAX_YAW_STEP_DEG="$2"; shift 2 ;;
        --settle)        SETTLE="$2"; shift 2 ;;
        --goal-timeout)  GOAL_TIMEOUT="$2"; shift 2 ;;
        --endpoint)      ENDPOINT="$2"; shift 2 ;;
        --reuse-map)     REUSE_MAP=1; shift ;;
        --align)         DO_ALIGN=1; shift ;;
        --plan-only)     PLAN_ONLY=1; shift ;;
        -y|--yes)        ASSUME_YES=1; shift ;;
        -h|--help)       usage; exit 0 ;;
        *) echo "未知參數 / Unknown option: $1"; usage; exit 2 ;;
    esac
done
[ -n "$NAME" ] && [ -n "$VENUE" ] || { usage; exit 2; }

C_OK=$'\033[32m'; C_WARN=$'\033[33m'; C_ERR=$'\033[31m'; C_DIM=$'\033[2m'; C_R=$'\033[0m'
ok()   { echo "${C_OK}  ✓${C_R} $*"; }
warn() { echo "${C_WARN}  !${C_R} $*"; }
die()  { echo "${C_ERR}  ✗${C_R} $*"; exit 1; }
step() { echo; echo "${C_DIM}────────────────────────────────────────────────────────${C_R}"; echo "▶ $*"; }
confirm() { [ "$ASSUME_YES" = 1 ] && return 0; read -r -p "$1 [y/N] " a; [[ "$a" =~ ^[Yy]$ ]]; }
# ⚠️ 2026-09-11：只 kill $LOGGER_PID 是不夠的 —— 那是 logger_keepalive.sh 的外殼，
#    真正在寫檔的 log_kachaka_pose2d.py 是它的子程序，母程序被殺之後會變孤兒繼續跑，
#    下一趟再開一支就有兩支寫同一個 csv（時間戳成對重複）。要連子程序一起收。
stop_logger() {
    [ -n "${LOGGER_PID:-}" ] && kill "$LOGGER_PID" 2>/dev/null
    pkill -f "log_kachaka_pose2d.py --out $POSE_LOG" 2>/dev/null
    LOGGER_PID=""
}

DATE=$(date +%Y%m%d)
mkdir -p "$OUTPUT_DIR" "$PREVIEW_DIR"
MAP_YAML="$OUTPUT_DIR/kachaka_2d_${VENUE}.yaml"
MAP_PNG="$OUTPUT_DIR/kachaka_2d_${VENUE}.png"
TRAJ="$OUTPUT_DIR/traj_${NAME}.csv"
GOALS="$OUTPUT_DIR/goals_${NAME}.csv"
POSE_LOG="$OUTPUT_DIR/pose2d_${NAME}.csv"
LOGGER_PID=""

cd "$WORK" || die "進不去 $WORK"

# ── 收尾：不管怎麼離開，logger 要停、SLAM 要正常收尾（絕不 kill T1，那會毀掉地圖）──
SLAM_OWNED=0
CLEANING=0
cleanup_resources() {
    [ "$CLEANING" = 1 ] && return
    [ "$SLAM_OWNED" = 0 ] && [ -z "${LOGGER_PID:-}" ] && return
    CLEANING=1
    echo
    warn "${1:-流程異常結束}，開始安全收尾 ..."
    if [ "$SLAM_OWNED" = 1 ] && tmux has-session -t "$SESSION" 2>/dev/null; then
        warn "送 Enter 讓 SLAM 收集正常結束（不是 kill）"
        tmux send-keys -t "$SESSION:collect" "" C-m 2>/dev/null
        sleep 5
    fi
    stop_logger && warn "已停 pose2d logger（含 keepalive 的子程序）"
    echo "SLAM 若還在收尾，讓它跑完；要看畫面：tmux attach -t $SESSION"
}
cleanup_on_exit() {
    rc=$?
    [ "$rc" -eq 0 ] || cleanup_resources "流程以 rc=$rc 結束"
}
handle_signal() {
    trap - EXIT
    cleanup_resources "收到中斷"
    exit 130
}
trap cleanup_on_exit EXIT
trap handle_signal INT TERM

# ══════════════════════════════════════════════════════════════════════
step "1/7 前置檢查"
# ══════════════════════════════════════════════════════════════════════
for f in make_bridge_traj.py drive_waypoints.py preview_traj.py; do
    [ -f "$WORK/$f" ] || die "找不到 $WORK/$f（本地副本，可以自己改）"
done
if [ "$PLAN_ONLY" = 0 ]; then
[ -f "$TOOLS/logger_keepalive.sh" ] || die "找不到 $TOOLS/logger_keepalive.sh"
# T1 一定要用 ~/ops 的修正版：acm 部署樹的 ma_slam_semantic/hooks.py 從 2026-09-02 起
# import 不存在的 dynamic_tracker（OPEN_ISSUES [MAP-08]），fungi 原版的 run_slam_oneshot.sh
# 會讓 T1 一啟動就 ModuleNotFoundError，然後這裡一路等到 900s 逾時，看起來像「卡在 T1」。
SLAM_SH="${KACHAKA_SLAM_RUNNER:-$HOME/ops/run_slam_oneshot.sh}"
if [ -x "$SLAM_SH" ]; then
    ok "SLAM 用修正版 $SLAM_SH（T1 走 run_t1_server_fixed.sh 的影子套件）"
else
    SLAM_SH="$TOOLS/run_slam_oneshot.sh"
    [ -f "$SLAM_SH" ] || die "找不到 $SLAM_SH"
    warn "找不到 ~/ops/run_slam_oneshot.sh，改用 fungi 原版 —— T1 很可能會炸在 dynamic_tracker"
fi
command -v tmux >/dev/null || die "找不到 tmux"
fi

# 已知會讓整趟白跑的環境問題（卡住的 ros2 CLI、殘留 logger、部署樹壞掉…）一次查完。
# 清單與來由見 KNOWN_ISSUES.md；獨立執行：./preflight.sh --name <run>
if [ -x "$WORK/preflight.sh" ]; then
    echo
    preflight_args=(--name "$NAME")
    [ "$PLAN_ONLY" = 1 ] && preflight_args+=(--plan-only)
    "$WORK/preflight.sh" "${preflight_args[@]}" || {
        echo
        die "preflight 有 FAIL（見上），修完再跑 —— 現在停下來比跑到一半發現便宜"
    }
    echo
else
    warn "找不到 $WORK/preflight.sh，跳過環境預檢"
fi

# Kachaka：沿用 .env 的 IP（DHCP 會漂，見 OPEN_ISSUES [INF-02]）
if [ "$ENDPOINT" = "$KACHAKA_DEFAULT" ]; then
    kip=$(grep -oP '^KACHAKA_IP=\K.*' "$ROBOT_ENV" 2>/dev/null)
    [ -n "$kip" ] && ENDPOINT="${kip}:26400"
fi
host="${ENDPOINT%%:*}"; port="${ENDPOINT##*:}"
timeout 3 bash -c "echo > /dev/tcp/$host/$port" 2>/dev/null \
    || die "連不上 Kachaka $ENDPOINT —— 開機了嗎？IP 漂了嗎？(~/robotic/find_kachaka.sh)"
ok "Kachaka $ENDPOINT"

# 目前載入哪張圖 + 載貨狀態（載貨才是這支腳本存在的理由，不是錯誤）
"$PY" - "$ENDPOINT" "$MAP_YAML" "$REUSE_MAP" <<'EOF' || die "查 Kachaka 狀態或 reuse-map 身分驗證失敗"
import sys
from pathlib import Path
for _p in (Path.home()/'robotic/robotic_system/kachaka-main/kachaka-api/python',
           Path('/opt/kachaka/kachaka-api/python')):
    if _p.is_dir():
        sys.path.insert(0, str(_p))
import kachaka_api
import yaml
c = kachaka_api.KachakaApiClient(target=sys.argv[1])
cur = c.get_current_map_id()
name = next((m.name for m in c.get_map_list() if m.id == cur), '?')
if sys.argv[3] == '1':
    map_yaml = Path(sys.argv[2])
    if not map_yaml.is_file():
        raise SystemExit(f"--reuse-map 指定的 YAML 不存在：{map_yaml}")
    saved = yaml.safe_load(map_yaml.read_text()) or {}
    expected_id = saved.get('map_id')
    expected_name = saved.get('map_name')
    mismatch = (expected_id and expected_id != cur) or (not expected_id and expected_name != name)
    if mismatch:
        raise SystemExit(f"拒絕沿用不同地圖：檔案={expected_name!r}/{expected_id or '舊檔無 map_id'}，"
                         f"目前={name!r}/{cur}。換 venue 名重新匯出。")
p = c.get_robot_pose()
print(f"  ✓ 目前地圖 {name} ({cur[:8]})")
print(f"  ✓ 機器人 map pose ({p.x:+.3f}, {p.y:+.3f}) yaw {p.theta:+.3f}")
s = c.get_moving_shelf_id()
print(f"  {'!' if s else '✓'} 載貨狀態 get_moving_shelf_id = {s!r}"
      + ("  ← 載著家具，所以本腳本用 move_to_pose 而非 cmd_vel（OPEN_ISSUES [KCK-04]）" if s else ""))
EOF
warn "上面那張地圖必須就是你要對齊的正式地圖 —— 錯了的話 pose2d 會落在錯的座標系，整趟白跑"
confirm "地圖正確，繼續？" || exit 0

# ══════════════════════════════════════════════════════════════════════
step "2/7 匯出 2D 地圖 → kachaka_2d_${VENUE}"
# ══════════════════════════════════════════════════════════════════════
if [ "$REUSE_MAP" = 1 ] && [ -f "$MAP_YAML" ] && [ -f "$MAP_PNG" ]; then
    ok "沿用既有 $MAP_YAML（--reuse-map）"
else
    [ -f "$MAP_YAML" ] && warn "$MAP_YAML 已存在，將覆蓋（要保留就用 --reuse-map）"
    "$PY" - "$ENDPOINT" "$MAP_PNG" "$MAP_YAML" <<'EOF' || die "匯出地圖失敗"
import sys
from pathlib import Path
for _p in (Path.home()/'robotic/robotic_system/kachaka-main/kachaka-api/python',
           Path('/opt/kachaka/kachaka-api/python')):
    if _p.is_dir():
        sys.path.insert(0, str(_p))
import kachaka_api
ep, png, yml = sys.argv[1], Path(sys.argv[2]), Path(sys.argv[3])
c = kachaka_api.KachakaApiClient(target=ep)
map_id = c.get_current_map_id()
m = c.get_png_map()
png.write_bytes(m.data)
yml.write_text("# 從 get_png_map() 自動匯出\n"
               f"resolution: {m.resolution}\norigin: [{m.origin.x}, {m.origin.y}, 0.0]\n"
               f"width: {m.width}\nheight: {m.height}\nmap_name: {m.name}\nmap_id: {map_id}\nimage: {png.name}\n")
print(f"  ✓ {m.name}  {m.width}x{m.height} @ {m.resolution:.4f} m/px")
EOF
fi

# ══════════════════════════════════════════════════════════════════════
step "3/7 規劃路徑 + 出預覽圖"
# ══════════════════════════════════════════════════════════════════════
"$PY" "$WORK/make_bridge_traj.py" --map-yaml "$MAP_YAML" --spacing "$SPACING" --pattern "$PATTERN" \
    --min-clearance "$CLEARANCE" --endpoint "$ENDPOINT" --out "$TRAJ" \
    --preview "$PREVIEW_DIR/${NAME}_raw.png" || die "產生路徑失敗"

"$PY" "$WORK/drive_waypoints.py" "$TRAJ" --map-yaml "$MAP_YAML" \
    --min-clearance "$CLEARANCE" --goal-spacing "$GOAL_SPACING" --min-step "$MIN_STEP" --settle "$SETTLE" \
    --max-yaw-step-deg "$MAX_YAW_STEP_DEG" --min-goal-distance "$MIN_GOAL_DISTANCE" \
    --goal-timeout "$GOAL_TIMEOUT" --save-goals "$GOALS" --endpoint "$ENDPOINT" \
    || die "抽稀導航目標失敗"
"$PY" "$WORK/preview_traj.py" --map-yaml "$MAP_YAML" --traj "$GOALS" --arrow-every 1 \
    --out "$PREVIEW_DIR/preview_goals_${NAME}.png" \
    || warn "預覽圖產生失敗（不影響後續）"

echo
warn "打開 $PREVIEW_DIR/preview_goals_${NAME}.png 確認：路徑有沒有涵蓋舊 run 走過的區域、有沒有貼牆"
if [ "$PLAN_ONLY" = 1 ]; then
    echo; ok "--plan-only：規劃完成，沒有動任何硬體"
    echo "  路徑 $TRAJ"; echo "  目標 $GOALS"
    exit 0
fi
confirm "路徑 OK，開始跑（機器人會移動）？" || exit 0

# ══════════════════════════════════════════════════════════════════════
step "4/7 開 pose2d logger（漏掉整趟白跑）"
# ══════════════════════════════════════════════════════════════════════
n0=0; [ -f "$POSE_LOG" ] && n0=$(wc -l < "$POSE_LOG")
PYTHONPATH="$KAPI${PYTHONPATH:+:$PYTHONPATH}" \
    "$TOOLS/logger_keepalive.sh" "$POSE_LOG" >/dev/null 2>&1 &
LOGGER_PID=$!
sleep 12
n1=0; [ -f "$POSE_LOG" ] && n1=$(wc -l < "$POSE_LOG")
[ "$n1" -gt "$n0" ] || die "logger 沒有在寫（$n0 → $n1 行）。這是最容易讓整趟白跑的坑，先查清楚
      看訊息：tail ${POSE_LOG%.csv}.log"
ok "logger 正常寫入（$n0 → $n1 行）→ $POSE_LOG"

# ══════════════════════════════════════════════════════════════════════
step "5/7 啟動 SLAM 收資料（manual 模式：collector 完全不碰機器人）"
# ══════════════════════════════════════════════════════════════════════
tmux kill-session -t "$SESSION" 2>/dev/null
tmux new-session -d -s "$SESSION" -n collect
SLAM_OWNED=1
tmux send-keys -t "$SESSION:collect" \
    "cd $TOOLS && $SLAM_SH manual --name $NAME -y" C-m

echo "  ${C_DIM}冷啟動要載權重、開相機，數分鐘正常。畫面：tmux attach -t $SESSION${C_R}"
# ⚠️ 不要用 `tmux capture-pane | grep 畫面字串` 判斷（2026-08-26 實測失敗，整趟報廢）：
#    capture-pane 不加 -S 只看得到**可見畫面**，「收集中」那行被後續輸出捲掉之後就再也
#    抓不到，迴圈永遠空轉到逾時；而且它偵測不到「collect 視窗已經結束」，不會報錯，
#    導航因此從沒執行、機器人一步沒動，卻照樣收出一趟 10 幀的廢資料。
#    改看兩個權威訊號：AI 節點 /status 的 is_mapping，以及 collect 視窗是否還活著。
api_mapping() { curl -s -m 5 "$API/status" 2>/dev/null | grep -q '"is_mapping":true'; }
collect_alive() { tmux list-panes -t "$SESSION:collect" -F '#{pane_dead}' 2>/dev/null | grep -q '^0$'; }

t0=$SECONDS
while true; do
    api_mapping && { ok "SLAM 開始收集（$((SECONDS-t0))s，/status is_mapping=true）"; break; }
    if ! collect_alive; then
        tmux capture-pane -p -t "$SESSION:collect" -S -40 2>/dev/null | tail -25
        die "collect 視窗已經結束但從未進入收集狀態（見上，多半是 preflight 失敗）"
    fi
    [ $((SECONDS-t0)) -ge 900 ] && die "15 分鐘還沒開始收集，自己看：tmux attach -t $SESSION"
    sleep 5
done

# ══════════════════════════════════════════════════════════════════════
step "6/7 導航（move_to_pose 逐點；Kachaka 自己的避障會生效）"
# ══════════════════════════════════════════════════════════════════════
"$PY" "$WORK/drive_waypoints.py" "$TRAJ" --map-yaml "$MAP_YAML" \
    --min-clearance "$CLEARANCE" --goal-spacing "$GOAL_SPACING" --min-step "$MIN_STEP" --settle "$SETTLE" \
    --max-yaw-step-deg "$MAX_YAW_STEP_DEG" --min-goal-distance "$MIN_GOAL_DISTANCE" \
    --goal-timeout "$GOAL_TIMEOUT" --endpoint "$ENDPOINT" --go
drive_rc=$?
[ "$drive_rc" -eq 0 ] || warn "導航非正常結束（rc=$drive_rc）—— 還是會照常收尾，資料不浪費"

step "6.5 結束收集（送 Enter，不是 kill）"
tmux send-keys -t "$SESSION:collect" "" C-m
SLAM_OWNED=0
echo "  ${C_DIM}收尾要 flush → 語意匯出 → deploy，1–2 分鐘。別中斷。${C_R}"
# 收尾完成一樣不看畫面：deploy 產物齊了（15 檔）或 collect 視窗自己結束，才算完成。
DEPLOY_DIR="$TOOLS/outputs_malong/$NAME/deploy"
t0=$SECONDS
while true; do
    nd=$(ls -1 "$DEPLOY_DIR" 2>/dev/null | wc -l)
    [ "$nd" -ge 15 ] && { ok "收尾完成（$((SECONDS-t0))s，deploy $nd/15）"; break; }
    collect_alive || { ok "collect 視窗已結束（deploy $nd/15）"; break; }
    [ $((SECONDS-t0)) -ge 1200 ] && { warn "20 分鐘還沒收完（deploy $nd/15），自己看：tmux attach -t $SESSION"; break; }
    sleep 5
done

stop_logger; ok "已停 pose2d logger（含 keepalive 的子程序）"

# ══════════════════════════════════════════════════════════════════════
step "7/7 Alignment"
# ══════════════════════════════════════════════════════════════════════
RUN_DIR="$TOOLS/outputs_malong/$NAME"
[ -d "$RUN_DIR" ] || die "找不到 $RUN_DIR —— 收集那段是不是失敗了？"
ok "橋 run 產物 $RUN_DIR（$(ls "$RUN_DIR/deploy" 2>/dev/null | wc -l)/15 deploy 檔）"

ALIGN_CMD="$TOOLS/align.sh $NAME $DATE $VENUE"
REG_CMD="conda run -n dualmap python register_runs.py \\
    outputs_malong/${OLD_RUN:-<舊 run>} outputs_malong/$NAME \\
    --out $OUTPUT_DIR/align_bridge_${DATE}/T_old_to_bridge.json"

if [ "$DO_ALIGN" = 1 ]; then
    warn "align.sh 會寫進 $TOOLS —— 你的帳號沒寫入權的話這步會失敗，要請 acm 帳號跑"
    echo; echo "  執行 $ALIGN_CMD"
    $ALIGN_CMD || die "align.sh 失敗 —— 停在這裡，不要帶著半套資料往下跑"
    if [ -n "$OLD_RUN" ]; then
        mkdir -p "$OUTPUT_DIR/align_bridge_${DATE}"
        echo; echo "  執行配準"
        eval "$REG_CMD" || die "register_runs.py 失敗"
        echo; warn "判讀：fitness > 0.5、inlier_rmse < voxel、tilt < 5° 才算可用"
    fi
else
    cat <<EOF

  接下來自己跑（確認過橋 run 品質再跑）：

    cd $TOOLS
    $ALIGN_CMD
$( [ -n "$OLD_RUN" ] && echo "    $REG_CMD" )

  判讀：sim2 的 rmse_all 超過 0.3 m 要查；配準要 fitness > 0.5、inlier_rmse < voxel、tilt < 5°。
EOF
fi

cat <<EOF

  這趟的東西：
    3D    $TOOLS/outputs_malong/$NAME
    pose2d $POSE_LOG
    路徑   $TRAJ / $GOALS
    圖     $PREVIEW_DIR/preview_goals_${NAME}.png
  全部收工：tmux kill-session -t $SESSION
EOF
