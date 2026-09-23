# kachaka_mapping —— 自動產生 Kachaka 建圖路線 / Portable field copy

從 `/home/acm/fungi` 複製過來的工具鏈（2026-09-11）。原版把所有輸出寫死在
`/home/acm/fungi`，那個目錄是 `acm:acm drwxrwxr-x`，本帳號不能寫，原版連
`--plan-only` 都會在 2/7 掛掉。這份副本把**產物路徑改到本目錄**，其餘邏輯未動。

## 檔案

| 檔案 | 說明 |
|---|---|
| `run_bridge_oneshot.sh` | 一鍵七步：匯出 2D 圖 → 產生路徑 → 抽稀＋預覽 → logger → SLAM → 導航走完 → alignment |
| `make_bridge_traj.py` | 從 Kachaka 2D 圖產生覆蓋路徑（蛇行 / 螺旋） |
| `drive_waypoints.py` | 抽稀成導航目標並用 `move_to_pose` 逐點走（`--go` 才會動） |
| `preview_traj.py` | 目標點預覽圖（含朝向箭頭） |

**沒有複製**、仍然呼叫 fungi 原版的（它們吃那邊的 container、權重、outputs_malong）：
`run_slam_oneshot.sh`、`logger_keepalive.sh`、`align.sh`。

## 相對原版的修改

1. `run_bridge_oneshot.sh`：`FUNGI` 拆成兩個變數 ——
   `WORK`（本目錄，放 2D 圖 / traj / goals / pose2d / align 結果）和
   `TOOLS=/home/acm/fungi`（唯讀：`run_slam_oneshot.sh`、`logger_keepalive.sh`、
   `align.sh`、`outputs_malong`）。三支 python 改叫本目錄的副本。
2. kachaka_api 的 import 路徑：原版只看 `~/robotic/...`（acm 專屬），
   非 acm 帳號會 `ModuleNotFoundError`。改成先找 `~/robotic`、再退回
   `/home/acm/robotic`（`run_bridge_oneshot.sh` 裡兩段內嵌 python 也一起改）。
3. 預覽圖預設目錄 `~/viz` → `~/viz_charles`（本帳號既有的可視化目錄）。
4. 4/7 的 pose2d logger：fungi 的 `log_kachaka_pose2d.py:26` 同樣把 kachaka_api 路徑寫死成
   `~/robotic`（acm 專屬），非 acm 帳號會 ModuleNotFoundError → logger 靜靜死掉 →
   腳本報「logger 沒有在寫（0 → 0 行）」。那支留在 fungi 沒有複製過來，所以改成呼叫
   `logger_keepalive.sh` 時用 `PYTHONPATH=$KAPI` 從外面補。順手把 `n0`/`n1` 改成先
   `[ -f ]` 再 `wc`，不然首次執行會噴一行 `No such file or directory` 的假錯誤。
5. 5/7 啟動 SLAM 改叫 `~/ops/run_slam_oneshot.sh`（你自己的修正版）而不是 fungi 原版：
   acm 部署樹的 `ma_slam_semantic/hooks.py` 從 2026-09-02 起 import 不存在的
   `dynamic_tracker`（OPEN_ISSUES [MAP-08]），原版會讓 T1 一啟動就 ModuleNotFoundError、
   T1 視窗掉回 shell，而外層一路等到 900 s 逾時，看起來像「卡在 T1」。
   修正版的 T1 走 `~/ops/run_t1_server_fixed.sh` 的影子套件。找不到修正版時會退回原版並警告。
6. `--align` 分支加一行警告：`align.sh` 會寫進 `/home/acm/fungi`，本帳號會失敗，
   要請 acm 帳號跑。

## 2026-09-14 遠端安全強化

- `--min-clearance` 已同步套用到軌跡生成與 waypoint 檢查。
- 新增 `--max-yaw-step-deg`（預設 70°）：抽稀遇到大轉角會補回原始點，超標則拒絕執行。
- `--goal-timeout` 預設 30 秒：單點逾時會取消 command，確認 10 秒內停止後跳下一點；
  只有取消後仍未停止才以 124 中止整趟。
- Ctrl+C、API 錯誤及 wrapper 非正常退出都會取消導航、收 logger，並收掉本趟啟動的 SLAM。
- pose log 改為 `pose2d_<run名>.csv`，同一天不同 run 不再共用檔案。
- 新匯出的 map YAML 會保存 `map_id`；`--reuse-map` 會和機器人當前 map id/name 比對，避免在錯圖規劃。
- `--plan-only` 使用輕量 preflight，不要求 SLAM/手臂 container 已啟動。
- 離線單元測試涵蓋成功、逾時取消、API 錯誤、端點保留與轉角限制。

## 調手臂建圖姿態 / 即時看相機（2026-09-11 新增）

3D 點雲裡的白色拖線 = 手臂上的白色支架從畫面底部入鏡、跟著相機軌跡被重建。用這支反覆
「送姿態 → 抓一幀 → 看圖」直到支架出鏡，再把數字填回 `~/ops/run_slam_oneshot.sh` 的
`MAPPING_STOW_POSE`（fungi 原版不可寫；`build_new_map.sh` 也有一份同名常數）：

```bash
./arm_cam_tune.sh status          # 目前關節角 + 相機是否空著
./arm_cam_tune.sh snap [label]    # 抓一幀 → ~/viz_charles/armcheck_<label>_<時間>.png（紅線 = 90% 高度）
./arm_cam_tune.sh live [8091]     # 瀏覽器即時看 http://HOST_IP:8091/ ；live-stop 結束
./arm_cam_tune.sh j5 0.35         # 收合姿態但 joint5 改 0.35 rad，送出後自動抓一幀（手臂會動）
./arm_cam_tune.sh pose j1 j2 j3 j4 j5 j6 [g]   # 任意姿態，會先對 URDF 範圍檢查
```

**目前生效的收合姿態（2026-09-11 實機確認鏡頭乾淨）**：
`[0.004483108, -0.031591084, 0.044534532, 0.023165632, 0.35, 0.0, 0.0]`（joint5 由 0.195 改 0.35）

`cam_snap.py` 是餵進 `gateway_slam` 執行的本體（pyrealsense2 直接開 <REALSENSE_SERIAL>）。
**只能在兩趟之間用** —— 建圖中相機被 collector 占用，要看畫面改用
`python3 /home/acm/fungi/live_view.py outputs_malong/<run名> --port 8092`（只讀 run 目錄）。

## 用法

```bash
cd ~/workspaces/kachaka_mapping
./run_bridge_oneshot.sh --name <run名> --venue <場地名> --plan-only   # 只規劃，不動硬體
./run_bridge_oneshot.sh --name <run名> --venue <場地名>               # 真的跑，會先停下來確認
```

預設 `--pattern boustrophedon --spacing 0.4 --min-clearance 0.40`。
覆蓋掃描線預設 `--spacing 0.6`（新地圖約 5 條／18 個導航點）；導航抽稀另預設
`--goal-spacing 0.4 --min-step 1.0 --settle 0.5`、
`--min-goal-distance 0.15 --max-yaw-step-deg 145`、`--goal-timeout 30`，都可在
wrapper 以同名旗標覆寫。70° 強制補點在尖角曾產生 1～5 cm 的連續目標，反而增加
停車轉向；現在預設不拆角，並以 0.15 m 最小目標距離作硬性拒絕條件。
`--min-clearance` 改了的話 `make_bridge_traj` 和 `drive_waypoints` 兩邊會同步帶下去
（腳本已經串好），不要只改一邊 —— 規劃貼著 0.35 m 邊界走、檢查用 0.40 m 會砍掉一半目標。

跑真的那一趟之前確認：機器人在一般模式、已載入要對齊的那張圖、路上沒有人。
3D 產物仍在 `/home/acm/fungi/outputs_malong/<run名>/`（container 以 root 寫入，
不受本帳號權限影響）。

## 2026-09-11 實測（地圖「自動建圖測試0911」346x240 @ 0.025）

`--plan-only` 跑通：7 條掃描線、37 個導航目標、路徑長 14.5 m、最大單步轉向 36.6°。
產物 `kachaka_2d_auto0911.*`、`traj_test0911.csv`、`goals_test0911.csv`，
預覽 `~/viz_charles/preview_goals_test0911.png`。

⚠️ **這張圖的東半邊掃不到**：free 17.30 m²，0.40 m 侵蝕後只剩 5.70 m²（連通 5.56）。
分帶統計：x[1,3) free 7.39 → 侵蝕後 4.40 m²；x[3,5) free 4.52 → **0.13 m²**；
x[5,7) free 1.52 → **0.08 m²**。東側的 free 與 unknown（234）交錯成細條，一侵蝕就沒了。
淨空降到 0.30 m 也只有 6.74 m² 連通。要讓 3D 圖涵蓋整個房間，得回 Kachaka App
把東半邊重掃一遍再匯出，不是調參數能解決的。

---

## 目前狀態（2026-09-11 收工）／下一步

**已驗證（實機）**
- 完整流程跑通一趟：`ec129_0911_bridge`（地圖「自動建圖測試0911」，910 幀、57 submaps、53 loops，
  deploy 15/15，產物 708 MB 在 `/home/acm/fungi/outputs_malong/ec129_0911_bridge/`）。
- `preflight.sh` 10 項全部實測；`arm_cam_tune.sh` 的 status/snap/live 實測；
  收合姿態 joint5=0.35 實機確認鏡頭乾淨（存證 `~/viz_charles/armcheck_final_j5_0.35_from_stream.jpg`）。
- pose2d log `pose2d_20260911.csv`（39393 列）收集期間單一來源、無重複時間戳，可直接用。

**未驗證（只有離線推估）**
- 新預設加 70° 轉角保護後，EC129 舊軌跡離線結果為 27 goals、14.3 m、最大 69.9°；
  原本 19 goals 的時間估算已失效，仍待下一趟真機驗證。
- 新收合姿態下整趟 3D 點雲是否真的沒有白色拖線（只確認了單幀乾淨）。

**下一步**

1. alignment（要 acm 帳號；你的家目錄 acm 進不去）：
   ```bash
   mkdir -p /home/acm/robotic/handoff_0911
   cp ~/workspaces/kachaka_mapping/pose2d_20260911.csv \
      ~/workspaces/kachaka_mapping/kachaka_2d_ec129_0911.{png,yaml} /home/acm/robotic/handoff_0911/
   # 請 acm 跑：
   cd /home/acm/fungi && POSE_LOG=/home/acm/robotic/handoff_0911/pose2d_20260911.csv \
     MAP_PNG=/home/acm/robotic/handoff_0911/kachaka_2d_ec129_0911.png \
     MAP_YAML=/home/acm/robotic/handoff_0911/kachaka_2d_ec129_0911.yaml \
     ./align.sh ec129_0911_bridge 20260911 ec129_0911     # rmse_all > 0.3 m 要查
   ```
2. 用新參數再跑一趟驗證（跑前 `./arm_cam_tune.sh live-stop`，T1 若還熱可省 2.4 分冷啟動）：
   ```bash
   cd ~/workspaces/kachaka_mapping
   ./run_bridge_oneshot.sh --name ec129_0911_b2 --venue ec129_0911 --reuse-map
   ```
3. 要完整房間的 3D 圖：回 Kachaka App 把東半邊重掃、存檔，**換新的 venue 名**再從 `--plan-only` 開始。

**要請 acm 做的**
- 同步 `MAPPING_STOW_POSE`（joint5 0.35）到 `/home/acm/fungi/run_slam_oneshot.sh:240` 與 `build_new_map.sh:186`。
- `[MAP-08]`：部署樹 `ma_slam_semantic` 三個檔還是 09-02 的壞版，現在靠 `~/ops` 影子套件撐著。
- `[INF-15]`（site ops issue tracker）併回正本。
