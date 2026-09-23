# 這條流程踩過的所有坑（2026-09-11 彙整）

一趟建圖要動到 fungi 的腳本、acm 的部署樹、mm_container 的手臂、Kachaka 本體，
任何一環壞掉都要等到跑起來才知道。這裡把**每一個實際踩過的失敗**記下來：
症狀長什麼樣、真因是什麼、修在哪、以及**開跑前怎麼先查出來**。

開跑前一律先跑這支，全部通過再碰機器人：

```bash
cd ~/workspaces/kachaka_mapping
./preflight.sh --name <這趟的 run 名>
```

⚠️ **一趟正在跑的時候不要編輯 `run_bridge_oneshot.sh`** —— bash 是邊跑邊讀檔案的，
改動會讓執行中的那份從錯的位元組偏移繼續讀，行為無法預期。要改等它跑完。

## 2026-09-14 遠端修正摘要

- `--min-clearance` 現在同時傳給路徑生成器與 waypoint 安全檢查，不再出現兩邊門檻不同。
- waypoint 最終朝向預設限制為相鄰 `70°`；必要時會從原始軌跡補回轉角點。EC129 舊軌跡
  離線重算為 27 goals、14.3 m、最大 69.9°（原抽稀結果最大 160.1°）。
- 每個 `move_to_pose` 預設 30 秒逾時（2026-09-16 依小房間實測由 120 降低）；逾時會呼叫 `cancel_command()`，確認命令停止後
  跳下一點。Ctrl+C／API 例外仍會取消並中止整趟。
- wrapper 在錯誤、SIGINT、SIGTERM 時會收 pose logger；若本趟啟動 SLAM，也會送 Enter 收尾。
- pose log 改成 `pose2d_<run名>.csv`，避免同日多趟混寫。
- `--reuse-map` 會比對 YAML 的 `map_id`（舊檔退回比對 `map_name`）和機器人目前地圖；不符就停止。
- `--plan-only` 的 preflight 只檢查本地規劃依賴，不再因 SLAM、手臂或 logger 狀態被擋。
- 清除可重建 Docker cache/dangling images 後，可用空間 25GB → 32GB；完整 preflight 已通過。

---

## A. 複製 fungi 原版就會踩的（都已修在本地副本）

| # | 停在哪 | 症狀 | 真因 | 已修 |
|---|---|---|---|---|
| 1 | 2/7 匯出地圖 | `Permission denied` | 原版 `FUNGI=/home/acm/fungi` 寫死，產物全寫那裡，而該目錄是 `acm:acm drwxrwxr-x`，本帳號不可寫。連 `--plan-only` 都過不了 | 拆成 `WORK`（本目錄放產物）/ `TOOLS`（fungi 唯讀） |
| 2 | 1/7、3/7 | `ModuleNotFoundError: kachaka_api` | `make_bridge_traj.py:373`、`drive_waypoints.py:175` 和腳本裡**兩段內嵌 python** 都寫死 `~/robotic/...`（acm 專屬路徑） | 改成先找 `~/robotic` 再退回 `/home/acm/robotic` |
| 3 | 4/7 logger | `logger 沒有在寫（0 → 0 行）` | 同一個 import 問題，但出在 `log_kachaka_pose2d.py:26`，那支留在 fungi 不能改。logger 靜靜死掉、每 5 秒重試，外層 12 秒後判定失敗 | 呼叫 keepalive 時用 `PYTHONPATH=$KAPI` 從外面補 |
| 4 | 4/7 前 | `line 198: …/pose2d_*.csv: No such file or directory` | `wc -l < "$POSE_LOG"` 在檔案還沒建立時的重導向錯誤。**純雜訊**，不影響執行，但會讓人以為是它害的 | 改成先 `[ -f ]` 再 `wc` |
| 5 | 5/7「卡在 T1」 | 外層一路等到 900 s 逾時；T1 視窗其實早就掉回 shell | fungi 原版的 `run_slam_oneshot.sh` 啟動的 T1 會 `ModuleNotFoundError: ma_slam_semantic.dynamic_tracker` —— 場地部署樹 2026-09-02 的補丁沒整合完（site ops issue `[MAP-08]`） | 改叫 `~/ops/run_slam_oneshot.sh`（T1 走 `run_t1_server_fixed.sh` 的影子套件） |
| 6 | 產出預覽圖 | 圖跑到 `~/viz`（本帳號沒這目錄） | 原版預設 `Path.home()/"viz"` | 改 `~/viz_charles` |
| 7 | 7/7 alignment | `align.sh` 跑不動 | 它是 `FUNGI=~/fungi`（家目錄相對）且要寫進 fungi；而且**你的家目錄是 `drwxr-x---`，acm 進不去**，pose2d/2D 圖交不過去 | 只印指令 + 警告；交接走 `/home/acm/robotic/handoff_*`（兩邊都可讀寫） |

## B. 環境類：每次開跑前要查（preflight 會自動查）

| # | 症狀 | 真因 | 檢查 / 修法 |
|---|---|---|---|
| 8 | 5/7「等手臂回報 /joint_states_feedback — 逾時 120s」，訊息把你導向 CAN／電源 | **DDS participant 被佔滿**。`mm_container` 是 `Net=host`，domain 0 額度全機共用；`ros2 topic echo --once` 在沒有 publisher 時永遠不退出，每支佔一個 index。手臂其實完全正常 | preflight #6/#7。修：`docker exec mm_container pkill -f "ros2 topic"`。**新寫的 [INF-15]** |
| 9 | 重跑之後 pose2d csv 時間戳成對重複 | 上一趟 `die` 之後 logger 沒被回收（`die` 只 exit，不 kill `LOGGER_PID`），再跑一趟就有兩支寫同一個檔 | preflight #8（會分辨「進行中的 run」）。修：`pkill -f log_kachaka_pose2d` |
| 10 | run 到一半沒空間 | `run_slam_oneshot` 門檻 30 G，一趟吃數 GB；這台長期 95% 滿 | preflight #4 |
| 11 | container 不存在 | `zealous_agnesi` / `gateway_slam` Exited 沒關係（會自動 `docker start`）；**manual 模式不需要 `ros2_bridge`**，它 Exited 是正常的 | preflight #5 |
| 12 | 深度品質變差 / 收不到畫面 | gateway 綁死序號 `<REALSENSE_SERIAL>`，插到 USB2 孔會變 USB 2.1 | `run_slam_oneshot` 自己會擋；人要確認插藍色孔 |
| 13 | 手臂**真的**壞掉 | USB-CAN 轉接器 stall（`err=-32`），症狀跟斷電一樣 | 分辨法看 `ip -s -d link show can_piper` 的 RX 有沒有在漲（正常約 3000 幀/s）。見 `[ARM-15]` |
| 14 | 整趟座標系錯 | App 載入的不是要對齊的那張圖 | preflight #9 會印當下地圖名；1/7 也會再確認一次 |
| 15a | 收尾等超過 20 分鐘後外層自己結束、印警告 | 外層的收尾等待上限是 1200 s。**這是設計，不是失敗** —— tmux 裡的 SLAM 會繼續把 deploy 做完 | `ls outputs_malong/<run>/deploy \| wc -l` 等到 15；`curl -s localhost:3636/status` 看 queue_size |
| 15b | 殺不掉的 pose2d logger | `kill $LOGGER_PID` 殺的是 `logger_keepalive.sh` 外殼，真正在寫檔的 `log_kachaka_pose2d.py` 是它的**子程序**，會變孤兒繼續跑。2026-09-11 兩次都留下孤兒 | 已修：`stop_logger()` 連子程序一起 `pkill` |
| 15 | 上一趟成功的資料被蓋掉 | 同名 `--name` 重跑時，`run_slam_oneshot` 的「要覆寫嗎？」被我們傳下去的 `-y` 自動答應了 | preflight #10（`--name` 帶上就會查） |

### ARM-15 現場復原：USB-CAN stall（2026-09-09、2026-09-15）

典型症狀是 `can_piper` 顯示 UP，但 RX/TX 都是 0；Piper pane 出現
`SendCanMessage(SEND_MESSAGE_FAILED (100017))`、`Automatic enable timeout`、
`can_piper is loss`。2026-09-15 的 kernel log 另有
`gs_usb 1-4.1:1.0 can_piper: usb xmit fail 9...2`。這代表 USB-CAN／gs_usb
沒有真的把訊框送到 transceiver，不能把「介面是 UP」當成健康。

先純監聽，不要先送手臂動作：

```bash
ip -s -d link show can_piper
timeout 6 candump can_piper
docker exec mm_container dmesg | grep -E 'gs_usb|err=-32|usb xmit fail' | tail
```

健康的手臂會持續送出 `0x2A1`～`0x2A8`。RX=0、錯誤計數也不增加，
或 kernel 有 `err=-32`／`usb xmit fail` 時，照以下順序復原：

1. 實體拔掉 USB-CAN，改插另一個 USB 孔，讓轉接器真正斷電並重新列舉。
2. 不要用 sysfs `authorized=0/1`；2026-09-09 實測無效且可能變成
   `RTNETLINK answers: Broken pipe`。
3. 換孔後不要只跑 `docker restart mm_container`。container 的
   `CAN_USB_ADDR` 是建立時寫死的，換孔後會仍拿舊 bus 位址。
4. 現查 `driver=gs_usb` 的新 bus 位址，並在 tmux pane 0 **同一行**
   啟動 CAN 和 Piper：

```bash
IFACE=$(ip -br link show type can | awk '{print $1}' | while read -r i; do
  [ "$(ethtool -i "$i" 2>/dev/null | awk -F': ' '/^driver:/{print $2}')" = gs_usb ] && echo "$i"
done | head -1)
ADDR=$(ethtool -i "$IFACE" | awk -F': ' '/^bus-info:/{print $2}')
docker exec mm_container ip link set "$IFACE" down
docker exec mm_container tmux send-keys -t mm:0.0 \
  "bash can_activate.sh can_piper 1000000 $ADDR && ros2 launch piper start_single_piper.launch.py" C-m
```

成功案例：2026-09-09 從 `1-2.2` 換到 `1-4.1` 後恢復。驗收必須三項全過：

- `candump can_piper` 持續收到 `0x2A1`～`0x2A8`。
- Piper pane 顯示 `Enable status:True`。
- `ros2 topic hz /joint_states_feedback` 約 200 Hz。

2026-09-15 再次發生於 `1-4.1`：重啟 `mm_container` 兩次及軟體重建
`can_piper`（1 Mbps）均無效，底盤未啟動。下一步仍是實體拔插／換孔，
之後依上述「現查位址」方式恢復，不能沿用舊 container env。

### 2026-09-15 現場實跑：房間配置已變，舊 2D 地圖不能再沿用

`onsite_run_20260915` 使用 App 地圖「自動建圖」
（ID `d4cc061d`）與 `--reuse-map`。20 個目標、規劃 13.4 m、最短目標
間距 0.37 m；目標 0～7 成功，目標 8
`(2.777, -0.179, yaw=-99.6°)` 在 120 秒逾時後被安全取消。

- 舊 2D 圖計算該目標淨空 0.503 m，停止位置淨空 0.556 m，所以不是 generator
  把點放進舊圖障礙物。
- 停止位置為 `(2.699, -0.591, yaw=78.8°)`，Kachaka 沒有現存 error；
  last result 的 10001 是 timeout 後取消留下的結果。
- 前鏡頭 `~/viz_charles/kachaka_front_after_goal8_timeout.jpg` 顯示正前方已有
  桌腳、桌面與箱子；使用者確認房間配置已變，舊 App 地圖和現況有落差。
- 結論：不可跳過該點硬跑，也不可再用同一 venue 的 `--reuse-map`。先用 Kachaka
  App 重掃並另存新地圖，載入新地圖後換一個新的 `--venue` 匯出／plan-only／看圖。

本趟保留下來的局部資料：226 RGB-D 幀、15 submaps、9/10 loops、deploy 15/15；
導航約 276 秒，收尾 115 秒。真彩點雲沒有白色支架沿相機路徑形成的拖線，
所以 joint5=0.35 的收合姿態通過局部驗證；但路徑只完成前半，**完整點雲覆蓋未驗收**。
MAP-09 仍未改善：快轉銳利度中位 48、sharp<20 有 39 幀、`|ω|>0.25`
占 61.1%。點雲疊軌跡圖：
`~/viz_charles/onsite_run_20260915_pointcloud_with_camera_path.png`。

### 2026-09-16 載 S01 與空機 A/B 測試：路徑正常，問題跟負載高度相關

新地圖「自動建圖0915」（ID `7ccec1a8`）上的稀疏路徑有 20 個點、約 9.9 m。
載 S01 跑 `onsite_remap_sparse_run_20260916` 時，前 18 個已完成／判定的目標中
13 個成功、5 個各耗滿舊的 120 秒後取消並跳點；第 19 個移動目標依使用者要求人工停止。
現場目視沒有障礙物。卸下 S01 後，以同一張地圖、同一套抽稀／抵達 yaw 邏輯直接跑
`empty_base_test_20260916`，**20/20 全成功、0 timeout、約 2.0 分鐘**，多數目標只需
4～7 秒。因此：

- 2D 地圖、路徑幾何及「逾時後取消再跳點」邏輯已通過空機驗證。
- 失敗和載 S01 高度相關；目前證據無法再區分 Kachaka 載架模式的速度／控制限制、
  輪子打滑、腳輪／架體阻力或純粹馬達扭力不足。下次應先查 S01 輪子、地面摩擦與
  shelf speed mode，再做載架 30 秒 timeout 的短測。
- 小房間預設 `goal timeout` 已由 120 秒降為 **30 秒**；這次載架正式 run 使用的仍是
  舊 120 秒，30 秒設定只在其後的空機測試通過，尚未完成載架驗收。

人工停止仍保留可用點雲：654 幀、41 submaps、38/38 loops、deploy 15/15。
與 `onsite_remap_run3_20260915` 做 4-DoF 配準（voxel 0.15 m）得 fitness 0.758、
RMSE 8.6 cm；10 cm BEV 主要覆蓋比 run3 約多 3～4%。地面重算為 tilt 2.55°、
inlier RMSE 1.62 cm，和 run3 的 2.74°／1.53 cm 接近。影像方面 `|omega|>0.25`
降至 44.0%，但 sharp<20 為 67/653（10.3%），略差於 run3 的 39/407（9.6%）；
長時間載架卡住多收了無用幀，故此 run 可用於幾何參考，但不應取代 run3 當首選基準。
疊圖：`~/viz_charles/onsite_remap_sparse_vs_run3_20260916_overlay.png`。

## C. 已知但還沒踩到

| # | 會怎麼炸 | 說明 |
|---|---|---|
| 16 | T1 起得來、**走到第一個 submap 才炸** | `instance_map.py` 的 `Obs.from_mask() got an unexpected keyword argument 'is_dynamic'`（同一份 09-02 補丁）。影子套件已含修好版，所以現在不會遇到；哪天沒走 `~/ops` 修正版就會炸 |
| 17 | 3D 圖只有房間一小塊 | 0.40 m 淨空侵蝕通常吃掉 6–7 成 free space。「自動建圖測試0911」那張：17.30 → 5.70 m²，東半邊幾乎全滅。**規劃完一定要看預覽圖** |
| 18 | alignment 交不出去 | 見 #7，先 `cp` 到 `/home/acm/robotic/handoff_*` 再請 acm 跑 |
| 19 | 路徑規劃在錯的地圖上 | `--reuse-map` 會沿用既有的 `kachaka_2d_<venue>.*`。換了新的 App 地圖就要**換 venue 名**，別沿用 |

| 20 | 3D 點雲裡有白色拖線 | **手臂上的白色支架從畫面底部（約最下 5–8%）入鏡**，跟相機剛性連動，整趟被當成近處物體重建、拖成沿軌跡的白線。收合姿態是 8/18 定的、當時「拍照確認鏡頭乾淨」，現在不乾淨 → 相機或支架的實體位置動過。**開跑前 `./arm_cam_tune.sh snap` 看一眼**，支架露出來就調 joint5 再填回 `MAPPING_STOW_POSE`。**✅ 2026-09-11 已解：joint5 0.195 → 0.35**（`~/ops/run_slam_oneshot.sh` 已改；fungi 原版與 `build_new_map.sh:186` 仍舊值，要請 acm 同步）|

## D. 要 acm 出手才能根治的

- **`[MAP-08]`**：部署樹的 `hooks.py` / `export.py` / `instance_map.py` 還停在 09-02 那份沒整合完的
  dynamic-object 補丁。要嘛把 `dynamic_tracker.py` 真的寫出來，要嘛 revert 回 08-21 版。
  在那之前每一趟都得靠 `~/ops` 的影子套件。
- **`[INF-11]`**：ma-long 部署樹沒進版控、沒備份，所以上面那種「某天突然壞掉」還會再發生。
- 另一個選項：把 `~/ops/ma-long-fix/ma_slam_semantic/` 直接覆蓋回部署樹（需要 acm 寫入權），
  之後就不必走影子套件 —— preflight #3 會在部署樹修好時告訴你。

---

# E. 為什麼一趟要 35 分鐘（2026-09-11 實測拆解）

`ec129_0911_bridge` 這趟的時間分布（14.5 m 的規劃路徑）：

| 階段 | 時間 | 說明 |
|---|---|---|
| T1 冷啟動 | 2.4 min | 20:48:53 起，第一幀 20:51:18。載 14 G 權重，每趟都要付 |
| 駕駛 | **17.8 min** | 20:51:18 → 21:09:05，收 910 幀 |
| 收尾 | **~26 min** | 佇列積壓 366 幀要跑完才能匯出（21:09 → 21:35 才排空）|
| 合計 | ~48 min | （不含今天 4 次失敗重跑） |

## 駕駛那 17.8 分鐘實際在做什麼

```
規劃路徑 14.5 m  →  實際走了 31.2 m（2.2 倍，來回修正）
規劃總轉角 1091° →  實際轉了 7486°（6.9 倍）
平均速度 0.029 m/s（Kachaka 巡航 0.3 m/s，純開這條路只要 0.8 分鐘）
時間分布：前進 19%｜原地旋轉 27%｜完全靜止 27%｜低速混合 27%
完全靜止 160 段（中位 1.3 s、最長 11 s），平均每個目標停 4.3 次
```

**主因：導航目標太密。** 37 個目標、中位間距 0.27 m，而 `move_to_pose` 是一次完整導航動作：
停 → 規劃 → 轉向對準 → 開過去 → **再轉到指令的最終朝向**。目標隔 0.27 m 時，幾公分的
定位誤差就讓「該朝哪開」大幅改變，於是每個目標都變成「轉一下、挪幾公分、再轉一下」。
載著 S01 手臂架時轉速受限，每個週期都更久。

另外**迴圈閉合也被原地旋轉放大**：一直在同一點附近轉，loop detector 不斷觸發
（這趟 `loops=51`），每次 loop event 都要讓 instances re-key，收尾又更久。

**收尾 25 分鐘是駕駛時間的連鎖後果**：收幀速率 0.85 fps × 17.8 min = 910 幀，而語意管線
只跑得動約 0.5 fps（收集期間）/ 1 fps（停止收幀後），差額累積成 366 幀佇列。
**原地旋轉也會觸發 gate 收幀** —— 那些幀對幾何沒有視差貢獻，卻照樣要付處理成本。
駕駛時間砍半，幀數和收尾時間就同比例砍半。

## 之後怎麼解（依效益排序）

1. **抽稀參數** ✅ 已做並於 2026-09-16 再調整：`run_bridge_oneshot.sh` 預設為
   `--spacing 0.6 --goal-spacing 0.4 --min-step 1.0 --settle 0.5`。新地圖目前位置實測
   7 → 5 條掃描線、32 → 18 goals、路徑 9.8 m、最短點距 0.31 m、raw 最大轉角 26.4°；
   run3 即使只走約一半也已收 407 幀、26 submaps、20 loops，支持減少停車目標。2026-09-14
   加入 70° 轉角上限後會在 U 迴轉補點；EC129 舊軌跡的最終結果是 27 goals、最大 69.9°，
   不再下發原本最大 160° 的單步轉向。因此
   **不要一次跳到 1.0 m**（中位 62°、最大 178° = 原地掉頭）。
2. **路徑形狀** ✅ 已採用：`--spacing 0.4 → 0.6`，掃描線從 7 條減到 4–5 條，直接砍掉幾個 U 迴轉
   （1091° 總轉角的來源）。覆蓋會略降，但這張圖的可用區本來就只有 5.7 m²。
3. **更激進的做法**：只給掃描線**端點**當目標（約 10 個），中間交給 Kachaka 自己的規劃器
   （它本來就有避障）。缺點是實際走法不完全等於規劃的蛇行，覆蓋較難預測。
4. **`--settle`**：預設每點停 1 s，配合 1. 之後可降到 0.3。
5. **先跑 `preflight.sh`** ✅ 已接進 1/7 自動執行：今天真正的時間大頭是 4 次失敗重跑，不是這 35 分鐘。
6. **T1 保溫**：連續測試時不要在趟與趟之間關掉 SLAM container，可省每趟 2.4 分鐘的冷啟動。

原本「19 goals、總計約 15 分鐘」的估算已被 70° 轉角保護改變；目前離線是 27 goals，
實際駕駛、幀數與收尾時間要等下一趟真機資料再填，不能沿用舊估算。
