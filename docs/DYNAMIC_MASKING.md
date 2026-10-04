# 動態物件遮罩 + 即時 3D SLAM / Dynamic-object masking with live 3D SLAM

Branch `gaurav/dynamic-masking` · Gaurav · 2026-09-28, updated 2026-10-05

這個分支把 Gaurav 的動態物件遮罩方法接進即時 3D SLAM：人一律移除；其他可移動物件（筆電、背包、瓶子…）只有在移動或被拿著時才移除，靜止的物件保留在地圖中。
This branch adds Gaurav's dynamic-object masking to the live 3D SLAM run by `run_bridge_oneshot.sh`. People are always removed. Other movable objects (laptop, bag, bottle…) are removed only while they move or are carried; static objects stay in the map. Masked pixels never become map points.

> **安全 / Safety:** the full run moves the robot. Plan first (`--plan-only`), check the preview, keep the path clear and a person at the emergency stop.

---

## 1. 內容 / What is in this branch

| Path | Purpose |
|---|---|
| `dynamic_masking/` | The method: `chunk_fusion_masker.py` (fusion + object policy + geo gate), `dynamic_fusion.py` (optical-flow motion residual), `dynamic_object_mask.py` (YOLOv9e-seg), `flowseek_flow.py`, `motion_compensation.py` |
| `slam_integration/fusion_solver.py` | Hooks the masker into the SLAM server between inference and point creation; live mask view |
| `slam_integration/*.patch` | Small patches for the team's SLAM server (`server_api.py`, `viz.py`) and camera gateway (`gateway_node_robot.py`, `realsense.py`) |
| `slam_integration/install.sh` | Links `fusion_solver.py` into a server checkout and applies the patches (idempotent) |
| `slam_tools/` | Run, live-view, control and result tools (all machine values come from `.env`) |

### How the method decides / 方法

Per chunk of 16 frames:

1. **Semantic (YOLOv9e-seg):** person confidence ≥ 0.15 (others ≥ 0.25). A person is always removed. Any other movable-class object is removed only if it is **moving** (≥ 30 % of its pixels have a flow residual above the frame threshold) or **carried** (touches a person, 10 px).
2. **Motion (FlowSeek optical flow vs. camera-pose flow):** adaptive threshold `median + 3·MAD`, checked against both neighbour frames. **Geo gate (new, default `anchor`):** a motion blob is kept only if it touches a movable-class detection (30 px). On the 22 Sept lab capture this cut person-free frames with more than 1 % removed from 59/296 to 1/296, with the people removal unchanged. Trade-off: an object YOLO cannot label is no longer removed by motion alone. `DYNAMIC_GEO_GATE=none` gives the old behaviour.
3. **Bridge:** fills up to 3 frames without a YOLO detection when the same chunk has a detection before and after the gap (one side only is allowed for a person entering or leaving through the image border).
4. **Person cleanup (2026-10-01):** person masks grow by 5 px; inside the person's detection box, pixels at the person's depth are added (legs behind a stool), and the box is extended 0.6 × its height downwards (legs under furniture); depth-jump ("flying") pixels within 12 px of the final mask are removed. The overlap frame shared by two submaps gets the later chunk's mask too.
5. **3D carving (2026-10-01):** after the map is written, every point is projected into every frame. A point that frames only ever see on removed pixels (and on at most one kept pixel, its own) is dropped and saved to `carved_pcd.ply`. This removes points that other frames' predicted depth puts on a person, which no 2D mask can stop.

All of this is on by default. Switches: put them in `.env` (every `slam_tools` script loads it, also when `run_bridge_oneshot.sh` starts the server in tmux):

| Variable | Default | Before 2026-10-01 |
|---|---|---|
| `DYNAMIC_GEO_GATE` | `anchor` | `none` |
| `DYNAMIC_PERSON_DILATE_PX` | `5` | `0` |
| `DYNAMIC_PERSON_BOX_FILL` | `1` | `0` |
| `DYNAMIC_BRIDGE_MAX_GAP` | `3` | `1` |
| `DYNAMIC_BOX_FILL_DOWN` | `0.6` | `0` |
| `DYNAMIC_EDGE_RING_PX` | `12` | `0` |
| `DYNAMIC_CARVE` | `1` | `0` |
| `SEMANTIC` | unset (`--no_deploy`) | — set `1` for semantic instances + deploy files (~20 GB more GPU memory) |

Trade-off: when a person leans on a stool or box, the per-frame mask also covers that object; it stays in the map through the other views.

---

## 2. 一次性安裝 / One-time setup

Assumes the team's lab machine: Docker, the SLAM image (`slam_node:malong`), the gateway image (`gateway_slam_node:dev`), the `fungi` workspace and the ma-long weights already exist.

```bash
# 1. get the branch
git clone https://github.com/Gauravmeena1/kachaka_mapping.git
cd kachaka_mapping
git switch gaurav/dynamic-masking

# 2. settings: copy and edit every value (robot IP, camera serial, paths, container names)
cp .env.example .env
nano .env
```

`.env` keys added by this branch (see `.env.example`):

| Key | Meaning |
|---|---|
| `KACHAKA_TOOLS_DIR` | Host workspace mounted at `/fungi` (gateway code, run outputs) |
| `KACHAKA_SLAM_CONTAINER` | Camera → ROS 2 gateway container |
| `KACHAKA_AI_CONTAINER` | SLAM server + masking container (GPU) |
| `KACHAKA_SERVER_DIR_CT` | SLAM server repo path **inside** the AI container |
| `KACHAKA_CODE_DIR` / `KACHAKA_CODE_DIR_CT` | Host folder that holds both `ma-long-server/` and `kachaka_mapping/`, and its path in the container. Both repos must sit side by side in it (the installed link is relative). |
| `KACHAKA_WEIGHTS_DIR` | ma-long tree with `src/weights` and `vendor/weights` |
| `KACHAKA_AI_IMAGE` / `KACHAKA_GATEWAY_IMAGE` | Images used by `make_containers.sh` |
| `KACHAKA_CAMERA_SERIAL` | RealSense used for mapping |
| `KACHAKA_SLAM_RUNNER` | `<repo>/slam_tools/run_slam_bridge.sh` |
| `KACHAKA_T1_RUNNER` | `<repo>/slam_tools/run_t1_server.sh` |
| `KACHAKA_ARM_OPTIONAL` | `1` = missing arm node is a preflight warning (mapping never moves the arm) |

```bash
# 3. model weights (not in git): yolov9e-seg.pt (117 MB) and thirdparty/ (FlowSeek code + weights,
#    645 MB). On the lab machine, copy both from Gaurav's method folder (ask Gaurav for the path):
cp <method-folder>/yolov9e-seg.pt dynamic_masking/
cp -r <method-folder>/thirdparty dynamic_masking/
#    (elsewhere: FlowSeek code and weights from the project page https://flowseek25.github.io/ ;
#     flowseek_T_TartanCT_TSKH.pth and depth_anything_v2_vits.pth go in
#     dynamic_masking/thirdparty/flowseek/weights/)

# 4. load .env into this shell (the commands below use its values)
set -a; . ./.env; set +a

# 5. hook the masker into your SLAM server checkout and patch the gateway workspace
slam_integration/install.sh /path/to/ma-long-server  "$KACHAKA_TOOLS_DIR"

# 6. containers (only if they do not exist yet; running containers are never touched)
slam_tools/make_containers.sh
```

Check: `slam_integration/install.sh` prints four ✓ lines; `make_containers.sh` prints the mounts.

**Every new terminal:** run `cd kachaka_mapping && set -a && . ./.env && set +a` first, so `$KACHAKA_…` values below are defined.

---

## 3. 每次開機後 / After every reboot

```bash
cd kachaka_mapping && set -a && . ./.env && set +a
docker start "$KACHAKA_AI_CONTAINER" "$KACHAKA_SLAM_CONTAINER"
PY=$KACHAKA_PYTHON

$PY slam_tools/kachaka_ctl.py status      # map, pose, battery, errors (read-only)
slam_tools/cam_check.sh                   # camera serial + USB 3.x, saves camcheck.jpg in the workspace
./preflight.sh --name RUN                 # must end with 全部通過 (all passed)
```

`camcheck.jpg` must show the room, not a wall at 0.5 m, and nothing (arm, bracket) at the bottom edge. The camera must report `USB 3.x`.

---

## 4. 先離線驗證方法 / Test the method offline first (no robot)

Replays a recording through exactly the same server code path as a live run.

```bash
# window 1: SLAM server with masking (wait for "models resident", ~1 min)
tmux new -s t1
slam_tools/run_t1_server.sh --mode rgb+depth+intr --backend ma

# window 2: replay (the recording must be in $KACHAKA_TOOLS_DIR/replay/<name>/{rgb,depth})
python3 slam_tools/replay_ab.py mapping_20260922_01 offline_test1

# bird's-eye view: before | after | removed in red
docker exec "$KACHAKA_AI_CONTAINER" \
  python "$KACHAKA_CODE_DIR_CT/kachaka_mapping/slam_tools/bev_compare.py" /fungi/outputs_malong/offline_test1
```

Expected on `mapping_20260922_01` (389 frames): mean removed ≈ 2.2 % per frame, motion-only ≈ 0.1 %.

---

## 5. 即時建圖 / Live automatic mapping run

```bash
# 1. make the 2D map in the Kachaka App (start from the dock), save it as VENUE, load it.

# 2. drive the robot about 1 m off the dock into open space (planning from the dock fails)
$PY slam_tools/kachaka_ctl.py goto X Y YAW          # ROBOT MOVES — or move it in the App

# 3. plan only (answer y to the map question), then look at the preview
./run_bridge_oneshot.sh --name RUN --venue VENUE --goal-timeout 60 --plan-only
code artifacts/previews/preview_goals_RUN.png

# 4. full run in tmux — answer y twice (map correct? / route OK, robot moves?)
tmux new -s mapping
./run_bridge_oneshot.sh --name RUN --venue VENUE --reuse-map --goal-timeout 60
```

Stages: 4/7 pose logger → 5/7 SLAM + masking start (1–3 min) → 6/7 robot drives (a goal over 60 s is skipped) → 6.5 recording finishes and the map is saved → 7/7 alignment commands.
When `$KACHAKA_TOOLS_DIR/outputs_malong/RUN/static_only_pcd.ply` exists, the map is saved: press **Ctrl+C** in the `mapping` window instead of waiting (the script otherwise waits up to 20 min for semantic export files that `--no_deploy` never creates).

---

## 6. 即時觀看 / Watching live

```bash
python3 slam_tools/live_view.py "$KACHAKA_TOOLS_DIR/outputs_malong/RUN" --mask --port 8092   # camera | removed pixels
python3 slam_tools/live_view.py "$KACHAKA_TOOLS_DIR/outputs_malong/RUN" --port 8093          # raw camera
```

| View | URL |
|---|---|
| 3D map (removed points in red) + camera panel | `http://HOST:9090/?url=rerun%2Bhttp%3A%2F%2FHOST%3A9876%2Fproxy` |
| Camera \| removed pixels | `http://HOST:8092/` |
| Raw camera | `http://HOST:8093/` |

Panel colours: **red** = semantic (YOLO), **yellow** = motion only, **blue** = bridge. From a laptop over VS Code Remote-SSH, use `localhost` and forward ports 8092, 8093, 9090 **and 9876** (the 3D viewer's data port).

---

## 7. 停止與控制 / Stopping and robot control

| Goal | Command |
|---|---|
| Stop the run early (cancels driving, stops the logger, saves the map so far) | Ctrl+C in the `mapping` window |
| Stop the robot | `$PY slam_tools/kachaka_ctl.py cancel` |
| Return to dock (robot moves) | `$PY slam_tools/kachaka_ctl.py home` |
| Save a recording by hand | `curl -s -m 900 -X POST localhost:3636/stop` |
| Clean up after a crash (saves any session, closes live views) | `slam_tools/stop_mapping.sh` |
| Recording with the robot parked (people walk) | `COLLECT_EXTRA='--keyframe-disparity 0 --blur-k 15' slam_tools/run_slam_oneshot.sh manual --name RUN --duration 180 -y` |

Never kill the SLAM server while it records or saves: the map in memory is lost.

---

## 8. 結果 / Results

| Output | Path |
|---|---|
| 2D map | `artifacts/runs/kachaka_2d_VENUE.{png,yaml}` |
| Robot pose log, route | `artifacts/runs/pose2d_RUN.csv`, `traj_RUN.csv`, `goals_RUN.csv` |
| 3D map, dynamic objects removed | `$KACHAKA_TOOLS_DIR/outputs_malong/RUN/static_only_pcd.ply` |
| 3D map before removal / removed points only | `all_points_pcd.ply` / `dynamic_pcd.ply` (same folder) |
| Points removed by 3D carving | `carved_pcd.ply` (same folder) |
| Semantic map + deploy files (only with `SEMANTIC=1`) | `combined_pcd.ply` and the deploy folder (same folder) |
| Every frame, coloured by channel + per-frame % | `mask_viz/*.jpg`, `mask_viz/removed.csv` |
| Bird's-eye before / after | `bev_compare.png` (made by `bev_compare.py`) |

Open the `.ply` files in CloudCompare or MeshLab. The three point files are sampled separately, so compare them visually, not by point count.

---

## 9. 常見問題 / Troubleshooting

| Symptom | Fix |
|---|---|
| Session ends after 90 s: “no intrinsics arrived” | The gateway's tuned capture needs `$KACHAKA_TOOLS_DIR/semantic_slam/src/ma-long/src/ma_slam_stream/realsense.py`; T2 must print “intrinsics sent” |
| “RealSense failed to start: Couldn't resolve requests” | Camera on USB 2; use a blue USB 3 port (a 640x480 fallback exists) |
| “waiting for the run_slam action server — timed out” | Broken shared ROS 2 daemon; the launcher uses `--no-daemon` (see `tmux attach -t slam`, window T2) |
| Plan fails: goals 0.0x m apart / yaw jump > 145° | Robot on or next to the dock: move it ~1 m into open space and plan again |
| Many goals time out | Robot carries a shelf; remove it or accept skipped goals |
| 3D viewer empty on a laptop | Forward port 9876 as well as 9090 |
| Only 1–2 frames while parked | Use `COLLECT_EXTRA` (section 7) with any `--duration` other than 120 |
| Orange / blotchy area in the map | Tinted glass: depth sees through it; not a masking error |

---

## 10. 測試紀錄 / Validation

- Offline, `mapping_20260922_01` (389 frames, lab, people walking/sitting): with the geo gate, person-free frames with > 1 % removed 59/296 → 1/296; YOLO-channel removal unchanged; reproduced through this branch's `slam_tools` (mean removed 2.18 %, motion-only 0.08 %).
- Live, map `lab_20260925` (ID c3bff72d): runs `run3`, `move1` (people walking, sitting, carrying a bottle removed). Map `lab_20260927` (ID 623d8033): `run1`, 10/19 goals.
- Live with `SEMANTIC=1`, map `Map803_3`, run `map803_0930_run5` (181 frames, people leaning on a stool and boxes): person points left in the map (checked against SAM 3 outlines and sensor depth) 36,244 → 10,149 (person fill, bridge) → 5,225 → 3,546 (box fill down, edge ring, carving). Much of the rest is box edges under a hand. Alignment RMSE 0.077 m with the camera lever arm corrected; 57/68 semantic objects get a navigation goal.
- Without `SEMANTIC=1`, alignment cannot run (`align.sh` needs the semantic export that `--no_deploy` skips).
