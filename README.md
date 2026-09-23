# Kachaka 自動建圖路線工具 / Kachaka Automatic Mapping Route Tools

這個專案從 Kachaka 2D occupancy map 產生覆蓋路徑、導航目標與人工審查圖，並可在現場串接 pose logger、3D SLAM、Kachaka navigation 與 alignment，完成建圖資料收集。

This repository generates coverage paths, navigation goals, and review images from a Kachaka 2D occupancy map. At a configured site it can also coordinate pose logging, 3D SLAM, Kachaka navigation, and alignment for one mapping run.

> **安全 / Safety:** 完整流程會移動 Kachaka，部分相機調整指令也會移動機械手臂。第一次使用請先跑離線測試與 `--plan-only`，正式執行時必須有人守在急停按鈕旁。The full workflow moves the robot, and some camera-tuning commands move the arm. Run offline tests and `--plan-only` first. A trained operator must stay within reach of the emergency stop during hardware runs.

## 功能 / Features

- 蛇行或螺旋覆蓋路徑 / Boustrophedon or spiral coverage paths
- 機器人半徑、牆面淨空與連通區域檢查 / Robot-radius, wall-clearance, and connectivity checks
- Waypoint 抽稀、最小點距與 yaw 變化限制 / Waypoint thinning, minimum-distance, and yaw-step limits
- 帶方向及順序的預覽圖 / Directional, ordered route previews
- 導航逾時安全取消 / Safe cancellation after navigation timeout
- Pose2D logger、3D SLAM、alignment 串接 / Pose2D logger, 3D SLAM, and alignment orchestration
- Container、相機、手臂、磁碟與殘留程序 preflight / Preflight checks for containers, camera, arm, disk, and stale processes

## 專案結構 / Repository layout

| File | 中文用途 / Purpose |
|---|---|
| `run_bridge_oneshot.sh` | 七階段現場流程；移動前會確認 / Seven-stage field workflow with confirmation before motion |
| `make_bridge_traj.py` | 從 2D map 產生密集覆蓋路徑 / Generate a dense coverage trajectory from a 2D map |
| `drive_waypoints.py` | 抽稀及驗證導航目標；只有 `--go` 會移動 / Thin and validate goals; motion requires `--go` |
| `preview_traj.py` | 將路徑疊在地圖上 / Render a trajectory over the map |
| `preflight.sh` | 唯讀現場檢查 / Read-only site checks |
| `arm_cam_tune.sh` | 相機畫面與手臂收合姿態調整 / Camera view and arm stow-pose tuning |
| `blur_vs_omega.py` | 選用的模糊與角速度分析 / Optional blur-versus-angular-speed analysis |
| `test_drive_waypoints.py` | 不需機器人的離線測試 / Offline tests without a robot |
| `KNOWN_ISSUES.md` | 實機問題、原因與排除紀錄 / Field issues, causes, and workarounds |

生成物預設寫入 `artifacts/runs/` 與 `artifacts/previews/`，兩者都不會進 Git。Generated files go to `artifacts/runs/` and `artifacts/previews/`; both are ignored by Git.

## 快速開始 / Quick start (offline)

需求 / Requirements: Python 3.10+.

```bash
git clone <REPOSITORY_URL>
cd kachaka_mapping

python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt

python -m unittest discover -s . -p 'test_*.py' -v
```

在沒有 Kachaka 的電腦上也能規劃。準備相鄰的 occupancy-map PNG/YAML，並以 `--start x,y,yaw` 提供起點。Planning works without a robot when an adjacent occupancy-map PNG/YAML pair and an explicit start pose are provided.

```bash
python make_bridge_traj.py \
  --map-yaml /path/to/map.yaml \
  --start 0.0,0.0,0.0 \
  --pattern boustrophedon \
  --spacing 0.6 \
  --min-clearance 0.40 \
  --out artifacts/runs/traj_demo.csv

python drive_waypoints.py artifacts/runs/traj_demo.csv \
  --map-yaml /path/to/map.yaml \
  --save-goals artifacts/runs/goals_demo.csv

python preview_traj.py \
  --map-yaml /path/to/map.yaml \
  --traj artifacts/runs/goals_demo.csv
```

以上指令沒有 `--go`，不會移動硬體。These commands omit `--go` and therefore do not move hardware.

## 現場設定 / Site configuration

完整現場流程還需要 site-provided Kachaka API、Docker containers、pose logger 與 3D SLAM 工具鏈。The field workflow also requires a site-provided Kachaka API, Docker containers, pose logger, and 3D SLAM toolchain.

```bash
cp .env.example .env
# Edit .env: paths, endpoint, container names, and camera serial.
${EDITOR:-nano} .env

./preflight.sh --help
./preflight.sh --plan-only
```

`.env` 是本機設定並已被 `.gitignore` 排除。不要把內網 IP、帳號路徑、token 或相機序號提交到 Git。`.env` is local and ignored. Never commit private IP addresses, account-specific paths, tokens, or camera serial numbers.

`kachaka_api` 由 Kachaka SDK 提供，不在 PyPI requirements 裡；用 `KACHAKA_API_PATH` 指向其 Python package。`kachaka_api` comes from the Kachaka SDK and is not a PyPI dependency; point `KACHAKA_API_PATH` to its Python package.

選用相機／分析功能 / Optional camera and analysis tools:

```bash
python -m pip install -r requirements-optional.txt
```

`pyrealsense2` 依平台安裝，未必能直接由 PyPI 安裝。`pyrealsense2` is platform-specific and may require the vendor installation method.

## 執行 / Run

先只規劃與出圖 / Plan and preview only:

```bash
./run_bridge_oneshot.sh --name demo_01 --venue lab --plan-only
```

人工確認預覽圖、目前 Kachaka map ID、淨空與急停措施後，才執行完整流程。Run the full workflow only after reviewing the preview, active Kachaka map ID, clearance, and emergency-stop setup.

```bash
./run_bridge_oneshot.sh --name demo_01 --venue lab
```

安全機制 / Safety gates:

- `drive_waypoints.py` 沒有 `--go` 時只印計畫 / Without `--go`, it only prints the plan.
- Wrapper 在導航前要求人工確認；第一次不要用 `-y` / The wrapper confirms before navigation; do not use `-y` on a first run.
- `--reuse-map` 驗證目前 Kachaka map ID / `--reuse-map` verifies the active map ID.
- 生成與執行必須使用相同 `--min-clearance` / Planning and execution must use the same `--min-clearance`.
- Ctrl+C、API error 與 timeout 會走取消／收尾流程 / Ctrl+C, API errors, and timeouts trigger cancellation and cleanup.

所有參數 / Full CLI help:

```bash
./run_bridge_oneshot.sh --help
./preflight.sh --help
python make_bridge_traj.py --help
python drive_waypoints.py --help
python preview_traj.py --help
./arm_cam_tune.sh --help
```

## 地圖與 alignment / Maps and alignment

2D map、3D run、Pose2D log 與 alignment 必須屬於同一個 Kachaka map ID。Point-cloud quality alone is not sufficient: the 2D map, 3D run, Pose2D log, and alignment must belong to the same Kachaka map ID.

目前驗證紀錄 / Current validation reports:

- [`docs/ALIGNMENT_VALIDATION_20260921.md`](docs/ALIGNMENT_VALIDATION_20260921.md)
- [`docs/ONSITE_REMAP_RUN3_VALIDATION_20260922.md`](docs/ONSITE_REMAP_RUN3_VALIDATION_20260922.md)
- [`docs/ROBOTIC_AGENT_BUNDLE_VALIDATION_20260922.md`](docs/ROBOTIC_AGENT_BUNDLE_VALIDATION_20260922.md)

## 提交前驗證 / Verification before commit

```bash
python -m unittest discover -s . -p 'test_*.py' -v
python -m compileall -q .
bash -n run_bridge_oneshot.sh preflight.sh preflight_legacy.sh arm_cam_tune.sh
```

GitHub Actions 會執行相同的離線檢查。GitHub Actions runs the same offline checks. Hardware-affecting changes additionally require a supervised low-speed test.

## GitHub 上傳 / Publish to GitHub

先確認忽略規則，尤其不要加入 `.env`、地圖、pose logs、軌跡或 SLAM outputs。Check ignored files first; do not add `.env`, maps, pose logs, trajectories, or SLAM outputs.

```bash
cd /path/to/kachaka_mapping

git status --short
git check-ignore -v .env artifacts/ || true
git diff --check

git add .gitignore .env.example .github README.md CONTRIBUTING.md KNOWN_ISSUES.md \
  docs requirements.txt requirements-optional.txt \
  '*.py' '*.sh'
git status --short
git diff --cached --check

# Review exactly what will be committed.
git diff --cached --stat
git diff --cached

git commit -m "Prepare bilingual Kachaka mapping toolkit"
git branch -M main
git remote add origin git@github.com:<OWNER>/<REPOSITORY>.git
git push -u origin main
```

若 `origin` 已存在 / If `origin` already exists:

```bash
git remote -v
git remote set-url origin git@github.com:<OWNER>/<REPOSITORY>.git
git push -u origin main
```

建議在 GitHub 建立空 repository，不要先自動加入 README、`.gitignore` 或 LICENSE，避免第一次 push 前產生不必要的 history conflict。Create an empty GitHub repository without generated starter files to avoid an unnecessary first-push history conflict.

## License

公開前請選擇並加入合適的 `LICENSE`。沒有 license 時，其他人雖能閱讀程式碼，但不代表取得複製、修改或散布授權。Choose and add a `LICENSE` before publishing. Without one, others may view the code but do not automatically receive permission to copy, modify, or redistribute it.
