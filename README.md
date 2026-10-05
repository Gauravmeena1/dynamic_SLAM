# dynamic_SLAM

Automatic 3D mapping with a Kachaka robot, with people and moving objects removed from the map.

This repository runs automatic 3D mapping with a Kachaka robot. It plans a coverage route on the Kachaka 2D map, drives it, and records RGB-D video for a 3D SLAM server. While the map is built, the server removes people and moving objects. The 3D map is then aligned to the robot's 2D map.

Project page: <https://gauravmeena1.github.io/dynamic_SLAM/>

> **Safety:** The full workflow moves the robot, and some camera-tuning commands move the arm. Run the offline tests and `--plan-only` first. A trained operator must stay within reach of the emergency stop during hardware runs.

## Features

- Boustrophedon or spiral coverage paths with robot-radius and wall-clearance checks
- Waypoint thinning, yaw-step limits, and ordered route previews
- Safe cancellation after a navigation timeout
- Pose2D logging, 3D SLAM, and 2D/3D alignment in one workflow
- Live removal of people and moving objects from the 3D map (YOLOv9e-seg + optical flow)
- Multi-view 3D carving of leftover person points
- Preflight checks for containers, camera, arm, disk, and stale processes

## Repository layout

| Path | Purpose |
|---|---|
| `run_bridge_oneshot.sh` | Seven-stage field workflow with confirmation before motion |
| `make_bridge_traj.py` | Generate a coverage trajectory from a 2D map |
| `drive_waypoints.py` | Thin and validate goals; motion requires `--go` |
| `preview_traj.py` | Render a trajectory over the map |
| `preflight.sh` | Read-only site checks |
| `arm_cam_tune.sh` | Camera view and arm stow-pose tuning |
| `dynamic_masking/` | Dynamic-object masking method |
| `slam_integration/` | Hooks the masker into the SLAM server (`install.sh`, patches) |
| `slam_tools/` | SLAM launchers, live view, replay, robot control |
| `docs/` | Guides and validation reports |
| `KNOWN_ISSUES.md` | Field issues, causes, and workarounds |

Generated files go to `artifacts/`, which is ignored by Git. Model weights are not in Git either.

## Quick start (offline)

Requirements: Python 3.10+.

```bash
git clone https://github.com/Gauravmeena1/dynamic_SLAM.git
cd dynamic_SLAM

python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt

python -m unittest discover -s . -p 'test_*.py' -v
```

Planning works without a robot when an occupancy-map PNG/YAML pair and a start pose are given:

```bash
python make_bridge_traj.py --map-yaml /path/to/map.yaml --start 0,0,0 \
  --pattern boustrophedon --spacing 0.6 --min-clearance 0.40 \
  --out artifacts/runs/traj_demo.csv
python drive_waypoints.py artifacts/runs/traj_demo.csv \
  --map-yaml /path/to/map.yaml --save-goals artifacts/runs/goals_demo.csv
python preview_traj.py --map-yaml /path/to/map.yaml --traj artifacts/runs/goals_demo.csv
```

These commands do not use `--go`, so they never move hardware.

## Site configuration

The field workflow also needs the Kachaka SDK, Docker containers, and the 3D SLAM toolchain.

```bash
cp .env.example .env
${EDITOR:-nano} .env      # paths, robot endpoint, container names, camera serial
./preflight.sh --plan-only
```

`.env` is local and ignored by Git. Never commit IP addresses, account paths, tokens, or camera serial numbers.

## Run

```bash
# plan and preview only
./run_bridge_oneshot.sh --name demo_01 --venue lab --plan-only

# full run, after checking the preview, the active map ID, and the emergency-stop setup
./run_bridge_oneshot.sh --name demo_01 --venue lab
```

Safety gates:

- Without `--go`, `drive_waypoints.py` only prints the plan.
- The wrapper asks for confirmation before navigation; do not use `-y` on a first run.
- `--reuse-map` checks the active Kachaka map ID.
- Ctrl+C, API errors, and timeouts trigger cancellation and cleanup.

## Dynamic-object masking

People are always removed from the 3D map. Other movable objects are removed only while they move or are carried. Setup, the offline test, the live run, and all settings are in [`docs/DYNAMIC_MASKING.md`](docs/DYNAMIC_MASKING.md).

## Maps and alignment

The 2D map, 3D run, Pose2D log, and alignment must all belong to the same Kachaka map ID. Validation reports are in [`docs/`](docs/).

## Verification before commit

```bash
python -m unittest discover -s . -p 'test_*.py' -v
python -m compileall -q .
bash -n run_bridge_oneshot.sh preflight.sh preflight_legacy.sh arm_cam_tune.sh
```

GitHub Actions runs the same offline checks. Changes that affect the hardware also need a supervised low-speed test.

## Credits

The mapping route tools come from [h44343880/kachaka_mapping](https://github.com/h44343880/kachaka_mapping). Dynamic-object masking and the SLAM integration are by Gaurav.

## License

No license has been chosen yet. Without one, others may read the code but may not copy, modify, or redistribute it.
