# 貢獻指南 / Contributing

## 修改前 / Before opening a change

1. 把機器專用路徑、IP、token 與相機序號放在 `.env`，不要寫進 source。Keep machine-specific paths, IP addresses, tokens, and camera serial numbers in `.env`, not in source files.
2. 不要提交生成的 maps、trajectories、pose logs、SLAM outputs 或 `artifacts/`。Do not commit generated maps, trajectories, pose logs, SLAM outputs, or `artifacts/`.
3. 新增或修改 CLI 時，`--help` 必須在沒有機器人、Docker 與選用相依套件時仍可執行。CLI `--help` must work without a robot, Docker, or optional dependencies.

## 離線檢查 / Offline checks

```bash
python3 -m unittest discover -s . -p 'test_*.py' -v
python3 -m compileall -q .
bash -n run_bridge_oneshot.sh preflight.sh preflight_legacy.sh arm_cam_tune.sh
git diff --check
```

## 安全 / Safety

影響 `--go`、手臂命令、cleanup、timeout 或 clearance 的修改，必須先有離線測試，再做有人監看的低速實機測試。不要為了簡化自動化而移除 `--go` 或人工確認。

Changes affecting `--go`, arm commands, cleanup, timeouts, or clearance checks require an offline test followed by a supervised low-speed hardware test. Never remove the explicit `--go` or confirmation gates merely to simplify automation.

## Pull request checklist

- [ ] 中英文操作說明仍一致 / Chinese and English instructions still agree.
- [ ] 離線測試與 shell syntax checks 通過 / Offline and shell syntax checks pass.
- [ ] 沒有 secrets、現場 IP 或大型產物 / No secrets, site IPs, or generated large files.
- [ ] 若改變實機行為，附上 map ID、測試條件與結果 / Hardware behavior changes include the map ID, test conditions, and result.
