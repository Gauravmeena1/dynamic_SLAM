#!/usr/bin/env python3
"""影格銳利度 vs. 機器人角速度 —— [MAP-09] 的量測與驗收工具（全唯讀）。

用法（要有 cv2，Thor 上用 dualmap env）：
    python blur_vs_omega.py \
        /path/to/outputs_malong/<run>/rgb  pose2d_YYYYMMDD.csv

銳利度 = 灰階 Laplacian variance（跟 ma_slam_stream.realsense.laplacian_var 同一個定義）。
|ω| = pose2d log 在影格時間 ±0.2 s 內的 |dyaw|/dt。影格時間取自檔名 NNNN_<epoch ms>_rgb.png。
"""
import argparse
import bisect
import csv
import glob
import math
import os

HALF = 0.20
BINS = [(0, 0.02, "靜止"), (0.02, 0.1, "極慢"), (0.1, 0.25, "慢轉"), (0.25, 0.5, "中轉"), (0.5, 99, "快轉")]


def load_pose(path):
    ts, yaw = [], []
    with open(path) as f:
        for r in csv.DictReader(f):
            ts.append(float(r["t_wall"])); yaw.append(float(r["yaw"]))
    return ts, yaw


def state(ts, yaw, t):
    i0 = bisect.bisect_left(ts, t - HALF); i1 = bisect.bisect_right(ts, t + HALF) - 1
    if i1 - i0 < 2:
        return None, None
    d = sum(abs((yaw[i + 1] - yaw[i] + math.pi) % (2 * math.pi) - math.pi) for i in range(i0, i1))
    return d / (ts[i1] - ts[i0]), yaw[(i0 + i1) // 2]


def main():
    parser = argparse.ArgumentParser(
        description="影格銳利度與角速度分析 / Analyse image sharpness versus robot angular speed")
    parser.add_argument("rgb_dir", help="包含 *_rgb.png 的資料夾 / Directory containing *_rgb.png")
    parser.add_argument("pose_csv", help="含 t_wall,yaw 的 Pose2D CSV / Pose2D CSV with t_wall,yaw")
    args = parser.parse_args()
    try:
        import cv2
        import numpy as np
    except ModuleNotFoundError as exc:
        raise SystemExit(
            f"缺少選用相依套件 / Missing optional dependency: {exc.name}. "
            "Install requirements-optional.txt."
        ) from exc

    rgb_dir, pose_csv = args.rgb_dir, args.pose_csv
    ts, yaw = load_pose(pose_csv)
    rows = []
    for p in sorted(glob.glob(os.path.join(rgb_dir, "*_rgb.png"))):
        t = int(os.path.basename(p).split("_")[1]) / 1000.0
        img = cv2.imread(p, cv2.IMREAD_GRAYSCALE)
        if img is None:
            continue
        w, y = state(ts, yaw, t)
        if w is not None:
            rows.append((os.path.basename(p), float(cv2.Laplacian(img, cv2.CV_64F).var()),
                         float(img.mean()), w, y))
    if not rows:
        raise SystemExit("沒有影格對得上 pose log / No RGB frame overlaps the Pose2D time range")

    print(f"對得上 pose 的影格 {len(rows)}")
    print(f"{'|ω| 區間 (rad/s)':<22}{'幀數':>6}{'sharp 中位':>11}{'p10':>8}{'亮度':>7}")
    for lo, hi, name in BINS:
        sel = [r for r in rows if lo <= r[3] < hi]
        if sel:
            s = [r[1] for r in sel]
            print(f"{name} [{lo:.2f},{hi:.2f})".ljust(22) + f"{len(sel):>6}{np.median(s):>11.0f}"
                  f"{np.percentile(s, 10):>8.0f}{np.mean([r[2] for r in sel]):>7.1f}")
    w = [r[3] for r in rows]
    print(f"|ω| 中位 {np.median(w):.3f}  p90 {np.percentile(w, 90):.3f}  max {max(w):.3f} rad/s；"
          f"|ω|>0.25 佔 {100 * sum(x > 0.25 for x in w) / len(w):.1f}%；"
          f"sharp<20 的幀 {sum(r[1] < 20 for r in rows)}")

    print("\n丟掉轉動幀後的方位覆蓋（30° 一格，共 12 格）：")
    for thr in [None, 0.25, 0.15]:
        sel = rows if thr is None else [r for r in rows if r[3] < thr]
        h = np.zeros(12, int)
        for r in sel:
            h[int(((r[4] + math.pi) % (2 * math.pi)) / (2 * math.pi) * 12) % 12] += 1
        lab = "全部" if thr is None else f"|ω|<{thr}"
        print(f"  {lab:<9} 保留 {len(sel):4d} ({100 * len(sel) / len(rows):5.1f}%)  "
              f"覆蓋 {int((h > 0).sum()):2d}/12  最少一格 {h.min()} 幀")


if __name__ == "__main__":
    main()
