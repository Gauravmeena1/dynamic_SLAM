#!/usr/bin/env python3
"""把 traj.csv 疊在 2D 地圖上畫成一張看得懂的預覽圖（純 PIL，Thor 上沒有 matplotlib）。

跟 make_bridge_traj.py 內建的簡易預覽不同，這支是給「跑之前人工審查」用的：
用顏色漸層表示行進順序、畫方向箭頭、標出起點/終點與比例尺，
方便確認路徑順序合理、不會擦到牆、涵蓋範圍是你要的那一塊。

    python3 preview_traj.py --map-yaml kachaka_2d_map4_20260826.yaml \
        --traj traj_bridge_20260826.csv --out preview.png
"""
import os
import argparse
import csv
import math
from pathlib import Path

import numpy as np
import yaml
from PIL import Image, ImageDraw

FREE, UNKNOWN, WALL = 253, 234, 175


def main():
    ap = argparse.ArgumentParser(description="將 traj.csv 疊在 2D map / Render a trajectory over a 2D map for review.")
    ap.add_argument("--map-yaml", required=True, help="Kachaka occupancy-map YAML / 地圖 YAML")
    ap.add_argument("--traj", required=True, help="Trajectory or goal CSV / 路徑或目標 CSV")
    ap.add_argument("--out", default=None,
                    help="輸出圖 / Output image (default: artifacts/previews/preview_<trajectory>.png)")
    ap.add_argument("--scale", type=int, default=4, help="放大倍率 / Image scale (default: 4)")
    ap.add_argument("--arrow-every", type=int, default=25, help="方向箭頭間隔 / Draw a heading arrow every N waypoints")
    args = ap.parse_args()

    # 預覽圖預設收在專案的 artifacts/previews，可用 KACHAKA_PREVIEW_DIR 覆寫
    if args.out is None:
        viz = Path(os.getenv("KACHAKA_PREVIEW_DIR", Path(__file__).parent / "artifacts" / "previews"))
        viz.mkdir(parents=True, exist_ok=True)
        args.out = str(viz / f"preview_{Path(args.traj).stem}.png")

    yp = Path(args.map_yaml)
    cfg = yaml.safe_load(yp.read_text())
    res = float(cfg["resolution"])
    ox, oy = float(cfg["origin"][0]), float(cfg["origin"][1])
    W, H = int(cfg["width"]), int(cfg["height"])
    gray = np.array(Image.open(yp.parent / cfg["image"]).convert("L"))

    S = args.scale
    # 底圖：free 淡白、unknown 淺灰、牆深灰，拉開對比才看得出結構
    rgb = np.zeros((H, W, 3), np.uint8)
    rgb[gray == FREE] = (252, 252, 250)
    rgb[gray == UNKNOWN] = (232, 232, 232)
    rgb[gray == WALL] = (90, 95, 105)
    other = ~np.isin(gray, [FREE, UNKNOWN, WALL])
    rgb[other] = np.dstack([gray] * 3)[other]

    im = Image.fromarray(rgb).resize((W * S, H * S), Image.NEAREST)
    d = ImageDraw.Draw(im)

    def to_px(x, y):
        return ((x - ox) / res * S, (H - 1 - (y - oy) / res) * S)

    pts = []
    with open(args.traj) as f:
        for row in csv.DictReader(f):
            pts.append((float(row["x"]), float(row["y"]), float(row["yaw"])))
    if len(pts) < 2:
        raise SystemExit("traj 至少要兩個點")

    # 顏色漸層：藍(起點) → 紅(終點)，一眼看出行進順序
    def grad(t):
        return (int(40 + 215 * t), int(90 * (1 - t) + 40 * t), int(230 * (1 - t) + 40 * t))

    for i in range(len(pts) - 1):
        t = i / (len(pts) - 2)
        d.line([to_px(*pts[i][:2]), to_px(*pts[i + 1][:2])], fill=grad(t), width=max(2, S // 2))

    # 方向箭頭
    for i in range(0, len(pts) - 1, args.arrow_every):
        x, y, _ = pts[i]
        nx, ny, _ = pts[min(i + 3, len(pts) - 1)]
        a = math.atan2(ny - y, nx - x)
        px, py = to_px(x, y)
        L = S * 3.2
        tip = (px + L * math.cos(a), py - L * math.sin(a))
        for s in (+1, -1):
            b = a + s * 2.6
            d.line([tip, (tip[0] + L * 0.62 * math.cos(b), tip[1] - L * 0.62 * math.sin(b))],
                   fill=(20, 20, 20), width=max(1, S // 3))

    # 起點 / 終點
    def marker(p, color, r=S * 2.4):
        px, py = to_px(*p[:2])
        d.ellipse([px - r, py - r, px + r, py + r], fill=color, outline=(255, 255, 255), width=max(1, S // 3))

    marker(pts[0], (0, 90, 235))
    marker(pts[-1], (235, 40, 40))
    # 起點朝向
    px, py = to_px(*pts[0][:2])
    L = S * 6
    d.line([(px, py), (px + L * math.cos(pts[0][2]), py - L * math.sin(pts[0][2]))],
           fill=(0, 90, 235), width=max(2, S // 2))

    # 比例尺：1 公尺
    m_px = S / res
    x0, y0 = S * 6, H * S - S * 8
    d.line([(x0, y0), (x0 + m_px, y0)], fill=(20, 20, 20), width=max(2, S // 2))
    for xx in (x0, x0 + m_px):
        d.line([(xx, y0 - S * 2), (xx, y0 + S * 2)], fill=(20, 20, 20), width=max(2, S // 2))
    d.text((x0, y0 - S * 7), "1 m", fill=(20, 20, 20))

    length = sum(math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
                 for i in range(len(pts) - 1))
    d.text((S * 6, S * 5),
           f"{Path(args.traj).name}   {len(pts)} waypoints   {length:.1f} m\n"
           f"blue=start  red=end  arrows=direction",
           fill=(20, 20, 20))

    im.save(args.out)
    print(f"✅ {args.out}  ({im.size[0]}x{im.size[1]})  路徑長 {length:.1f} m / {len(pts)} 點")


if __name__ == "__main__":
    main()
