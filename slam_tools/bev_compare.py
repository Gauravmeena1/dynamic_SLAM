#!/usr/bin/env python3
"""Bird's-eye view of a masked SLAM run: before removal | after removal | removed points in red.

    python bev_compare.py /fungi/outputs_malong/<run> [--cell 0.02] [--ceiling 1.9]

Reads all_points_pcd.ply / static_only_pcd.ply / dynamic_pcd.ply (fusion_solver.export_split_clouds)
and camera_poses.txt. The SLAM world is camera convention (y down) and tilted by the first frame's
pitch, so the floor is found with RANSAC (open3d) and the cloud is rotated so the floor normal is up.
Each 2 cm cell shows the colour of its highest point below --ceiling metres (a view from the ceiling).
Writes <run>/bev_compare.png.
"""
import argparse
import os

import numpy as np
from PIL import Image, ImageDraw


def read_ply(path):
    with open(path, "rb") as fh:
        n = 0
        while True:
            line = fh.readline().decode().strip()
            if line.startswith("element vertex"):
                n = int(line.split()[-1])
            if line == "end_header":
                break
        dt = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                       ("r", "u1"), ("g", "u1"), ("b", "u1")])
        v = np.frombuffer(fh.read(n * dt.itemsize), dtype=dt, count=n)
    return (np.stack([v["x"], v["y"], v["z"]], 1).astype(np.float64),
            np.stack([v["r"], v["g"], v["b"]], 1))


def level_frame(P, cams):
    """Rotation + floor height that make the floor z=0 and 'up' +z."""
    up0 = -np.mean([T[:3, 1] for T in cams], axis=0)        # camera -y averaged over all headings
    up0 /= np.linalg.norm(up0)
    h = P @ up0
    low = P[h < np.percentile(h, 25)]                        # floor lives in the lowest quarter
    import open3d as o3d            # needed only for the floor fit; keeps --help dependency-free
    pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(low))
    (a, b, c, d), _ = pc.segment_plane(distance_threshold=0.03, ransac_n=3, num_iterations=2000)
    n = np.array([a, b, c]); n /= np.linalg.norm(n)
    if n @ up0 < 0:
        n = -n
    z = n
    x = np.cross([0, 1, 0] if abs(z[1]) < 0.9 else [1, 0, 0], z); x /= np.linalg.norm(x)
    y = np.cross(z, x)
    R = np.stack([x, y, z])
    inl = np.abs(low @ np.array([a, b, c]) + d) < 0.03                # RANSAC floor inliers (unflipped plane)
    floor = float(np.median((low[inl] @ R.T)[:, 2]))
    return R, floor


def render(P, C, lo, hi, cell, ceiling, bg=(255, 255, 255)):
    keep = (P[:, 2] > 0.05) & (P[:, 2] < ceiling)            # drop floor itself + ceiling/lights
    P, C = P[keep], C[keep]
    W, H = int((hi[0] - lo[0]) / cell) + 1, int((hi[1] - lo[1]) / cell) + 1
    ix = ((P[:, 0] - lo[0]) / cell).astype(int); iy = ((hi[1] - P[:, 1]) / cell).astype(int)
    ok = (ix >= 0) & (ix < W) & (iy >= 0) & (iy < H)
    ix, iy, z, C = ix[ok], iy[ok], P[ok, 2], C[ok]
    order = np.argsort(z)                                     # later (higher) points overwrite
    img = np.full((H, W, 3), bg, np.uint8)
    img[iy[order], ix[order]] = C[order]
    return img


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("--cell", type=float, default=0.02)
    ap.add_argument("--ceiling", type=float, default=1.9)
    a = ap.parse_args()
    run = a.run.rstrip("/")
    name = os.path.basename(run)
    Pa, Ca = read_ply(f"{run}/all_points_pcd.ply")
    Ps, Cs = read_ply(f"{run}/static_only_pcd.ply")
    Pd, _ = read_ply(f"{run}/dynamic_pcd.ply")
    cams = [np.array(l.split(), float).reshape(4, 4) for l in open(f"{run}/camera_poses.txt") if l.strip()]

    R, floor = level_frame(Pa, cams)
    tf = lambda P: (P @ R.T) - [0, 0, floor]
    Pa, Ps, Pd = tf(Pa), tf(Ps), tf(Pd)
    cam = tf(np.array([T[:3, 3] for T in cams]))

    lo = np.percentile(Pa[:, :2], 0.5, axis=0) - 0.3
    hi = np.percentile(Pa[:, :2], 99.5, axis=0) + 0.3
    before = render(Pa, Ca, lo, hi, a.cell, a.ceiling)
    after = render(Ps, Cs, lo, hi, a.cell, a.ceiling)
    grey = render(Pa, np.full((len(Pa), 3), 185, np.uint8), lo, hi, a.cell, a.ceiling)
    red = render(Pd, np.tile([[230, 0, 0]], (len(Pd), 1)).astype(np.uint8), lo, hi, a.cell, a.ceiling)
    mask = (red != 255).any(-1)
    grey[mask] = red[mask]

    panels = [Image.fromarray(x) for x in (before, after, grey)]
    d = ImageDraw.Draw(panels[2])                             # camera path on the removal panel
    pts = [((p[0] - lo[0]) / a.cell, (hi[1] - p[1]) / a.cell) for p in cam]
    d.line(pts, fill=(40, 100, 255), width=2)
    d.ellipse([pts[0][0] - 5, pts[0][1] - 5, pts[0][0] + 5, pts[0][1] + 5], fill=(0, 160, 0))

    w, h = panels[0].size
    out = Image.new("RGB", (3 * w + 40, h + 50), (255, 255, 255))
    D = ImageDraw.Draw(out)
    titles = [f"{name}: BEFORE removal",
              "AFTER removal (static map)",
              "removed points in RED, blue = camera path"]
    for k, (p, t) in enumerate(zip(panels, titles)):
        out.paste(p, (k * (w + 20), 40))
        D.text((k * (w + 20) + 4, 8), t, fill=(0, 0, 0))
    bar = int(1.0 / a.cell)
    D.line([(10, h + 45), (10 + bar, h + 45)], fill=(0, 0, 0), width=3); D.text((14 + bar, h + 38), "1 m", fill=(0, 0, 0))
    # point counts are NOT comparable across the three files (each is sampled to its own cap), so
    # report the pixel-level removal from mask_viz/removed.csv instead
    try:
        v = np.loadtxt(f"{run}/mask_viz/removed.csv", delimiter=",", skiprows=1)[:, 1]
        D.text((60 + bar, h + 38), f"pixels removed: mean {v.mean():.1f}% per frame, "
               f"{(v > 1).sum()}/{len(v)} frames > 1%", fill=(0, 0, 0))
    except Exception:
        pass
    out = out.resize((out.size[0] * 2, out.size[1] * 2), Image.NEAREST)
    dst = f"{run}/bev_compare.png"
    out.save(dst)
    print(f"{name}: wrote {dst}  ({out.size[0]}x{out.size[1]})")


if __name__ == "__main__":
    main()
