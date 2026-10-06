"""Offline tests for the zigzag pattern and the end-of-segment goal merge (no robot)."""
import csv
import math
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np
from PIL import Image

from drive_waypoints import minimum_goal_distance, thin

HERE = Path(__file__).resolve().parent
FREE, WALL = 253, 175


def _room(tmp: Path, w_m=4.0, h_m=2.0, res=0.05):
    """A walled rectangular room as a Kachaka-style 2D map (PNG + YAML)."""
    W, H = int(w_m / res), int(h_m / res)
    img = np.full((H, W), FREE, np.uint8)
    img[:3, :] = img[-3:, :] = WALL
    img[:, :3] = img[:, -3:] = WALL
    Image.fromarray(img).save(tmp / "room.png")
    (tmp / "room.yaml").write_text(
        f"image: room.png\nresolution: {res}\norigin: [0.0, 0.0, 0.0]\nwidth: {W}\nheight: {H}\n")
    return tmp / "room.yaml"


class ZigzagTests(unittest.TestCase):
    def test_last_waypoint_is_not_a_few_cm_before_the_end(self):
        # straight run a bit longer than min_step: before the fix the last relay point
        # sat ~5 cm before the end and drive_waypoints rejected the too-close goals
        pts = [(0.05 * i, 0.0, 0.0) for i in range(26)]          # 0 .. 1.25 m: goals 1.20 and 1.25 before
        goals = thin(pts, min_step=1.0, goal_spacing=0.4)
        self.assertGreaterEqual(minimum_goal_distance(goals), 0.4 - 1e-9)
        self.assertAlmostEqual(goals[-1][0], 1.25, places=6)

    def test_zigzag_plan_runs_and_stays_inside_the_room(self):
        with tempfile.TemporaryDirectory() as d:
            tmp = Path(d)
            yaml_path = _room(tmp)
            out = tmp / "traj.csv"
            r = subprocess.run([sys.executable, str(HERE / "make_bridge_traj.py"), "--map-yaml", str(yaml_path),
                                "--start", "0.5,1.0,0.0", "--pattern", "zigzag", "--spacing", "0.6",
                                "--min-clearance", "0.40", "--out", str(out), "--preview", str(tmp / "preview.png")],
                               capture_output=True, text=True, cwd=tmp, timeout=120)
            self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
            with open(out) as f:
                rows = [(float(x["x"]), float(x["y"])) for x in csv.DictReader(f)]
            self.assertGreater(len(rows), 2)
            xs, ys = [p[0] for p in rows], [p[1] for p in rows]
            # 0.40 m clearance from walls of a 4 x 2 m room
            self.assertGreaterEqual(min(xs), 0.40 - 0.06)
            self.assertLessEqual(max(xs), 4.0 - 0.40 + 0.06)
            self.assertGreaterEqual(min(ys), 0.40 - 0.06)
            self.assertLessEqual(max(ys), 2.0 - 0.40 + 0.06)
            self.assertGreater(max(xs) - min(xs), 1.5)            # it actually travels along the long axis
            self.assertTrue(all(math.isfinite(v) for p in rows for v in p))



if __name__ == "__main__":
    unittest.main()
