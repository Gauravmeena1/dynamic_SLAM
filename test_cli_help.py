"""Smoke tests for public CLI entry points.

These tests intentionally run without robot SDKs and optional camera packages.
Every --help command must remain usable on a fresh GitHub checkout.
"""

from pathlib import Path
import subprocess
import sys
import unittest


ROOT = Path(__file__).resolve().parent


class CliHelpTests(unittest.TestCase):
    def assert_help(self, command, english_marker):
        result = subprocess.run(
            command,
            cwd=ROOT,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=15,
            check=False,
        )
        self.assertEqual(result.returncode, 0, result.stdout)
        self.assertIn(english_marker, result.stdout)
        self.assertTrue(any("\u4e00" <= char <= "\u9fff" for char in result.stdout),
                        "help output should also contain Chinese guidance")

    def test_python_cli_help_without_optional_dependencies(self):
        cases = [
            ("make_bridge_traj.py", "Generate a coverage trajectory"),
            ("drive_waypoints.py", "Execute a trajectory"),
            ("preview_traj.py", "Render a trajectory"),
            ("cam_snap.py", "RealSense snapshot"),
            ("blur_vs_omega.py", "Analyse image sharpness"),
        ]
        for script, marker in cases:
            with self.subTest(script=script):
                self.assert_help([sys.executable, str(ROOT / script), "--help"], marker)

    def test_shell_cli_help(self):
        cases = [
            ("run_bridge_oneshot.sh", "Required"),
            ("preflight.sh", "Read-only field preflight"),
            ("arm_cam_tune.sh", "Mapping-camera and arm-pose utility"),
        ]
        for script, marker in cases:
            with self.subTest(script=script):
                self.assert_help(["bash", str(ROOT / script), "--help"], marker)


if __name__ == "__main__":
    unittest.main()
