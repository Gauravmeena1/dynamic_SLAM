import math
import time
import unittest

from drive_waypoints import (
    minimum_goal_distance,
    move_with_timeout,
    orient_goals_for_arrival,
    thin,
    wait_for_command_stop,
)


class FakeClient:
    def __init__(self, delay=0.0, error=None):
        self.delay = delay
        self.error = error
        self.cancelled = False

    def move_to_pose(self, x, y, yaw, wait_for_completion=True):
        time.sleep(self.delay)
        if self.error is not None:
            raise self.error
        return (x, y, yaw, wait_for_completion)

    def cancel_command(self):
        self.cancelled = True

    def is_command_running(self):
        return False


class DriveWaypointsTests(unittest.TestCase):
    def test_move_returns_before_deadline(self):
        client = FakeClient()
        result = move_with_timeout(client, 1.0, 2.0, 0.5, 0.2)
        self.assertEqual(result, (1.0, 2.0, 0.5, True))
        self.assertFalse(client.cancelled)

    def test_move_timeout_cancels_command(self):
        client = FakeClient(delay=0.2)
        with self.assertRaises(TimeoutError):
            move_with_timeout(client, 1.0, 2.0, 0.5, 0.02)
        self.assertTrue(client.cancelled)

    def test_cancelled_command_is_stopped_before_next_goal(self):
        self.assertTrue(wait_for_command_stop(FakeClient(), timeout_s=0.01))

    def test_move_propagates_transport_error(self):
        client = FakeClient(error=RuntimeError("grpc failed"))
        with self.assertRaisesRegex(RuntimeError, "grpc failed"):
            move_with_timeout(client, 1.0, 2.0, 0.5, 0.2)

    def test_thin_keeps_endpoints(self):
        points = [(i * 0.05, 0.0, 0.0) for i in range(41)]
        goals = thin(points, min_step=0.6, goal_spacing=0.25)
        self.assertEqual(goals[0][:2], points[0][:2])
        self.assertEqual(goals[-1][:2], points[-1][:2])
        self.assertLess(len(goals), len(points))

    def test_thin_preserves_measured_initial_heading(self):
        points = [
            (0.0, 0.0, -0.4),
            (0.0, 0.05, 0.0),
            (0.5, 0.05, 0.0),
            (1.0, 0.05, 0.0),
        ]
        goals = thin(points, min_step=0.6, goal_spacing=0.25,
                     max_yaw_step_deg=180.0)
        self.assertEqual(goals[0][:2], points[0][:2])
        self.assertAlmostEqual(goals[0][2], -0.4)

    def test_minimum_goal_distance(self):
        goals = [(0.0, 0.0, 0.0), (0.3, 0.4, 0.0), (0.3, 0.6, 0.0)]
        self.assertAlmostEqual(minimum_goal_distance(goals), 0.2)
        self.assertTrue(math.isinf(minimum_goal_distance(goals[:1])))

    def test_orient_goals_uses_incoming_direction(self):
        goals = [(0.0, 0.0, 0.3), (1.0, 0.0, 1.2), (1.0, -2.0, 2.4)]
        oriented = orient_goals_for_arrival(goals)
        self.assertAlmostEqual(oriented[0][2], 0.3)
        self.assertAlmostEqual(oriented[1][2], 0.0)
        self.assertAlmostEqual(oriented[2][2], -math.pi / 2)


    def test_thin_bounds_heading_changes_through_turn(self):
        points = [
            (math.cos(i * math.pi / 100), math.sin(i * math.pi / 100),
             math.pi / 2 + math.pi / 200 if i == 0 else 0.0)
            for i in range(101)
        ]
        goals = thin(points, min_step=1.0, goal_spacing=0.6,
                     max_yaw_step_deg=70.0)
        steps = [
            abs((goals[i][2] - goals[i - 1][2] + math.pi)
                % (2 * math.pi) - math.pi)
            for i in range(1, len(goals))
        ]
        self.assertLessEqual(math.degrees(max(steps)), 70.1)


if __name__ == "__main__":
    unittest.main()
