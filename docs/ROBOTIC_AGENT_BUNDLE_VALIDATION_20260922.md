# Robotic-agent bundle validation — `ec129_0911_bridge`

Validation date: 2026-09-22

## Bundle

Prepared in:

`$HOME/workspaces/robotic_system/robot_ws/data/lab/ec129_0911_bridge`

Deployed additively to the running robotic-agent workspace as:

`/robot_ws/data/lab/ec129_0911_bridge`

No existing site bundle or active launch parameter was overwritten.

## Export validation

- Three point-cloud NPZ files: 1,890,982 points each
- Semantic instances: 170 instances across 30 categories
- Detected `bottle` instances: IDs 15, 25, and 27
- fungi-to-agent transform equivalence: maximum difference `1.831e-15 m`
- Alignment RMSE: 0.108 m over 909 matched frames

## Occupancy map

The Kachaka native grayscale map was converted as follows:

- occupied: 175 → 0, 1,118 pixels
- free: 253 → 254, 27,675 pixels
- unknown: 234 → 205, 54,247 pixels

All source/output category counts match exactly. The unknown area remains large, so this bundle is restricted to the mapped part of the room.

## Navigation-goal dry run

Using the robotic-system `OccupancyGrid` and default goal constraints (0.30–0.45 m obstacle clearance, ≤1.0 m standoff):

- All three bottle instances are in known-free cells.
- All three bottle instances produce a valid free-space navigation goal.
- Two of three table instances produce valid goals; the third lies in unknown space.
- Two sofa instances can produce nearby goals despite unknown target cells; the third has no valid goal.

## ROS container validation

Tests ran in isolated ROS domains and did not invoke navigation or manipulation.

- `object_query_server` loaded the alignment, all 170 instances, and 30 categories.
- `/object_query` correctly rejected ambiguous `bottle` and requested an instance selection.
- `/object_query` for `bottle#15` succeeded and returned raw 3D position `(0.614, 0.296, 3.579)`.
- `agent_decision_maker_node` loaded the alignment, all instance bounding boxes, and the 346×240 occupancy map.
- Decision maker reported `live execution, object_query=live` with camera pre-action checking enabled.

The shutdown traceback observed in the supervised timeout is caused by the node calling `rclpy.shutdown()` after the timeout signal already shut down the same context. It occurs after successful startup and is unrelated to bundle loading.

## Hard gate before physical execution

The bundle is tied to:

- map name: `自動建圖測試0911`
- map ID: `0fb1b07a-b3b4-4981-a444-4bbe88792d7c`

At validation time, the robot was using:

- map name: `lab_0919`
- map ID: `c2e40f1b-1e0c-48f5-9170-29f80bc6fc9b`

**Physical navigation must not run while these IDs differ.** Load the matching 0911 Kachaka map, or build and validate a new alignment/bundle for `lab_0919`.

Even after the map ID is corrected, manipulation still requires a supervised grasp calibration test because previous physical tests reached and closed the gripper but did not actually retain the object.
