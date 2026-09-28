#!/usr/bin/env python3
"""kachaka_ctl.py — small Kachaka control/status tool for the mapping workflow.

    kachaka_ctl.py status          map, pose, shelf, battery, running command, errors, locations
    kachaka_ctl.py cancel          cancel the running command (robot stops where it is)
    kachaka_ctl.py home            return to the charging dock          (ROBOT MOVES)
    kachaka_ctl.py goto X Y YAW    drive to map pose X Y (m), YAW (rad)  (ROBOT MOVES)
    kachaka_ctl.py front [FILE]    save the robot's front-camera image (default ./kachaka_front.jpg)

Run with the Python that has kachaka_api's dependencies (KACHAKA_PYTHON in .env):
    $KACHAKA_PYTHON slam_tools/kachaka_ctl.py status
Robot endpoint and SDK path come from .env (KACHAKA_ENDPOINT, KACHAKA_API_PATH).
"""
import os
import sys


def _load_repo_env():
    """Read the repo's .env (KEY=VALUE lines) without overriding variables already set."""
    p = os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", ".env")
    if os.path.isfile(p):
        for line in open(p):
            line = line.strip()
            if line and not line.startswith("#") and "=" in line:
                k, v = line.split("=", 1)
                os.environ.setdefault(k.strip(), os.path.expandvars(v.strip().strip("'\"")))


_load_repo_env()


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else "status"
    if cmd in ("-h", "--help", "help"):
        print(__doc__)
        return
    ep = os.environ.get("KACHAKA_ENDPOINT", "")
    if not ep or ep.startswith("KACHAKA_IP"):
        sys.exit("KACHAKA_ENDPOINT is not set (put it in .env)")
    sys.path.insert(0, os.environ.get("KACHAKA_API_PATH", "/opt/kachaka/kachaka-api/python"))
    import kachaka_api  # the Kachaka SDK; imported here so --help works without it
    _run(cmd, kachaka_api.KachakaApiClient(target=ep))


def _run(cmd, c):
    if cmd == "status":
        cur = c.get_current_map_id()
        name = next((m.name for m in c.get_map_list() if m.id == cur), "?")
        p = c.get_robot_pose()
        lvl, st = c.get_battery_info()
        print(f"map      : {name} ({cur[:8]})")
        print(f"pose     : x={p.x:+.2f} y={p.y:+.2f} yaw={p.theta:+.2f} rad")
        print(f"shelf    : {c.get_moving_shelf_id() or '-'}")
        print(f"battery  : {lvl:.0f}% (status {st})")
        print(f"command  : {'RUNNING' if c.is_command_running() else 'idle'}")
        print(f"last     : {str(c.get_last_command_result()).replace(chr(10), ' ')}")
        print(f"errors   : {c.get_error() or 'none'}")
        for L in c.get_locations():
            print(f"location : {L.name} [{L.id}] x={L.pose.x:+.2f} y={L.pose.y:+.2f}")
    elif cmd == "cancel":
        print("cancel:", c.cancel_command())
    elif cmd == "home":
        print("return home (robot moves) ...")
        print(c.return_home())
    elif cmd == "goto":
        x, y, yaw = (float(v) for v in sys.argv[2:5])
        print(f"move_to_pose({x:+.2f}, {y:+.2f}, {yaw:+.2f}) (robot moves) ...")
        print(c.move_to_pose(x, y, yaw))
        p = c.get_robot_pose()
        print(f"now at x={p.x:+.2f} y={p.y:+.2f} yaw={p.theta:+.2f}")
    elif cmd == "front":
        out = sys.argv[2] if len(sys.argv) > 2 else "kachaka_front.jpg"
        open(out, "wb").write(c.get_front_camera_ros_compressed_image().data)
        print("saved", out)
    else:
        print(__doc__)
        sys.exit(2)


if __name__ == "__main__":
    main()
