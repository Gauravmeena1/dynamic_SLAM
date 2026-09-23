#!/usr/bin/env python3
"""用 Kachaka 自己的導航 API（move_to_pose）逐點走完一條 traj.csv —— 橋 run 專用。

為什麼不用 run_slam_oneshot.sh 的 robot 模式（2026-08-26 實機發現）：
  robot 模式是 replay_and_collect_ros2.py 的 follower 自己算速度、直接發
  /kachaka/manual_control/cmd_vel。**載著手臂架（Kachaka 認定為家具 S01）時，
  原始速度指令的旋轉會被 Kachaka 擋掉**：實測 follower 持續送 angular.z=0.6
  （最大轉速）、linear.x=0，機器人 odom yaw 變化 < 0.0001 rad 完全不轉，
  於是 heading_error 永遠降不下來 → 卡在「原地轉向」狀態直到逾時，整趟報廢。

  而 robotic_agent / mm_system 一直都是載著同一個手臂架在跑的，因為它們走的是
  `client.move_to_pose(x, y, yaw)`（robot_driver.py:86、nav_to_instance.py:122）
  —— Kachaka 自己的規劃器，載貨狀態下正常運作。這支就是改走那條路。

  附帶好處：move_to_pose 有 Kachaka 自己的避障，cmd_vel follower 完全沒有。

用法（機器人要在**一般模式**、已載入目標地圖；座標是該地圖的 map frame）：
    python3 drive_waypoints.py traj_bridge_20260826.csv            # 先看抽稀結果，不動
    python3 drive_waypoints.py traj_bridge_20260826.csv --go       # 真的跑

⚠️ 這支會讓機器人實際移動。--go 之前先看一次抽稀出來的目標點清單。
⚠️ 目標點離障礙物太近時 Kachaka 自己的避障會擋住、move_to_pose 會阻塞不返回
   （ed305_check_anchors.py:93 實測）。所以本工具會用地圖檢查每個目標點的淨空，
   不足的直接剔除，不要讓它跑到一半卡死。
"""
import os
import argparse
import csv
import math
import queue
import sys
import threading
import time
from pathlib import Path

import numpy as np
import yaml
from PIL import Image

FREE = 253


def load_traj(path):
    with open(path, encoding="utf-8") as f:
        return [(float(r["x"]), float(r["y"]), float(r["yaw"])) for r in csv.DictReader(f)]


def tangents(pts):
    """每個點的路徑切線方向（中央差分；端點用單側）。比「到下一個保留點的弦向量」穩定太多。"""
    n = len(pts)
    out = []
    for i in range(n):
        a = pts[max(i - 1, 0)]
        b = pts[min(i + 1, n - 1)]
        out.append(math.atan2(b[1] - a[1], b[0] - a[0]))
    return out


def thin(pts, min_step, goal_spacing, flat_turn=0.09, max_yaw_step_deg=70.0):
    """沿**弧長等距**重採樣成導航目標，朝向取路徑切線。

    ⚠️ 不要用「到下一個保留點的弦向量」判斷轉向（2026-08-26 連踩兩次）：
      · 用單點局部轉角 → 圓弧上每步只轉幾度全被丟掉，整段弧塌成 1 個目標 → 相鄰目標差 110°。
      · 改用累積弦向量 + 最小間距門檻 → 弧上的點被整批跳過，一個目標橫跨整個迴轉 → 差 165°。
      弦向量在密集點上本來就會亂跳。等距重採樣 + 切線朝向則有數學保證：
      相鄰目標的朝向差 ≈ goal_spacing / 迴轉半徑，跟點的疏密無關。
      （0.25 m 間距、0.4 m 半徑 → 約 36°；要更平緩就把 goal_spacing 調小。）

    直線段上切線幾乎不變，就把目標合併到 min_step，不要產生一堆無意義的中繼點。
    """
    if len(pts) < 2:
        return [(p[0], p[1], p[2]) for p in pts]
    tan = tangents(pts)

    # 1) 沿弧長等距取樣
    picked = [0]
    acc = 0.0
    for i in range(1, len(pts)):
        acc += math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])
        if acc >= goal_spacing:
            picked.append(i)
            acc = 0.0
    if picked[-1] != len(pts) - 1:
        picked.append(len(pts) - 1)

    # 2) 直線段合併：切線沒怎麼變、又還沒走到 min_step，就不必留成目標
    keep = [picked[0]]
    for i in picked[1:-1]:
        j = keep[-1]
        d = math.hypot(pts[i][0] - pts[j][0], pts[i][1] - pts[j][1])
        turned = abs((tan[i] - tan[j] + math.pi) % (2 * math.pi) - math.pi)
        if turned < flat_turn and d < min_step:
            continue
        keep.append(i)
    keep.append(picked[-1])

    # min_step can otherwise skip across an entire rounded turn. Restore just
    # enough raw-path points to bound every commanded final-yaw change.
    limit = math.radians(max_yaw_step_deg)
    refined = [keep[0]]
    for target in keep[1:]:
        i = refined[-1] + 1
        while i <= target:
            turned = abs(
                (tan[i] - tan[refined[-1]] + math.pi)
                % (2 * math.pi) - math.pi
            )
            if turned > limit:
                candidate = max(refined[-1] + 1, i - 1)
                if candidate != refined[-1]:
                    refined.append(candidate)
                if candidate == i:
                    i += 1
                continue
            i += 1
        if refined[-1] != target:
            refined.append(target)

    goals = [(pts[i][0], pts[i][1], tan[i]) for i in refined]
    # make_bridge_traj deliberately stores the robot's measured heading in the
    # first raw point. Replacing it with the one-sided tangent can command a
    # large in-place turn for a centimetre-scale connector, followed by the
    # opposite turn at the next goal. That made a loaded Kachaka circle around
    # an otherwise reachable goal. Preserve the measured initial heading.
    goals[0] = (goals[0][0], goals[0][1], pts[0][2])
    return goals


class Clearance:
    """用 2D 地圖算某個點離最近非 free 像素的距離，篩掉會被避障擋住的目標點。"""

    def __init__(self, yaml_path):
        cfg = yaml.safe_load(Path(yaml_path).read_text(encoding="utf-8"))
        self.res = float(cfg["resolution"])
        self.ox, self.oy = float(cfg["origin"][0]), float(cfg["origin"][1])
        self.W, self.H = int(cfg["width"]), int(cfg["height"])
        gray = np.array(Image.open(Path(yaml_path).parent / cfg["image"]).convert("L"))
        self.blocked_rc = np.array(np.nonzero(gray != FREE))   # (2, N)

    def of(self, x, y):
        r = self.H - 1 - (y - self.oy) / self.res
        c = (x - self.ox) / self.res
        d = np.hypot(self.blocked_rc[0] - r, self.blocked_rc[1] - c).min()
        return d * self.res


def move_with_timeout(client, x, y, yaw, timeout_s):
    """Run blocking move_to_pose with a deadline and cancel on timeout."""
    result_q = queue.Queue(maxsize=1)

    def worker():
        try:
            value = client.move_to_pose(
                x, y, yaw, wait_for_completion=True
            )
            result_q.put((True, value))
        except BaseException as exc:
            result_q.put((False, exc))

    # The worker must not keep the process alive if the gRPC call itself fails
    # to unwind after cancel_command().
    threading.Thread(target=worker, daemon=True).start()
    try:
        succeeded, value = result_q.get(timeout=timeout_s)
    except queue.Empty:
        client.cancel_command()
        raise TimeoutError(f"move_to_pose 超過 {timeout_s:.0f} 秒")
    if not succeeded:
        raise value
    return value


def wait_for_command_stop(client, timeout_s=10.0):
    """Wait until a cancelled Kachaka command is no longer running."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if not client.is_command_running():
            return True
        time.sleep(0.2)
    return not client.is_command_running()


def minimum_goal_distance(goals):
    """相鄰導航目標的最短直線距離；少於兩點時回傳無限大。"""
    return min(
        (math.hypot(goals[i][0] - goals[i - 1][0],
                    goals[i][1] - goals[i - 1][1])
         for i in range(1, len(goals))),
        default=math.inf,
    )


def orient_goals_for_arrival(goals):
    """Set each intermediate goal yaw to its incoming travel direction.

    A loaded Kachaka can reach an intermediate position but then wait forever
    trying to perform the final in-place yaw correction.  The geometric
    thinning still uses dense-path tangents; only the yaw sent to move_to_pose
    is changed so arrival does not require a second rotation.  The first goal
    keeps the measured robot heading.
    """
    if len(goals) < 2:
        return list(goals)
    out = [goals[0]]
    for i in range(1, len(goals)):
        x, y, _ = goals[i]
        px, py, _ = goals[i - 1]
        if math.hypot(x - px, y - py) < 1e-9:
            yaw = out[-1][2]
        else:
            yaw = math.atan2(y - py, x - px)
        out.append((x, y, yaw))
    return out


def main() -> int:
    ap = argparse.ArgumentParser(
                                 description="用 Kachaka navigation API 執行 traj.csv / Execute a trajectory with the Kachaka navigation API.",
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("traj", help="輸入 traj.csv，map frame / Input trajectory CSV in map frame")
    ap.add_argument("--go", action="store_true", help="真的移動 / Move the robot; omit for dry-run output")
    ap.add_argument("--endpoint", default=os.getenv("KACHAKA_ENDPOINT", "KACHAKA_IP:26400"),
                    help="Kachaka gRPC endpoint / Kachaka gRPC 端點")
    ap.add_argument("--min-step", type=float, default=1.0, help="直線段目標最小間隔 / Minimum straight-segment goal spacing in metres")
    ap.add_argument("--goal-spacing", type=float, default=0.4,
                    help="沿弧長取樣間距 / Arc-length goal spacing in metres (default: 0.4); smaller gives smoother turns")
    ap.add_argument("--max-yaw-step-deg", type=float, default=145.0,
                    help="最大 yaw 變化 / Maximum yaw change between goals in degrees (default: 145)")
    ap.add_argument("--min-goal-distance", type=float, default=0.15,
                    help="最小目標距離 / Minimum Euclidean goal distance in metres (default: 0.15)")
    ap.add_argument("--map-yaml", default=None,
                    help="淨空檢查用 map YAML / Map YAML for clearance checks (strongly recommended)")
    ap.add_argument("--min-clearance", type=float, default=0.40,
                    help="障礙物最小距離 / Minimum obstacle clearance in metres")
    ap.add_argument("--settle", type=float, default=1.0, help="到位停留秒數 / Camera settling time at each goal")
    ap.add_argument("--goal-timeout", type=float, default=30.0,
                    help="單點 timeout / Per-goal timeout in seconds (default: 30; cancels before continuing)")
    ap.add_argument("--save-goals", default=None,
                    help="另存導航目標 CSV / Save thinned goals for preview_traj.py")
    args = ap.parse_args()

    pts = load_traj(args.traj)
    goals = thin(pts, args.min_step, args.goal_spacing,
                 max_yaw_step_deg=args.max_yaw_step_deg)
    print(f"{args.traj}: {len(pts)} 個原始點 → 抽稀成 {len(goals)} 個導航目標")

    if args.map_yaml:
        cl = Clearance(args.map_yaml)
        kept, dropped = [], 0
        for g in goals:
            if cl.of(g[0], g[1]) >= args.min_clearance:
                kept.append(g)
            else:
                dropped += 1
        if dropped:
            print(f"⚠️ {dropped} 個目標點淨空不足 {args.min_clearance} m 被剔除"
                  f"（太近會讓 move_to_pose 阻塞不返回）")
        goals = kept
    else:
        print("⚠️ 沒給 --map-yaml，略過淨空檢查 —— 目標點若太靠近障礙物會卡住")

    goals = orient_goals_for_arrival(goals)

    if not goals:
        print("❌ 沒有可用的目標點")
        return 1

    min_goal_distance = minimum_goal_distance(goals)
    if min_goal_distance < args.min_goal_distance:
        print(f"❌ 相鄰導航目標最短只有 {min_goal_distance:.3f} m，"
              f"低於限制 {args.min_goal_distance:.3f} m；拒絕執行以避免原地頻繁轉向")
        return 1

    yaw_steps = [
        abs((goals[i][2] - goals[i - 1][2] + math.pi) % (2 * math.pi) - math.pi)
        for i in range(1, len(goals))
    ]
    max_yaw_deg = math.degrees(max(yaw_steps, default=0.0))
    if max_yaw_deg > args.max_yaw_step_deg + 1.0:
        print(f"❌ 淨空篩選後最大朝向跳變 {max_yaw_deg:.1f}°，"
              f"超過限制 {args.max_yaw_step_deg:.1f}°")
        return 1

    total = sum(math.hypot(goals[i + 1][0] - goals[i][0], goals[i + 1][1] - goals[i][1])
                for i in range(len(goals) - 1))
    print(f"路徑長 {total:.1f} m；最短目標間距 {min_goal_distance:.2f} m；"
          f"最大相鄰朝向差 {max_yaw_deg:.1f}°\n")
    for i, (x, y, yaw) in enumerate(goals):
        print(f"  [{i:2d}] ({x:+.3f}, {y:+.3f})  yaw {math.degrees(yaw):+7.1f}°")

    if args.save_goals:
        with open(args.save_goals, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["idx", "x", "y", "yaw"])
            for i, (x, y, yaw) in enumerate(goals):
                w.writerow([i, f"{x:.5f}", f"{y:.5f}", f"{yaw:.5f}"])
        print(f"\n目標點另存 → {args.save_goals}")

    if not args.go:
        print("\n（未加 --go，沒有移動任何東西。確認上面的點沒問題再加 --go 跑）")
        return 0

    # kachaka_api 的位置：先找自己家目錄，再退回 acm 的（本機唯一一份），
    # 也可用環境變數 PYTHONPATH 覆蓋。原版只看 ~/robotic，非 acm 帳號會 ModuleNotFoundError。
    for _p in (Path.home() / "robotic/robotic_system/kachaka-main/kachaka-api/python",
               Path(os.getenv("KACHAKA_API_PATH", "/opt/kachaka/kachaka-api/python"))):
        if _p.is_dir():
            sys.path.insert(0, str(_p))
    import kachaka_api
    c = kachaka_api.KachakaApiClient(target=args.endpoint)

    shelf = c.get_moving_shelf_id()
    print(f"\n載貨狀態 get_moving_shelf_id = {shelf!r}"
          f"{'（載著手臂架 —— 這正是要用 move_to_pose 而不是 cmd_vel 的原因）' if shelf else ''}")
    p = c.get_robot_pose()
    print(f"起始位姿 ({p.x:+.3f}, {p.y:+.3f}) yaw {math.degrees(p.theta):+.1f}°")
    print("開始導航（Ctrl+C 中止；中止後機器人會停在當下位置）\n")

    t0 = time.time()
    ok = 0
    timed_out = 0
    try:
        for i, (x, y, yaw) in enumerate(goals):
            print(f"  [{i:2d}/{len(goals)-1}] → ({x:+.3f}, {y:+.3f}) "
                  f"yaw {math.degrees(yaw):+7.1f}° ... ", end="", flush=True)
            ts = time.time()
            try:
                res = move_with_timeout(c, x, y, yaw, args.goal_timeout)
            except TimeoutError as exc:
                timed_out += 1
                print(f"\n     ⚠️ {exc}；已取消這個點，準備跳到下一點")
                if not wait_for_command_stop(c):
                    print("❌ 取消後 10 秒命令仍在執行；為安全起見中止整條路線")
                    return 124
                if args.settle > 0:
                    time.sleep(args.settle)
                continue
            p = c.get_robot_pose()
            err = math.hypot(x - p.x, y - p.y)
            success = getattr(res, "success", None)
            print(f"{time.time()-ts:5.1f}s  誤差 {err:.3f} m"
                  f"{'' if success is None else ('  ✓' if success else f'  ✗ {res}')}")
            if success is False:
                print("     ⚠️ 這個點沒到位（避障擋住？被家具佔住？）—— 繼續下一個")
            else:
                ok += 1
            if args.settle > 0:
                time.sleep(args.settle)
    except KeyboardInterrupt:
        print("\n\n收到 Ctrl+C，取消目前導航命令")
        try:
            c.cancel_command()
        except Exception as exc:
            print(f"⚠️ 取消命令失敗：{exc}")
        return 130
    except Exception as exc:
        try:
            c.cancel_command()
        except Exception:
            pass
        print(f"\n❌ 導航 API 失敗：{exc}")
        return 1

    print(f"\n完成 {ok}/{len(goals)} 個目標點，逾時跳過 {timed_out} 點，"
          f"耗時 {(time.time()-t0)/60:.1f} 分鐘")
    return 0


if __name__ == "__main__":
    sys.exit(main())
