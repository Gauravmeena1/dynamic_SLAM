#!/usr/bin/env python3
"""從 Kachaka 匯出的 2D 地圖自動產生「橋 run」的 traj.csv（蛇行覆蓋路徑）。

為什麼需要這支：橋 run 必須在**一般模式**（載入已存好的地圖）下跑，Kachaka 在
一般模式不會自己探索，所以要餵一條路徑給 run_slam_oneshot.sh 的 robot 模式。
手上的 traj.csv 是舊場地的座標，換場地就得重產。

⚠️ traj.csv 是「形狀」不是絕對座標：follower 預設 --align-start，會把整條軌跡
   剛體變換成「第一個點 = 機器人當下的 odom pose」。所以本工具把第一個點寫成
   機器人**當下的 map pose**，這樣 map frame → odom frame 是同一個剛體變換，
   形狀才會正確落在地圖上。機器人開跑前不要移動，否則整條路徑會平移/旋轉。

⚠️ follower 沒有避障（純 cmd_vel + odom 路徑跟隨，不看雷射）。所以路徑一定要
   自己留安全邊界：free space 先侵蝕 (robot-radius + margin) 才拿來規劃，而且
   每一段連線都會逐點驗證落在安全區內，不安全就整段捨棄。
   跑的時候路上不能有人或臨時障礙物 —— 它會直接撞上去。

⚠️ follower 推進 waypoint 是看**距離**（dist > waypoint-tolerance 才換下一個），
   同一個 x,y 只改 yaw 的點會被直接跳過，所以本工具不產生原地旋轉點。要增加
   點雲涵蓋角度，用 --spacing 調密一點讓機器人多轉幾次彎。

用法：
    python3 make_bridge_traj.py --map-yaml kachaka_2d_map4_20260826.yaml \
        --out traj_bridge_20260826.csv
    # 起點預設用 gRPC 讀機器人當下 map pose；離線測試可用 --start x,y,yaw
"""
import os
import argparse
import csv
import math
import sys
from pathlib import Path

import numpy as np
import yaml
from PIL import Image

FREE, UNKNOWN, WALL = 253, 234, 175


def erode(mask: np.ndarray, r_px: int) -> np.ndarray:
    """二值侵蝕（圓形結構元素）。沒有 scipy，用累積和做 O(1) 方框侵蝕再套圓遮罩。"""
    if r_px <= 0:
        return mask.copy()
    H, W = mask.shape
    out = mask.copy()
    # 圓形結構元素：對每個 dy 算出該列的半寬，逐列 AND
    for dy in range(-r_px, r_px + 1):
        dx = int(math.floor(math.sqrt(max(r_px * r_px - dy * dy, 0))))
        shifted = np.ones_like(mask)
        ys = slice(max(0, -dy), H - max(0, dy))
        yd = slice(max(0, dy), H - max(0, -dy))
        row = mask[ys, :]
        # 該列往左右各 dx 做侵蝕
        acc = row.copy()
        for k in range(1, dx + 1):
            acc[:, k:] &= row[:, :-k]
            acc[:, :-k] &= row[:, k:]
            acc[:, :k] = False
            acc[:, W - k:] = False
        shifted[yd, :] = acc
        if dy > 0:
            shifted[:dy, :] = False
        elif dy < 0:
            shifted[dy:, :] = False
        out &= shifted
    return out


def flood(mask: np.ndarray, seed_rc) -> np.ndarray:
    """回傳與 seed 相連的連通區域（4-連通，BFS，避免遞迴爆棧）。"""
    H, W = mask.shape
    out = np.zeros_like(mask)
    r0, c0 = seed_rc
    if not (0 <= r0 < H and 0 <= c0 < W and mask[r0, c0]):
        return out
    stack = [(r0, c0)]
    out[r0, c0] = True
    while stack:
        r, c = stack.pop()
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            rr, cc = r + dr, c + dc
            if 0 <= rr < H and 0 <= cc < W and mask[rr, cc] and not out[rr, cc]:
                out[rr, cc] = True
                stack.append((rr, cc))
    return out


class MapGrid:
    def __init__(self, yaml_path: Path):
        cfg = yaml.safe_load(yaml_path.read_text())
        self.res = float(cfg["resolution"])
        self.ox, self.oy = float(cfg["origin"][0]), float(cfg["origin"][1])
        self.W, self.H = int(cfg["width"]), int(cfg["height"])
        img = yaml_path.parent / cfg["image"]
        self.gray = np.array(Image.open(img).convert("L"))
        if self.gray.shape != (self.H, self.W):
            raise SystemExit(f"❌ 圖片尺寸 {self.gray.shape} 與 yaml {(self.H, self.W)} 不符")

    # 影像 row 0 在最上方 = y 最大（ROS map_server 慣例，已用機器人實際位置驗證過）
    def world_to_rc(self, x, y):
        return int(round(self.H - 1 - (y - self.oy) / self.res)), int(round((x - self.ox) / self.res))

    def rc_to_world(self, r, c):
        return self.ox + c * self.res, self.oy + (self.H - 1 - r) * self.res


def seg_safe(safe, g: MapGrid, p, q, step=None) -> bool:
    """逐點檢查 p→q 這一段是否全落在安全區。"""
    step = step or g.res * 0.5
    d = math.hypot(q[0] - p[0], q[1] - p[1])
    n = max(2, int(d / step) + 1)
    for i in range(n + 1):
        t = i / n
        r, c = g.world_to_rc(p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1]))
        if not (0 <= r < g.H and 0 <= c < g.W and safe[r, c]):
            return False
    return True


def build_sweeps(safe, g: MapGrid, spacing_m: float):
    """水平掃描線 → 每列取最長的安全區間，回傳 [(y, x_left, x_right), ...] 由下而上。"""
    step_r = max(1, int(round(spacing_m / g.res)))
    out = []
    for r in range(0, g.H, step_r):
        row = safe[r]
        if not row.any():
            continue
        best, cur = None, None
        for c in range(g.W + 1):
            on = c < g.W and row[c]
            if on and cur is None:
                cur = c
            elif not on and cur is not None:
                if best is None or c - cur > best[1] - best[0]:
                    best = (cur, c - 1)
                cur = None
        if best is None:
            continue
        x0, y0 = g.rc_to_world(r, best[0])
        x1, _ = g.rc_to_world(r, best[1])
        if x1 - x0 < spacing_m * 0.5:      # 太短的殘段不值得跑
            continue
        out.append((y0, x0, x1))
    out.sort(key=lambda s: s[0])
    return out


def bfs_path(mask, start_rc, goal_rc):
    """在安全網格上找一條 start→goal 的路徑（8-連通 BFS）。回 [(r,c),...] 或 None。

    只用直線連接兩條掃描線的話，中間隔著家具就整條掃描線被丟掉（實測 0.4m 間距
    掉了 5 條，路徑退化成一個外框）。改成繞路規劃，掃描線才連得起來。
    """
    from collections import deque
    H, W = mask.shape
    if not mask[goal_rc] or not mask[start_rc]:
        return None
    prev = {start_rc: None}
    q = deque([start_rc])
    while q:
        cur = q.popleft()
        if cur == goal_rc:
            path = []
            while cur is not None:
                path.append(cur)
                cur = prev[cur]
            return path[::-1]
        r, c = cur
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == 0 and dc == 0:
                    continue
                nxt = (r + dr, c + dc)
                if (0 <= nxt[0] < H and 0 <= nxt[1] < W
                        and mask[nxt] and nxt not in prev):
                    prev[nxt] = cur
                    q.append(nxt)
    return None


def shortcut(world_pts, safe, g):
    """視線簡化：BFS 出來的是鋸齒狀格點，能直達就跳過中間點。"""
    if len(world_pts) <= 2:
        return world_pts
    out = [world_pts[0]]
    i = 0
    while i < len(world_pts) - 1:
        j = len(world_pts) - 1
        while j > i + 1 and not seg_safe(safe, g, world_pts[i], world_pts[j]):
            j -= 1
        out.append(world_pts[j])
        i = j
    return out


def trace_contour(mask):
    """Moore 邊界追蹤：回傳 mask 最外圈的有序邊界像素 [(r,c),...]（順時針）。"""
    H, W = mask.shape
    start = None
    for r in range(H):
        cs = np.nonzero(mask[r])[0]
        if len(cs):
            start = (r, int(cs[0]))
            break
    if start is None:
        return []
    nb = [(-1, 0), (-1, 1), (0, 1), (1, 1), (1, 0), (1, -1), (0, -1), (-1, -1)]
    contour = [start]
    # start 是「最上面那列最左邊」的像素，所以視為從**西邊**走進來的：
    # backtrack = (prev_dir+4)%8 要等於 6(W) → prev_dir = 2(E)。設錯的話第一次搜尋
    # 就從錯的象限開始，追蹤兩三步就斷掉（實測只回 5 個點）。
    cur, prev_dir = start, 2
    for _ in range(4 * H * W):
        found = False
        for k in range(8):
            d = (prev_dir + 5 + k) % 8       # 從「上一步的反方向 + 1」開始逆時針找
            rr, cc = cur[0] + nb[d][0], cur[1] + nb[d][1]
            if 0 <= rr < H and 0 <= cc < W and mask[rr, cc]:
                cur, prev_dir, found = (rr, cc), d, True
                break
        if not found:
            break
        if cur == start and len(contour) > 2:
            break
        contour.append(cur)
    return contour


def spiral_path(safe, g, spacing, start_rc):
    """等高線平行（向內螺旋）覆蓋路徑。

    為什麼要這個（2026-08-26 使用者回報「載著手臂架純旋轉容易卡住」）：
    蛇行(boustrophedon)每一趟結尾都必須**反向 180°**，那是圖形本身的性質，
    再怎麼圓角也只能把它攤開，實測仍有 4 段導航目標要轉 90–150°。
    螺旋則是一路同向繞行，除了換圈時沒有任何反向 —— 對「不能原地轉」的載具友善得多。
    """
    rings = []
    r_px = 0
    step_px = max(1, int(round(spacing / g.res)))
    while True:
        m = erode(safe, r_px) if r_px else safe.copy()
        if not m.any():
            break
        comp = flood(m, start_rc) if r_px == 0 else None
        if comp is not None and comp.any():
            m = comp
        cont = trace_contour(m)
        if len(cont) < 8:
            break
        rings.append([g.rc_to_world(r, c) for r, c in cont])
        r_px += step_px
        if len(rings) > 40:
            break
    return rings


def bezier(p, ctrl, q, n=20):
    """二次貝茲曲線取樣（不含終點，交給下一段接）。用來把 U 型迴轉做成圓弧。"""
    out = []
    for k in range(n):
        t = k / n
        u = 1 - t
        out.append((u * u * p[0] + 2 * u * t * ctrl[0] + t * t * q[0],
                    u * u * p[1] + 2 * u * t * ctrl[1] + t * t * q[1]))
    return out


def cubic_bezier(p, ctrl1, ctrl2, q, n=24):
    """三次貝茲曲線取樣（含起點、不含終點）。

    U 型迴轉需要兩個控制點，才能讓起點與終點的切線分別貼合兩條反向
    掃描線。單控制點的二次貝茲無法同時滿足這兩個切線條件，末端會留下
    尖折，之後抽稀時只能靠公分級導航點拆開大角度。
    """
    out = []
    for k in range(n):
        t = k / n
        u = 1 - t
        out.append((u ** 3 * p[0] + 3 * u * u * t * ctrl1[0]
                    + 3 * u * t * t * ctrl2[0] + t ** 3 * q[0],
                    u ** 3 * p[1] + 3 * u * u * t * ctrl1[1]
                    + 3 * u * t * t * ctrl2[1] + t ** 3 * q[1]))
    return out


def smooth_corner(prev, vertex, nxt, radius, safe, g):
    """把一個尖角換成圓角（沿兩邊各退 radius，用貝茲接起來）。不安全就回 None。

    為什麼要這個（2026-08-26 使用者回報）：Kachaka 載著手臂架時**純旋轉很容易卡住**，
    所以路徑上的急轉彎越少越好。圓角讓機器人邊走邊轉，不必停下來原地轉。
    """
    def cut(a, b, r):
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        if d < 1e-9:
            return None
        r = min(r, d * 0.45)          # 別把整段邊吃掉
        return (a[0] + (b[0] - a[0]) * r / d, a[1] + (b[1] - a[1]) * r / d)

    p = cut(vertex, prev, radius)
    q = cut(vertex, nxt, radius)
    if p is None or q is None:
        return None
    arc = bezier(p, vertex, q, n=10) + [q]
    for i in range(len(arc) - 1):
        if not seg_safe(safe, g, arc[i], arc[i + 1]):
            return None
    return arc


def round_path(pts, radius, safe, g):
    """對真正的折線尖角做圓角；已平滑曲線的小轉角不再重複處理。"""
    if radius <= 0 or len(pts) < 3:
        return pts, 0
    out = [pts[0]]
    n_round = 0
    for i in range(1, len(pts) - 1):
        incoming = math.atan2(pts[i][1] - pts[i - 1][1],
                              pts[i][0] - pts[i - 1][0])
        outgoing = math.atan2(pts[i + 1][1] - pts[i][1],
                              pts[i + 1][0] - pts[i][0])
        turn = abs((outgoing - incoming + math.pi) % (2 * math.pi) - math.pi)
        d_in = math.hypot(pts[i][0] - pts[i - 1][0],
                          pts[i][1] - pts[i - 1][1])
        d_out = math.hypot(pts[i + 1][0] - pts[i][0],
                           pts[i + 1][1] - pts[i][1])
        # U 型貝茲本身每一小段只有幾度轉角。舊版把這些點逐一再圓角，
        # 相鄰圓角會互相重疊並形成微小回勾。除了角度，也要求兩側線段
        # 足夠長；密集取樣的既有曲線一律不能再做第二次圓角。
        long_enough = min(d_in, d_out) >= radius * 0.5
        arc = (smooth_corner(pts[i - 1], pts[i], pts[i + 1], radius, safe, g)
               if turn >= math.radians(15.0) and long_enough else None)
        if arc:
            out += arc
            n_round += 1
        else:
            out.append(pts[i])
    out.append(pts[-1])
    return out, n_round


def max_turn_deg(dense):
    """回報路徑上最大的單步轉向角（度）—— 用來判斷會不會逼機器人原地轉。"""
    worst = 0.0
    for i in range(1, len(dense) - 1):
        a0 = math.atan2(dense[i][1] - dense[i - 1][1], dense[i][0] - dense[i - 1][0])
        a1 = math.atan2(dense[i + 1][1] - dense[i][1], dense[i + 1][0] - dense[i][0])
        worst = max(worst, abs((a1 - a0 + math.pi) % (2 * math.pi) - math.pi))
    return math.degrees(worst)


def densify(pts, step):
    """把折線加密成固定間距，並用行進方向算 yaw（follower 靠距離推進，不吃原地轉）。"""
    dense = []
    for i in range(len(pts) - 1):
        (x0, y0), (x1, y1) = pts[i], pts[i + 1]
        d = math.hypot(x1 - x0, y1 - y0)
        n = max(1, int(d / step))
        yaw = math.atan2(y1 - y0, x1 - x0)
        for k in range(n):
            t = k / n
            dense.append((x0 + t * (x1 - x0), y0 + t * (y1 - y0), yaw))
    if pts:
        x, y = pts[-1]
        dense.append((x, y, dense[-1][2] if dense else 0.0))
    return dense


def main() -> int:
    ap = argparse.ArgumentParser(
                                 description="從 Kachaka 2D map 產生覆蓋路徑 / Generate a coverage trajectory from a Kachaka 2D map.",
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--map-yaml", required=True, help="Kachaka occupancy-map YAML / 地圖 YAML")
    ap.add_argument("--out", required=True, help="輸出 traj.csv / Output trajectory CSV")
    ap.add_argument("--robot-radius", type=float, default=0.25, help="機器人半徑 / Robot radius in metres (default: 0.25)")
    ap.add_argument("--margin", type=float, default=0.10, help="額外安全邊界 / Extra safety margin in metres (default: 0.10)")
    ap.add_argument("--min-clearance", type=float, default=0.40,
                    help="最小淨空 / Minimum clearance in metres (default: 0.40; must match drive_waypoints.py). "
                         "規劃時保證的最小淨空；"
                         "侵蝕半徑取 max(robot-radius+margin, 本值) —— 兩邊不一致的話，"
                         "螺旋路徑貼著 0.35 m 邊界走，會被 0.40 m 的檢查砍掉一半目標、路徑斷成碎片")
    ap.add_argument("--spacing", type=float, default=0.7, help="掃描線間距 / Sweep-line spacing in metres (default: 0.7)")
    ap.add_argument("--step", type=float, default=0.05, help="Waypoint 間距 / Dense waypoint step in metres (default: 0.05)")
    ap.add_argument("--pattern", choices=("spiral", "boustrophedon"), default="spiral",
                    help="覆蓋圖形 / Coverage pattern: spiral avoids 180-degree reversals; "
                         "boustrophedon gives regular sweep rows")
    ap.add_argument("--turn-radius", type=float, default=0.40,
                    help="迴轉半徑 / Turn radius in metres (default: 0.40); larger is smoother but uses more space")
    ap.add_argument("--no-skip-rows", dest="skip_rows", action="store_false",
                    help="關閉隔行跳掃 / Disable alternating-row skip order")
    ap.add_argument("--start", default=None, help="起點 x,y,yaw / Start pose; omit to read the robot map pose over gRPC")
    ap.add_argument("--endpoint", default=os.getenv("KACHAKA_ENDPOINT", "KACHAKA_IP:26400"),
                    help="Kachaka gRPC endpoint / Kachaka gRPC 端點")
    ap.add_argument("--preview", default=None,
                    help="預覽圖 / Preview image (default: artifacts/previews/<output>_raw.png)")
    args = ap.parse_args()

    if args.preview is None:
        viz = Path(os.getenv("KACHAKA_PREVIEW_DIR", Path(__file__).parent / "artifacts" / "previews"))
        viz.mkdir(parents=True, exist_ok=True)
        args.preview = str(viz / f"{Path(args.out).stem}_raw.png")

    g = MapGrid(Path(args.map_yaml))

    if args.start:
        sx, sy, syaw = [float(v) for v in args.start.split(",")]
    else:
        # kachaka_api 的位置：先找自己家目錄，再退回 acm 的（本機唯一一份），
        # 也可用環境變數 PYTHONPATH 覆蓋。原版只看 ~/robotic，非 acm 帳號會 ModuleNotFoundError。
        for _p in (Path.home() / "robotic/robotic_system/kachaka-main/kachaka-api/python",
                   Path(os.getenv("KACHAKA_API_PATH", "/opt/kachaka/kachaka-api/python"))):
            if _p.is_dir():
                sys.path.insert(0, str(_p))
        import kachaka_api
        p = kachaka_api.KachakaApiClient(target=args.endpoint).get_robot_pose()
        sx, sy, syaw = p.x, p.y, p.theta
    print(f"起點 map pose: ({sx:.3f}, {sy:.3f}, {syaw:.3f})")

    free = g.gray == FREE
    clear_m = max(args.robot_radius + args.margin, args.min_clearance)
    r_px = int(round(clear_m / g.res))
    safe = erode(free, r_px)
    print(f"free {free.sum()*g.res**2:.2f} m² → 侵蝕 {r_px}px ({clear_m:.2f} m 淨空) 後 "
          f"{safe.sum()*g.res**2:.2f} m²")

    sr, sc = g.world_to_rc(sx, sy)
    if not (0 <= sr < g.H and 0 <= sc < g.W):
        return err("機器人不在地圖範圍內")
    if not safe[sr, sc]:
        # 起點離牆太近：往外找最近的安全格當作實際起跑點
        ys, xs = np.nonzero(safe)
        if len(ys) == 0:
            return err(f"侵蝕後沒有任何安全區（robot-radius+margin={args.robot_radius+args.margin:.2f}m 太大？）")
        i = np.argmin((ys - sr) ** 2 + (xs - sc) ** 2)
        sr, sc = int(ys[i]), int(xs[i])
        nx, ny = g.rc_to_world(sr, sc)
        print(f"⚠️ 起點距離障礙物太近，改從最近的安全點 ({nx:.2f}, {ny:.2f}) 起跑"
              f"（離原位置 {math.hypot(nx-sx, ny-sy):.2f} m）")
        # ⚠️ 世界座標一定要跟著更新（2026-08-26 踩過）：只改格點 sr/sc 的話，
        #    路徑第一個點仍在安全區外，後面每一段 BFS 都從區外出發 → 全部回 None
        #    → 每一圈/每一條掃描線都被 continue 掉，最後只剩 1 個 waypoint 而且不報錯。
        sx, sy = nx, ny

    comp = flood(safe, (sr, sc))
    print(f"與起點相連的可走區域: {comp.sum()*g.res**2:.2f} m²")

    if args.pattern == "spiral":
        rings = spiral_path(comp, g, args.spacing, (sr, sc))
        if not rings:
            return err("產生不出等高線 —— 可走區域太窄，試著調小 --robot-radius/--margin")
        pts = [(sx, sy)]
        for ring in rings:
            # 從離當下位置最近的那一點切入這一圈，然後沿著圈走完
            k = min(range(len(ring)), key=lambda j: math.hypot(ring[j][0] - pts[-1][0],
                                                               ring[j][1] - pts[-1][1]))
            ordered = ring[k:] + ring[:k]
            if not seg_safe(comp, g, pts[-1], ordered[0]):
                leg = bfs_path(comp, g.world_to_rc(*pts[-1]), g.world_to_rc(*ordered[0]))
                if leg is None:
                    continue
                pts += shortcut([g.rc_to_world(r, c) for r, c in leg], comp, g)[1:-1]
            pts += ordered
        print(f"等高線 {len(rings)} 圈（螺旋，無 180° 反向）")
        return finish(args, g, comp, pts, sx, sy, syaw)

    sweeps = build_sweeps(comp, g, args.spacing)
    if not sweeps:
        return err("產生不出掃描線 —— 可走區域太窄，試著調小 --robot-radius/--margin 或 --spacing")

    # 蛇行：由離起點最近的那條掃描線開始，逐條上行；每次連線都驗證安全
    # 掃描線順序：預設「隔行跳掃」（0,2,4,… 再 …,5,3,1）。
    # 相鄰兩趟相隔 2×spacing，迴轉半徑加倍 —— 農機的 skip-row 就是這樣避免急迴轉的。
    # 這是為了 Kachaka 載著手臂架時**純旋轉容易卡住**（2026-08-26 使用者回報）。
    idx = sorted(range(len(sweeps)), key=lambda i: sweeps[i][0])
    if args.skip_rows and len(idx) >= 4:
        seq = idx[0::2] + idx[1::2][::-1]
    else:
        seq = idx
    # 從離機器人最近的那一趟開始（把序列旋轉過去，不改變隔行結構）
    near = min(range(len(seq)), key=lambda k: abs(sweeps[seq[k]][0] - sy))
    seq = seq[near:] + seq[:near]

    R = args.turn_radius
    pts = [(sx, sy)]
    first_sweep = True
    dropped = 0
    arcs = 0
    for i in seq:
        y, xl, xr = sweeps[i]
        # 兩端各內縮一個迴轉半徑，讓 U 型迴轉的圓弧能落在安全區內
        ixl, ixr = xl + R, xr - R
        if ixr - ixl < args.spacing * 0.5:       # 內縮後太短就不內縮
            ixl, ixr = xl, xr
        # 從**離當下位置較近的那一端**進入，而不是硬性左右交替 ——
        # 交替式在第一趟就可能逼機器人掉頭 180°（實測 173.6°），載著手臂架時正是會卡住的動作。
        cx = pts[-1][0]
        a, b = ((ixl, y), (ixr, y)) if abs(cx - ixl) <= abs(cx - ixr) else ((ixr, y), (ixl, y))
        if first_sweep and ixl <= sx <= ixr:
            # If the start already lies within the first sweep's x span, do
            # not hook back to the nearest endpoint and immediately reverse.
            # That creates a 120-180 degree hairpin within centimetres, which
            # made the loaded Kachaka circle without converging. Join at least
            # R metres ahead on the side most aligned with its current yaw.
            candidates = []
            for sign, end_x in ((1.0, ixr), (-1.0, ixl)):
                entry_x = min(ixr, max(ixl, sx + sign * R))
                heading = math.atan2(y - sy, entry_x - sx)
                delta = abs((heading - syaw + math.pi) % (2 * math.pi) - math.pi)
                candidates.append((delta, (entry_x, y), (end_x, y)))
            _, a, b = min(candidates, key=lambda item: item[0])
        if not seg_safe(comp, g, a, b):
            dropped += 1
            continue
        first_sweep = False
        # 連接：優先用圓弧 U 迴轉，不安全再退回直線 / BFS 繞路
        joined = False
        if len(pts) > 1:
            # 用三次貝茲和兩個控制點，起終點切線才能同時貼合兩條反向掃描線。
            # 控制點外推 4R/3 時，中點弧高約為 R，可近似半圓 U-turn。
            h_in = math.atan2(pts[-1][1] - pts[-2][1],
                              pts[-1][0] - pts[-2][0])
            h_out = math.atan2(b[1] - a[1], b[0] - a[0])
            # 弧撞到牆就把半徑縮小重試，不要一次失敗就退回尖角
            # （7 條掃描線的 6 個迴轉，全尺寸只過 4 個）。小一點的弧仍遠勝直角。
            for scale in (1.0, 0.75, 0.5, 0.35):
                lead = (4.0 / 3.0) * R * scale
                # Use the actual incoming and outgoing tangents. Inferring the
                # side from endpoint x positions fails on short/irregular scan
                # lines and can reverse the first control point, creating a
                # near-zero-speed cusp instead of a U-turn.
                ctrl1 = (pts[-1][0] + lead * math.cos(h_in),
                         pts[-1][1] + lead * math.sin(h_in))
                ctrl2 = (a[0] - lead * math.cos(h_out),
                         a[1] - lead * math.sin(h_out))
                curve = cubic_bezier(pts[-1], ctrl1, ctrl2, a, n=24) + [a]
                local_turns = []
                for k in range(1, len(curve) - 1):
                    h0 = math.atan2(curve[k][1] - curve[k - 1][1],
                                    curve[k][0] - curve[k - 1][0])
                    h1 = math.atan2(curve[k + 1][1] - curve[k][1],
                                    curve[k + 1][0] - curve[k][0])
                    local_turns.append(abs((h1 - h0 + math.pi) % (2 * math.pi) - math.pi))
                smooth_enough = max(local_turns, default=0.0) <= math.radians(45.0)
                if (smooth_enough
                        and all(seg_safe(comp, g, curve[k], curve[k + 1])
                                for k in range(len(curve) - 1))):
                    pts += curve[1:]
                    arcs += 1
                    joined = True
                    break
        if not joined:
            if not seg_safe(comp, g, pts[-1], a):
                leg = bfs_path(comp, g.world_to_rc(*pts[-1]), g.world_to_rc(*a))
                if leg is None:
                    dropped += 1
                    continue
                pts += shortcut([g.rc_to_world(r, c) for r, c in leg], comp, g)[1:-1]
            pts.append(a)
        pts.append(b)
    if dropped:
        print(f"⚠️ 有 {dropped} 條掃描線連不過去被捨棄（繞路也走不到）")
    if len(pts) < 3:
        return err("安全的掃描線太少，產不出有意義的路徑")
    print(f"掃描線 {len(seq)} 條（{'隔行跳掃' if args.skip_rows else '逐行'}），"
          f"圓弧迴轉 {arcs} 次")

    return finish(args, g, comp, pts, sx, sy, syaw)


def finish(args, g, comp, pts, sx, sy, syaw):
    """共用收尾：圓角 → 加密 → 寫檔 → 出預覽圖。"""
    R = args.turn_radius
    pts, n_round = round_path(pts, R, comp, g)
    if n_round:
        print(f"圓角處理 {n_round} 個轉折點（半徑 {R} m）")

    dense = densify(pts, args.step)
    length = sum(math.hypot(dense[i+1][0]-dense[i][0], dense[i+1][1]-dense[i][1])
                 for i in range(len(dense)-1))
    dense[0] = (dense[0][0], dense[0][1], syaw)   # 第一個點的 yaw 要是機器人當下朝向
    print(f"路徑最大單步轉向 {max_turn_deg(dense):.1f}°"
          f"（越小越不會逼機器人原地轉）")

    out = Path(args.out)
    with out.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["idx", "x", "y", "yaw"])
        for i, (x, y, yaw) in enumerate(dense):
            w.writerow([i, f"{x:.5f}", f"{y:.5f}", f"{yaw:.5f}"])
    print(f"✅ {out}：{len(dense)} 個 waypoint，路徑長 {length:.1f} m")

    prev = Path(args.preview or (str(out) + ".png"))
    rgb = np.dstack([g.gray] * 3).astype(np.uint8)
    rgb[comp] = (np.array([200, 255, 200]) * 0.5 + rgb[comp] * 0.5).astype(np.uint8)
    for x, y, _ in dense:
        r, c = g.world_to_rc(x, y)
        if 0 <= r < g.H and 0 <= c < g.W:
            rgb[r, c] = (255, 0, 0)
    r, c = g.world_to_rc(sx, sy)
    rgb[max(0, r-2):r+3, max(0, c-2):c+3] = (0, 0, 255)
    Image.fromarray(rgb).resize((g.W * 3, g.H * 3), Image.NEAREST).save(prev)
    print(f"   預覽圖 {prev}（紅=路徑 藍=起點 淺綠=可走區域）—— 跑之前務必打開看一眼")
    return 0


def err(msg):
    print(f"❌ {msg}")
    return 1


if __name__ == "__main__":
    sys.exit(main())
