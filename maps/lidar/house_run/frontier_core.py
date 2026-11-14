#!/usr/bin/env python3
"""Map-based frontier explorer core (pure numpy, no ROS) for Rover's house run.

Why this exists: Rover's old explore was purely reactive (VFH steer from the L8/VL53 ToF + random
wander). The only "memory" was a heading histogram (rover_local_memory.py, IMU-yaw bins that decay in
45 s, and in arbiter mode its output never reaches /cmd_vel; rover_behaviour also kept a heading
histogram it never used). Nothing knew WHERE in the house Rover had been, so it looped.

This uses the slam_toolbox occupancy grid instead:
  * frontier = known-free cell touching unknown space; clustered; tiny specks ignored
  * Dijkstra over free space (occupied cells inflated by Rover's half-width) from Rover's map pose
  * goal = cheapest frontier cluster (path length, minus a bonus for big openings, plus a penalty
    for frontiers in places Rover has already driven, plus a little for turning around)
  * blacklisted goals (no progress / unreachable) are skipped for a while
  * output = a look-ahead point ~0.8 m along the path; rover_behaviour steers toward it while its
    own ToF / lidar guards keep every hard stop.
"""
from __future__ import annotations

import heapq
import math
import os

import numpy as np


def _e(k, d):
    try:
        return type(d)(os.environ.get(k, d))
    except (TypeError, ValueError):
        return d


OCC_T = 50            # >= this = occupied (slam_toolbox writes 0 / 100 / -1)
SQ2 = math.sqrt(2.0)


def disk_offsets(r_cells: int):
    out = []
    for dy in range(-r_cells, r_cells + 1):
        for dx in range(-r_cells, r_cells + 1):
            if dx * dx + dy * dy <= r_cells * r_cells:
                out.append((dy, dx))
    return out


def dilate(mask: np.ndarray, r_cells: int) -> np.ndarray:
    if r_cells <= 0:
        return mask.copy()
    h, w = mask.shape
    p = np.pad(mask, r_cells, mode="constant", constant_values=False)
    out = np.zeros_like(mask)
    for dy, dx in disk_offsets(r_cells):
        out |= p[r_cells + dy:r_cells + dy + h, r_cells + dx:r_cells + dx + w]
    return out


def neighbours_count(mask: np.ndarray) -> np.ndarray:
    h, w = mask.shape
    p = np.pad(mask.astype(np.int8), 1)
    c = np.zeros((h, w), np.int8)
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dy or dx:
                c += p[1 + dy:1 + dy + h, 1 + dx:1 + dx + w]
    return c


class FrontierPlanner:
    def __init__(self):
        self.HARD_M = _e("FRONTIER_INFLATE_M", 0.16)      # Rover half-width + margin: no path centre closer to a wall
        self.SOFT_M = _e("FRONTIER_SOFT_M", 0.40)         # prefer paths that keep this clear of walls
        self.MIN_CELLS = _e("FRONTIER_MIN_CELLS", 6)      # 6 x 5 cm = 0.3 m opening at least
        self.GAIN_W = _e("FRONTIER_GAIN_W", 0.6)          # m of path a 1 m wide opening is worth
        self.VISIT_W = _e("FRONTIER_VISIT_W", 2.5)        # m penalty for a frontier fully inside where Rover has driven
        self.TURN_W = _e("FRONTIER_TURN_W", 0.4)          # m penalty for a goal directly behind
        self.LOOKAHEAD_M = _e("FRONTIER_LOOKAHEAD_M", 0.8)
        self.KEEP_MARGIN_M = _e("FRONTIER_KEEP_MARGIN_M", 0.8)  # hysteresis: keep current goal unless new one is this much better
        self.BLACK_R_M = _e("FRONTIER_BLACKLIST_R_M", 0.6)

    # ---------------------------------------------------------------- grid helpers
    @staticmethod
    def world_to_cell(x, y, res, ox, oy):
        return int(math.floor((x - ox) / res)), int(math.floor((y - oy) / res))

    @staticmethod
    def cell_to_world(ix, iy, res, ox, oy):
        return ox + (ix + 0.5) * res, oy + (iy + 0.5) * res

    def layers(self, grid: np.ndarray, res: float):
        unknown = grid < 0
        occ = grid >= OCC_T
        free = (~unknown) & (~occ)
        # unknown specks fully surrounded by free space are SLAM noise, not frontiers
        speck = unknown & (neighbours_count(free) >= 6)
        unknown_f = unknown & ~speck
        free_f = free | speck
        hard = dilate(occ, max(1, int(round(self.HARD_M / res))))
        soft_r = max(int(round(self.SOFT_M / res)), int(round(self.HARD_M / res)) + 1)
        soft = dilate(occ, soft_r) & ~hard
        return unknown_f, occ, free_f, hard, soft

    # ---------------------------------------------------------------- planning
    def dijkstra(self, free, hard, soft, start, res):
        """Distances (m) over free cells. Hard-inflated free cells are allowed at 6x cost so Rover can
        always plan out of a tight spot it is already in; unknown/occupied are never traversed."""
        h, w = free.shape
        INF = float("inf")
        dist = np.full((h, w), INF, np.float64)
        parent = np.full((h, w), -1, np.int64)
        sx, sy = start
        if not (0 <= sx < w and 0 <= sy < h):
            return dist, parent
        mult = np.ones((h, w), np.float64)
        mult[soft] = 1.6
        mult[hard] = 6.0
        trav = free.copy()
        trav[sy, sx] = True
        dist[sy, sx] = 0.0
        pq = [(0.0, sx, sy)]
        nb = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
              (1, 1, SQ2), (1, -1, SQ2), (-1, 1, SQ2), (-1, -1, SQ2))
        while pq:
            d, x, y = heapq.heappop(pq)
            if d > dist[y, x]:
                continue
            for dx, dy, c in nb:
                nx, ny = x + dx, y + dy
                if nx < 0 or ny < 0 or nx >= w or ny >= h or not trav[ny, nx]:
                    continue
                if dx and dy and not (trav[y, nx] and trav[ny, x]):
                    continue                      # no diagonal corner-cutting through walls
                nd = d + c * res * mult[ny, nx]
                if nd < dist[ny, nx]:
                    dist[ny, nx] = nd
                    parent[ny, nx] = y * w + x
                    heapq.heappush(pq, (nd, nx, ny))
        return dist, parent

    @staticmethod
    def clusters(mask: np.ndarray):
        h, w = mask.shape
        seen = np.zeros_like(mask)
        ys, xs = np.nonzero(mask)
        out = []
        for y0, x0 in zip(ys.tolist(), xs.tolist()):
            if seen[y0, x0]:
                continue
            stack = [(x0, y0)]
            seen[y0, x0] = True
            cells = []
            while stack:
                x, y = stack.pop()
                cells.append((x, y))
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        nx, ny = x + dx, y + dy
                        if 0 <= nx < w and 0 <= ny < h and mask[ny, nx] and not seen[ny, nx]:
                            seen[ny, nx] = True
                            stack.append((nx, ny))
            out.append(cells)
        return out

    def path_to(self, parent, goal, w):
        x, y = goal
        out = [(x, y)]
        guard = 0
        while guard < 200000:
            p = parent[y, x]
            if p < 0:
                break
            y, x = divmod(int(p), w)
            out.append((x, y))
            guard += 1
        out.reverse()
        return out

    def plan(self, grid, res, origin, pose, visited=None, blacklist=(), current_goal=None):
        """grid: int8 HxW (row = y). origin (ox, oy). pose (x, y, yaw). visited: bool HxW (same grid) or None.
        blacklist: iterable of (x, y). Returns a dict (state = goal | done | unreachable | off_map)."""
        h, w = grid.shape
        ox, oy = origin
        px, py, pyaw = pose
        sx, sy = self.world_to_cell(px, py, res, ox, oy)
        if not (0 <= sx < w and 0 <= sy < h):
            return dict(state="off_map", n_frontier_cells=0, n_clusters=0)
        unknown, occ, free, hard, soft = self.layers(grid, res)
        # robust free: >= 4 free neighbours, so single lidar rays poking into unknown don't count
        robust = free & (neighbours_count(free) >= 4)
        frontier = robust & (neighbours_count(unknown) > 0)
        # a frontier hugging a wall is usually a lidar shadow / dark furniture, not a doorway
        frontier &= ~dilate(occ, 1)
        dist, parent = self.dijkstra(free, hard, soft, (sx, sy), res)
        cls = [c for c in self.clusters(frontier) if len(c) >= self.MIN_CELLS]
        black = list(blacklist)
        cands = []
        for c in cls:
            arr = np.array(c)
            d = dist[arr[:, 1], arr[:, 0]]
            k = int(np.argmin(d))
            dmin = float(d[k])
            if not math.isfinite(dmin):
                continue
            # aim slightly inside the opening: the reachable cell nearest the cluster centroid among the
            # cheaper half, so Rover drives INTO the gap rather than to its corner
            cx, cy = arr[:, 0].mean(), arr[:, 1].mean()
            ok = np.isfinite(d) & (d <= dmin + 1.0)
            idx = np.nonzero(ok)[0]
            j = idx[int(np.argmin((arr[idx, 0] - cx) ** 2 + (arr[idx, 1] - cy) ** 2))]
            gx, gy = int(arr[j, 0]), int(arr[j, 1])
            wx, wy = self.cell_to_world(gx, gy, res, ox, oy)
            if any(math.hypot(wx - bx, wy - by) < self.BLACK_R_M for bx, by in black):
                continue
            size_m = len(c) * res
            vis = 0.0
            if visited is not None:
                vis = float(visited[arr[:, 1], arr[:, 0]].mean())
            ang = math.atan2(wy - py, wx - px) - pyaw
            ang = abs(math.atan2(math.sin(ang), math.cos(ang)))
            score = float(dist[gy, gx]) - self.GAIN_W * min(size_m, 2.0) + self.VISIT_W * vis + self.TURN_W * ang / math.pi
            cands.append(dict(cell=(gx, gy), xy=(wx, wy), score=score, path_m=float(dist[gy, gx]),
                              size_m=size_m, visited=vis, n=len(c)))
        n_fc = int(frontier.sum())
        if not cands:
            unreach = sum(1 for c in cls)
            return dict(state="unreachable" if unreach else "done", n_frontier_cells=n_fc,
                        n_clusters=len(cls), frontier=frontier, hard=hard)
        cands.sort(key=lambda c: c["score"])
        best = cands[0]
        if current_goal is not None:
            for c in cands:
                if math.hypot(c["xy"][0] - current_goal[0], c["xy"][1] - current_goal[1]) < 0.5:
                    if c["score"] <= best["score"] + self.KEEP_MARGIN_M:
                        best = c
                    break
        path_cells = self.path_to(parent, best["cell"], w)
        path = [self.cell_to_world(x, y, res, ox, oy) for x, y in path_cells]
        return dict(state="goal", goal=best["xy"], goal_score=best["score"], goal_path_m=best["path_m"],
                    goal_size_m=best["size_m"], goal_visited=best["visited"], path=path,
                    n_frontier_cells=n_fc, n_clusters=len(cls), n_candidates=len(cands),
                    candidates=[(round(c["xy"][0], 2), round(c["xy"][1], 2), round(c["score"], 2)) for c in cands[:8]],
                    frontier=frontier, hard=hard)


def lookahead(path, pose, look_m, hard=None, res=None, origin=None):
    """Point on the path ~look_m ahead of the closest path point to Rover, with line of sight (no hard
    cell crossed) when the grid is given. Returns (x, y, bearing_rad (+left), dist_m) or None."""
    if not path:
        return None
    px, py, pyaw = pose
    d2 = [(x - px) ** 2 + (y - py) ** 2 for x, y in path]
    i0 = int(np.argmin(d2))
    pick = path[min(i0 + 1, len(path) - 1)]
    for j in range(i0, len(path)):
        x, y = path[j]
        if math.hypot(x - px, y - py) > look_m:
            break
        if hard is not None and not _los(hard, res, origin, (px, py), (x, y)):
            break
        pick = (x, y)
    dx, dy = pick[0] - px, pick[1] - py
    b = math.atan2(dy, dx) - pyaw
    b = math.atan2(math.sin(b), math.cos(b))
    return pick[0], pick[1], b, math.hypot(dx, dy)


def _los(hard, res, origin, a, b):
    ox, oy = origin
    h, w = hard.shape
    n = max(2, int(math.hypot(b[0] - a[0], b[1] - a[1]) / (res * 0.5)))
    for k in range(1, n + 1):
        t = k / n
        x = a[0] + (b[0] - a[0]) * t
        y = a[1] + (b[1] - a[1]) * t
        ix, iy = int(math.floor((x - ox) / res)), int(math.floor((y - oy) / res))
        if 0 <= ix < w and 0 <= iy < h and hard[iy, ix] and k > 2:
            return False
    return True
