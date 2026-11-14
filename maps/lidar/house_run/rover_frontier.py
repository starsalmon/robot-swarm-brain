#!/usr/bin/env python3
"""Rover map-based explore planner (runs in rover-lidar-slam). NEVER publishes /cmd_vel.

Reads slam_toolbox /map + TF map->base_link, picks the best frontier (frontier_core.py), and publishes a
steering hint that rover_behaviour's EXPLORE follows while its own ToF/lidar guards keep all hard stops:

  /rover/explore/goal_hint  std_msgs/Float32MultiArray
      [bearing_rad (+left, base_link), lookahead_dist_m, path_len_m, state, goal_x, goal_y]
      state: 1 = goal, 2 = done (no frontiers left), 3 = frontiers left but none reachable
      Published at 5 Hz ONLY while a pose + plan exist. Silence = rover_behaviour falls back to the old
      reactive explore after 1.5 s (so a dead planner never stops the safety guards, only the guidance).
  /rover/explore/frontier_status  std_msgs/String (JSON), also written to house_run/frontier.json
  house_run/frontier_latest.png   map + visited trail + frontiers + current path, every ~10 s
  house_run/visited.npz           where Rover has driven (persists across restarts of this node)
"""
from __future__ import annotations

import json
import math
import os
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray, String
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from frontier_core import FrontierPlanner, lookahead  # noqa: E402

RUN = Path(os.environ.get("HOUSE_RUN_DIR", "/maps/lidar/house_run"))
STATUS_FILE = RUN / "frontier.json"
VISITED_FILE = RUN / "visited.npz"
PNG = RUN / "frontier_latest.png"
VISIT_R_M = float(os.environ.get("FRONTIER_VISIT_R_M", "0.35"))
REPLAN_S = float(os.environ.get("FRONTIER_REPLAN_S", "1.5"))
PROGRESS_S = float(os.environ.get("FRONTIER_PROGRESS_S", "30"))      # no 0.25 m of progress in this long -> blacklist goal
ARRIVE_M = float(os.environ.get("FRONTIER_ARRIVE_M", "0.35"))
ARRIVE_HOLD_S = float(os.environ.get("FRONTIER_ARRIVE_HOLD_S", "8"))  # at the goal but frontier did not clear -> blacklist
BLACKLIST_S = float(os.environ.get("FRONTIER_BLACKLIST_S", "300"))
STATE_CODE = {"goal": 1.0, "done": 2.0, "unreachable": 3.0}


class Frontier(Node):
    def __init__(self):
        super().__init__("rover_frontier")
        self.planner = FrontierPlanner()
        self.buf = Buffer()
        self.tl = TransformListener(self.buf, self)
        q = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, q)
        self.phint = self.create_publisher(Float32MultiArray, "/rover/explore/goal_hint", 10)
        self.pstat = self.create_publisher(String, "/rover/explore/frontier_status", 10)
        self.map = None
        self.map_t = None
        self.pose = None
        self.pose_t = None
        self.plan = None
        self.plan_t = 0.0
        self.goal = None
        self.goal_best = None
        self.goal_best_t = None
        self.arrive_t = None
        self.blacklist = []           # (x, y, until)
        self.visited = set()          # world cells (floor(x/res), floor(y/res)) at 0.05 m
        self.vres = 0.05
        self.trail = []
        self.n_goals = 0
        self.png_t = 0.0
        self.save_t = time.time()
        self.plan_ms = None
        self._load_visited()
        self.create_timer(0.2, self.tick)
        self.get_logger().info("rover_frontier up: map frontiers -> /rover/explore/goal_hint (no cmd_vel)")

    # ------------------------------------------------------------------ io
    def _load_visited(self):
        try:
            if VISITED_FILE.exists() and os.environ.get("FRONTIER_FORGET_VISITED", "0") != "1":
                d = np.load(VISITED_FILE)
                self.visited = set(map(tuple, d["cells"].tolist()))
                self.get_logger().info(f"loaded {len(self.visited)} visited cells")
        except Exception as e:
            self.get_logger().warn(f"visited load failed: {e}")

    def _save_visited(self):
        try:
            arr = np.array(sorted(self.visited), np.int32).reshape(-1, 2)
            tmp = VISITED_FILE.with_name("visited.tmp.npz")
            np.savez_compressed(tmp, cells=arr)
            tmp.replace(VISITED_FILE)
        except Exception as e:
            self.get_logger().warn(f"visited save failed: {e}")

    def on_map(self, m):
        self.map = m
        self.map_t = time.time()

    def _pose(self):
        try:
            tf = self.buf.lookup_transform("map", "base_link", rclpy.time.Time(), Duration(seconds=0.0))
        except Exception:
            return None
        t = tf.transform
        q = t.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return (t.translation.x, t.translation.y, yaw)

    def _mark_visited(self, x, y):
        r = int(math.ceil(VISIT_R_M / self.vres))
        cx, cy = int(math.floor(x / self.vres)), int(math.floor(y / self.vres))
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx * dx + dy * dy <= r * r:
                    self.visited.add((cx + dx, cy + dy))

    def _visited_grid(self, h, w, res, ox, oy):
        g = np.zeros((h, w), bool)
        if not self.visited:
            return g
        arr = np.array(list(self.visited), np.int64)
        wx = (arr[:, 0] + 0.5) * self.vres
        wy = (arr[:, 1] + 0.5) * self.vres
        ix = np.floor((wx - ox) / res).astype(np.int64)
        iy = np.floor((wy - oy) / res).astype(np.int64)
        ok = (ix >= 0) & (iy >= 0) & (ix < w) & (iy < h)
        g[iy[ok], ix[ok]] = True
        return g

    # ------------------------------------------------------------------ main loop
    def tick(self):
        now = time.time()
        p = self._pose()
        if p is not None:
            self.pose, self.pose_t = p, now
            if not self.trail or math.hypot(p[0] - self.trail[-1][0], p[1] - self.trail[-1][1]) > 0.05:
                self.trail.append((p[0], p[1]))
                self.trail = self.trail[-4000:]
                self._mark_visited(p[0], p[1])
        self.blacklist = [b for b in self.blacklist if b[2] > now]
        if self.map is not None and self.pose is not None and now - self.plan_t >= REPLAN_S:
            self._replan(now)
        self._publish_hint(now)
        if now - self.save_t > 30:
            self.save_t = now
            self._save_visited()
        if self.plan is not None and now - self.png_t > 10:
            self.png_t = now
            try:
                self._render()
            except Exception as e:
                self.get_logger().warn(f"render failed: {e}")

    def _replan(self, now):
        m = self.map
        h, w, res = m.info.height, m.info.width, m.info.resolution
        ox, oy = m.info.origin.position.x, m.info.origin.position.y
        grid = np.array(m.data, np.int8).reshape(h, w)
        vis = self._visited_grid(h, w, res, ox, oy)
        t0 = time.time()
        out = self.planner.plan(grid, res, (ox, oy), self.pose, visited=vis,
                                blacklist=[(b[0], b[1]) for b in self.blacklist], current_goal=self.goal)
        self.plan_ms = int((time.time() - t0) * 1000)
        out["grid_shape"] = (h, w)
        out["res"] = res
        out["origin"] = (ox, oy)
        self.plan = out
        self.plan_t = now
        if out["state"] == "goal":
            g = out["goal"]
            if self.goal is None or math.hypot(g[0] - self.goal[0], g[1] - self.goal[1]) > 0.5:
                self.goal = g
                self.goal_best = out["goal_path_m"]
                self.goal_best_t = now
                self.arrive_t = None
                self.n_goals += 1
                self.get_logger().info(f"new frontier goal #{self.n_goals} ({g[0]:.2f},{g[1]:.2f}) path {out['goal_path_m']:.2f} m "
                                       f"opening {out['goal_size_m']:.2f} m visited {out['goal_visited']:.2f} "
                                       f"({out['n_candidates']} candidates, {self.plan_ms} ms)")
            else:
                self.goal = g
                if out["goal_path_m"] < self.goal_best - 0.25:
                    self.goal_best = out["goal_path_m"]
                    self.goal_best_t = now
            # progress watchdog: blacklist goals we are not getting closer to
            if now - self.goal_best_t > PROGRESS_S:
                self._blacklist(f"no progress for {PROGRESS_S:.0f}s")
            elif out["goal_path_m"] < ARRIVE_M:
                self.arrive_t = self.arrive_t or now
                if now - self.arrive_t > ARRIVE_HOLD_S:
                    self._blacklist("arrived but frontier did not clear (dark/glass surface?)")
            else:
                self.arrive_t = None
        else:
            if self.goal is not None:
                self.get_logger().info(f"frontier state {out['state']} ({out['n_clusters']} clusters, none reachable/left)")
            self.goal = None

    def _blacklist(self, why):
        if self.goal is None:
            return
        self.blacklist.append((self.goal[0], self.goal[1], time.time() + BLACKLIST_S))
        self.get_logger().info(f"blacklist goal ({self.goal[0]:.2f},{self.goal[1]:.2f}): {why}")
        self.goal = None
        self.goal_best_t = time.time()
        self.plan_t = 0.0               # replan now

    def _publish_hint(self, now):
        st = dict(wall_t=now, state="no_map" if self.map is None else ("no_pose" if self.pose is None else "init"),
                  pose=None if self.pose is None else [round(v, 3) for v in self.pose],
                  pose_age=None if self.pose_t is None else round(now - self.pose_t, 2),
                  map_age=None if self.map_t is None else round(now - self.map_t, 2),
                  visited_m2=round(len(self.visited) * self.vres * self.vres, 2), n_goals=self.n_goals,
                  blacklist=[(round(b[0], 2), round(b[1], 2)) for b in self.blacklist], plan_ms=self.plan_ms)
        pose_ok = self.pose_t is not None and now - self.pose_t < 1.0
        if self.plan is not None:
            pl = self.plan
            st.update(state=pl["state"], n_frontier_cells=pl.get("n_frontier_cells"), n_clusters=pl.get("n_clusters"),
                      goal=None if pl.get("goal") is None else [round(v, 2) for v in pl["goal"]],
                      path_m=None if pl.get("goal_path_m") is None else round(pl["goal_path_m"], 2),
                      candidates=pl.get("candidates"), plan_age=round(now - self.plan_t, 2))
            if not pose_ok:
                st["state"] = "no_pose"
            elif pl["state"] == "goal" and now - self.plan_t < 6.0:
                la = lookahead(pl.get("path"), self.pose, self.planner.LOOKAHEAD_M, pl.get("hard"), pl["res"], pl["origin"])
                if la is not None:
                    msg = Float32MultiArray()
                    msg.data = [float(la[2]), float(la[3]), float(pl["goal_path_m"]), 1.0,
                                float(pl["goal"][0]), float(pl["goal"][1])]
                    self.phint.publish(msg)
                    st.update(bearing_deg=round(math.degrees(la[2]), 1), look_m=round(la[3], 2))
            elif pl["state"] in ("done", "unreachable"):
                msg = Float32MultiArray()
                msg.data = [0.0, 0.0, 0.0, STATE_CODE[pl["state"]], 0.0, 0.0]
                self.phint.publish(msg)
        s = String()
        s.data = json.dumps(st)
        self.pstat.publish(s)
        try:
            tmp = STATUS_FILE.with_suffix(".tmp")
            tmp.write_text(s.data)
            tmp.replace(STATUS_FILE)
        except Exception:
            pass

    def _render(self):
        from PIL import Image, ImageDraw
        m = self.map
        h, w, res = m.info.height, m.info.width, m.info.resolution
        ox, oy = m.info.origin.position.x, m.info.origin.position.y
        g = np.array(m.data, np.int8).reshape(h, w)
        rgb = np.zeros((h, w, 3), np.uint8)
        rgb[g < 0] = (48, 48, 48)
        rgb[(g >= 0) & (g < 50)] = (245, 245, 245)
        rgb[g >= 50] = (0, 0, 0)
        vis = self._visited_grid(h, w, res, ox, oy) & (g >= 0) & (g < 50)
        rgb[vis] = (205, 230, 205)
        pl = self.plan
        if pl is not None and pl.get("frontier") is not None and pl["frontier"].shape == (h, w):
            rgb[pl["frontier"]] = (0, 150, 255)
        sc = 3
        img = Image.fromarray(rgb[::-1]).resize((w * sc, h * sc), Image.NEAREST)
        d = ImageDraw.Draw(img)
        px = lambda x, y: ((x - ox) / res * sc, (h - (y - oy) / res) * sc)
        if len(self.trail) > 1:
            d.line([px(*q) for q in self.trail], fill=(0, 120, 0), width=2)
        if pl is not None and pl.get("path"):
            d.line([px(*q) for q in pl["path"]], fill=(255, 0, 200), width=3)
        for bx, by, _ in self.blacklist:
            x, y = px(bx, by)
            d.line([x - 6, y - 6, x + 6, y + 6], fill=(255, 0, 0), width=2)
            d.line([x - 6, y + 6, x + 6, y - 6], fill=(255, 0, 0), width=2)
        if self.pose is not None:
            x, y = px(self.pose[0], self.pose[1])
            d.ellipse([x - 6, y - 6, x + 6, y + 6], outline=(255, 0, 0), width=3)
            d.line([x, y, x + 14 * math.cos(self.pose[2]), y - 14 * math.sin(self.pose[2])], fill=(255, 0, 0), width=3)
        d.text((4, 4), time.strftime("%H:%M:%S") + f"  state={pl['state'] if pl else '-'}  goals={self.n_goals}", fill=(255, 255, 0))
        tmp = PNG.with_name("frontier_latest.tmp.png")
        img.save(tmp)
        tmp.replace(PNG)


def main():
    rclpy.init()
    n = Frontier()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n._save_visited()
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
