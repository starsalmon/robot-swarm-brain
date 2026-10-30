"""Overhead Rover track that only trusts a blob once it has been seen MOVING while Rover was told to move.

Candidates come from room_eye.rover_candidates (non-carpet blobs). Motion comes from OpenCV's MOG2 background
subtractor, so static tubs, boxes and toys become background within seconds and never qualify.
  - Unconfirmed: a candidate that is mostly foreground while /cmd_vel commands motion is followed; once it has
    travelled >= CONFIRM_PX within CONFIRM_S it becomes the confirmed Rover.
  - Confirmed: follow the nearest candidate within GATE_PX (it may stop and blend in, that's fine).
    Drop it if nothing is near for LOST_S, or if /cmd_vel has said move for STILL_S and it moved < STILL_PX.
"""
import math

import cv2


class RoverTrack:
    def __init__(s, confirm_px=40.0, confirm_s=3.0, gate_px=150.0, lost_s=1.0, still_s=2.5, still_px=15.0, fg_frac=0.25):
        s.mog = cv2.createBackgroundSubtractorMOG2(history=150, varThreshold=25, detectShadows=False)
        s.confirm_px, s.confirm_s, s.gate_px, s.lost_s = confirm_px, confirm_s, gate_px, lost_s
        s.still_s, s.still_px, s.fg_frac = still_s, still_px, fg_frac
        s.reset()

    def reset(s):
        s.confirmed = False; s.pos = None; s.t_seen = None; s.hist = []   # hist: (t, x, y) while moving-commanded
        s.move_since = None; s.move_from = None

    def _fg(s, mask, c):
        _, cx, cy, bw, bh = c
        x0, y0 = max(0, int(cx - bw / 2)), max(0, int(cy - bh / 2))
        roi = mask[y0:int(cy + bh / 2), x0:int(cx + bw / 2)]
        return float((roi > 0).mean()) if roi.size else 0.0

    def update(s, frame, cands, moving, now):
        """frame: BGR; cands: [(area, cx, cy, w, h)]; moving: /cmd_vel fresh and non-zero. Returns the Rover cand or None."""
        mask = s.mog.apply(cv2.GaussianBlur(frame, (5, 5), 0))
        near = lambda p: min(cands, key=lambda c: math.hypot(c[1] - p[0], c[2] - p[1]), default=None)
        if s.confirmed:
            c = near(s.pos)
            if c is not None and math.hypot(c[1] - s.pos[0], c[2] - s.pos[1]) < s.gate_px:
                s.pos = (c[1], c[2]); s.t_seen = now
            elif now - s.t_seen > s.lost_s:
                s.reset(); return None
            else:
                return None
            if moving:
                if s.move_since is None: s.move_since, s.move_from = now, s.pos
                elif now - s.move_since > s.still_s:
                    if math.hypot(s.pos[0] - s.move_from[0], s.pos[1] - s.move_from[1]) < s.still_px:
                        s.reset(); return None           # told to drive but this blob never moved: not Rover
                    s.move_since, s.move_from = now, s.pos
            else:
                s.move_since = None
            return c
        if not moving:
            s.hist = []; return None
        mov = [c for c in cands if s._fg(mask, c) >= s.fg_frac]
        if not mov:
            return None
        c = near(s.hist[-1][1:]) if s.hist else max(mov, key=lambda c: c[0])
        if c not in mov or (s.hist and math.hypot(c[1] - s.hist[-1][1], c[2] - s.hist[-1][2]) > s.gate_px):
            c = max(mov, key=lambda c: c[0]); s.hist = []
        s.hist = [h for h in s.hist if now - h[0] < s.confirm_s] + [(now, c[1], c[2])]
        if math.hypot(c[1] - s.hist[0][1], c[2] - s.hist[0][2]) >= s.confirm_px:
            s.confirmed = True; s.pos = (c[1], c[2]); s.t_seen = now; s.hist = []
            return c
        return None
