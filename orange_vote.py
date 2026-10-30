#!/usr/bin/env python3
"""Consistency gate for the YOLO 'orange' (COCO class 49) detections, shared by rover_behaviour and room_eye.

A sighting only counts once the fruit has been detected in at least `need` of the last `window` frames, all
within `max_jump` (fraction of image width) of each other. One-frame false positives (a tub, a box, a lamp)
never pass. The best box per frame is the most confident one near the previous accepted position.
"""
import math
from collections import deque


class OrangeVote:
    def __init__(self, need=4, window=6, max_jump=0.15):
        self.need, self.window, self.max_jump = need, window, max_jump
        self.hist = deque(maxlen=window)        # per frame: (cx, cy, r, conf) normalised to image width, or None
        self.last = None

    def update(self, boxes, w, h):
        """boxes: list of (x1, y1, x2, y2, conf) in pixels. Returns the accepted sighting dict or None."""
        pick = None
        for x1, y1, x2, y2, c in boxes:
            bw, bh = x2 - x1, y2 - y1
            if bw < 4 or bh < 4 or not (0.5 < bw / bh < 2.0):  # an orange is round-ish
                continue
            cand = ((x1 + x2) / 2 / w, (y1 + y2) / 2 / w, max(bw, bh) / 2 / w, c)
            if self.last is not None:
                if math.hypot(cand[0] - self.last[0], cand[1] - self.last[1]) > self.max_jump:
                    cand = (cand[0], cand[1], cand[2], c * 0.5)   # far from the tracked one: allowed, but weaker
            if pick is None or cand[3] > pick[3]:
                pick = cand
        self.hist.append(pick)
        hits = [p for p in self.hist if p is not None]
        if pick is None or len(hits) < self.need:
            return None
        near = [p for p in hits if math.hypot(p[0] - pick[0], p[1] - pick[1]) <= self.max_jump]
        if len(near) < self.need:
            return None
        self.last = pick
        cx, cy, r, c = pick
        return dict(cx=cx, cy=cy * w / h, r=r, conf=round(c, 2), err=(cx - 0.5) * 2.0, votes=len(near))

    def reset(self):
        self.hist.clear(); self.last = None
