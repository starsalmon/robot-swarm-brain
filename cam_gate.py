"""Freshness gate for Rover's camera and the person it sees.

Live camera  = >= MIN_FPS frames in the last second AND the newest frame younger than MAX_GAP_S.
Person       = present in >= NEED frames of the last WINDOW_S seconds, spanning >= ENTER_S of frame times, camera live.
Anything else (camera silent, a short burst of buffered frames, a target left over from the last frame) = no person.
"""
from collections import deque


class CamGate:
    def __init__(s, min_fps=3.0, max_gap_s=0.5, need=4, window_s=1.0, enter_s=0.5):
        s.min_fps, s.max_gap_s, s.need, s.window_s, s.enter_s = min_fps, max_gap_s, need, window_s, enter_s
        s.frames = deque(); s.hits = deque(); s.since = None

    def on_frame(s, now, person):
        s.frames.append(now)
        if person: s.hits.append(now)
        s._trim(now)

    def _trim(s, now):
        for q in (s.frames, s.hits):
            while q and now - q[0] > max(1.0, s.window_s): q.popleft()

    def live(s, now):
        s._trim(now)
        return bool(s.frames) and now - s.frames[-1] <= s.max_gap_s and len(s.frames) >= s.min_fps

    def person(s, now):
        """(candidate, confirmed): candidate = seen in a live frame just now; confirmed = held long enough."""
        if not s.live(now):
            s.since = None; return False, False
        recent = [t for t in s.hits if now - t <= s.window_s]
        cand = bool(recent) and now - recent[-1] <= s.max_gap_s
        if not cand:
            s.since = None; return False, False
        s.since = s.since or recent[0]                    # held time is measured between FRAME times, not wall time,
        return True, len(recent) >= s.need and recent[-1] - s.since >= s.enter_s   # so a buffered burst can't qualify
