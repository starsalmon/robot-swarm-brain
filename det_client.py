"""Client for the YOLO-World detector service (detector/detector_server.py). Flag-gated, default OFF.

DET_REMOTE=1 turns it on. DET_URL picks the server: the NUC's own RTX 2060 container (default,
http://127.0.0.1:8765) or the Windows RTX 3080 PC (http://192.168.2.44:8765). While the server is unhealthy
(no answer, error, or slow) detect() returns None and callers use their local YOLO11, so nothing changes when it's off.
Rechecks /health every DET_RETRY_S after a failure.
"""
import json, os, threading, time, urllib.request

E = lambda k, d: type(d)(os.environ.get(k, d))
ENABLED = E("DET_REMOTE", 0) == 1
URL = E("DET_URL", "http://127.0.0.1:8765").rstrip("/")
PC_URL = "http://192.168.2.44:8765"                    # Windows RTX 3080, use as DET_URL to offload there
TIMEOUT = E("DET_TIMEOUT_S", 0.35)
RETRY_S = E("DET_RETRY_S", 10.0)
ORANGE_NAMES = {"orange fruit", "orange"}               # map YOLO-World names onto the COCO roles callers use
PERSON_NAMES = {"person"}


class Detector:
    def __init__(s, cam="rover"):
        s.cam = cam; s.ok = False; s.t_bad = -1e9; s.n_ok = s.n_fail = 0; s.why = "off" if not ENABLED else "not checked"
        s.lock = threading.Lock()

    def healthy(s):
        if not ENABLED: return False
        if s.ok: return True
        if time.monotonic() - s.t_bad < RETRY_S: return False
        try:
            with urllib.request.urlopen(URL + "/health", timeout=1.0) as r: s.ok = bool(json.load(r).get("ok"))
            s.why = "healthy" if s.ok else "health not ok"
        except Exception as e:
            s.ok = False; s.why = f"health: {e}"
        if not s.ok: s.t_bad = time.monotonic()
        return s.ok

    def detect(s, jpeg, track=False):
        """jpeg bytes -> list of {cls, conf, bbox[x1,y1,x2,y2], id} or None (unhealthy -> use local YOLO)."""
        if not s.healthy(): return None
        try:
            q = urllib.request.Request(f"{URL}/detect?cam={s.cam}&track={int(track)}", data=bytes(jpeg),
                                       headers={"Content-Type": "image/jpeg"})
            with urllib.request.urlopen(q, timeout=TIMEOUT) as r: out = json.load(r)
            s.n_ok += 1; return out["dets"]
        except Exception as e:
            s.n_fail += 1; s.ok = False; s.t_bad = time.monotonic(); s.why = f"detect: {e}"; return None

    @staticmethod
    def set_vocab(words=None, add=None, reset=False):
        """'find X' hook: replace (words), extend (add) or reset the server vocabulary. Returns the new vocab or None."""
        if not ENABLED: return None
        body = {"reset": True} if reset else ({"add": list(add)} if add else {"vocab": list(words or [])})
        try:
            q = urllib.request.Request(URL + "/vocab", data=json.dumps(body).encode(), headers={"Content-Type": "application/json"})
            with urllib.request.urlopen(q, timeout=2.0) as r: return json.load(r)["vocab"]
        except Exception:
            return None
