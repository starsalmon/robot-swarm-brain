"""GPU open-vocabulary detector for the robots (Windows + CUDA, Ultralytics YOLO-World).

API (JSON, port DET_PORT, default 8765):
  GET  /health                         -> {ok, model, device, vocab, fps{cam}, last{cam}}
  GET  /vocab                          -> {vocab:[...]}
  POST /vocab   {"vocab":[...]} or {"add":[...]} or {"reset":true}
  POST /detect?cam=rover&track=1       body = JPEG bytes  -> detections for that frame (brain sends its own frames)
  GET  /latest?cam=rover|beeper        -> newest detections from the background MJPEG pullers
Detection: {"cls": str, "conf": float, "bbox": [x1,y1,x2,y2] px, "id": track id or -1}
Response:  {"cam", "ts" (unix s, PC clock), "frame_ts", "w", "h", "infer_ms", "dets":[...]}
"""
import json, os, threading, time, urllib.request
import cv2, numpy as np, torch
from fastapi import FastAPI, Request, HTTPException
from ultralytics import YOLOWorld
import uvicorn

E = lambda k, d: type(d)(os.environ.get(k, d))
MODEL = E("DET_MODEL", "yolov8x-worldv2.pt")       # yolov8l-worldv2.pt is ~2x faster
PORT, CONF, IMGSZ = E("DET_PORT", 8765), E("DET_CONF", 0.25), E("DET_IMGSZ", 640)
CAMS = json.loads(E("DET_CAMS", '{"rover": "http://192.168.3.128:81/stream", "beeper": "http://192.168.3.241:81/stream"}'))
PULL = E("DET_PULL", 1)                              # 1 = keep pulling both MJPEG streams for /latest
DEFAULT_VOCAB = ["person", "orange fruit", "cup", "mug", "bottle", "bowl", "plate", "spoon", "fork", "knife", "banana", "apple",
                 "phone", "remote control", "keyboard", "computer mouse", "laptop", "book", "scissors", "teddy bear", "ball",
                 "toy", "orange ball", "orange toy", "basketball", "traffic cone", "orange lid", "shoe", "backpack", "handbag", "umbrella", "clock", "vase", "pot plant", "chair", "couch", "bed",
                 "dining table", "tv", "door", "cat", "dog", "lego", "box", "charging dock"]

VOCAB_FILE = E("DET_VOCAB_FILE", "vocab_extra.json")   # find-X additions, kept across restarts (cwd = /dweights on the NUC)
def load_extra():
    try: return [w for w in json.load(open(VOCAB_FILE)) if w not in DEFAULT_VOCAB]
    except Exception: return []
dev = "cuda:0" if torch.cuda.is_available() else "cpu"
lock = threading.Lock()
models, vocab = {}, list(DEFAULT_VOCAB) + load_extra()
latest, fps = {}, {}


def model_for(cam):
    """One model per camera so ByteTrack state is per stream; all share the same vocabulary."""
    if cam not in models:
        m = YOLOWorld(MODEL); m.set_classes(vocab); m.to(dev); models[cam] = m
    return models[cam]


def set_vocab(v):
    global vocab
    v = [str(x).strip() for x in v if str(x).strip()]
    if not v: raise HTTPException(400, "empty vocab")
    if v == vocab: return                        # unchanged: skip the CLIP re-encode (~0.5 s per model)
    with lock:
        vocab = v
        for m in models.values(): m.set_classes(vocab)
    try: json.dump([w for w in vocab if w not in DEFAULT_VOCAB], open(VOCAB_FILE, "w"))
    except Exception as e: print("vocab save failed:", e)


def run(cam, img, track, frame_ts, imgsz=None):
    t0 = time.perf_counter()
    with lock:
        m = model_for(cam)
        kw = dict(conf=CONF, imgsz=imgsz or IMGSZ, verbose=False, device=dev)
        r = (m.track(img, persist=True, tracker="bytetrack.yaml", **kw) if track else m.predict(img, **kw))[0]
    dets = []
    if r.boxes is not None and len(r.boxes):
        ids = r.boxes.id.int().tolist() if r.boxes.id is not None else [-1] * len(r.boxes)
        for c, cf, xy, i in zip(r.boxes.cls.int().tolist(), r.boxes.conf.tolist(), r.boxes.xyxy.tolist(), ids):
            dets.append({"cls": r.names[c], "conf": round(cf, 3), "bbox": [round(v, 1) for v in xy], "id": i})
    return {"cam": cam, "ts": time.time(), "frame_ts": frame_ts, "w": img.shape[1], "h": img.shape[0],
            "infer_ms": round((time.perf_counter() - t0) * 1e3, 1), "dets": dets}


def puller(cam, url):
    """Background MJPEG reader: decodes the newest frame, detects, keeps it for GET /latest."""
    n, t1 = 0, time.time()
    while True:
        try:
            buf = b""
            with urllib.request.urlopen(url, timeout=5) as s:
                while True:
                    buf += s.read(16384)
                    a = buf.find(b"\xff\xd8"); b = buf.find(b"\xff\xd9", a + 2)
                    if a < 0 or b < 0:
                        buf = buf[-200000:]; continue
                    jpg, buf = buf[a:b + 2], buf[b + 2:]
                    if buf.find(b"\xff\xd9") >= 0: continue          # behind: skip to the newest frame
                    img = cv2.imdecode(np.frombuffer(jpg, np.uint8), 1)
                    if img is None: continue
                    latest[cam] = run(cam + "_pull", img, True, time.time())
                    n += 1
                    if time.time() - t1 >= 2: fps[cam] = round(n / (time.time() - t1), 1); n, t1 = 0, time.time()
        except Exception as e:
            fps[cam] = 0.0; latest.setdefault(cam, {})["error"] = str(e); time.sleep(2)


app = FastAPI()

@app.get("/health")
def health():
    return {"ok": True, "model": MODEL, "device": dev, "gpu": torch.cuda.get_device_name(0) if dev != "cpu" else None,
            "vocab": vocab, "fps": fps, "last": {k: v.get("ts") for k, v in latest.items()}}

@app.get("/vocab")
def get_vocab(): return {"vocab": vocab}

@app.post("/vocab")
async def post_vocab(req: Request):
    j = await req.json()
    if j.get("reset"): set_vocab(DEFAULT_VOCAB)
    elif "add" in j: set_vocab(vocab + [x for x in j["add"] if x not in vocab])
    else: set_vocab(j.get("vocab", []))
    return {"vocab": vocab}

@app.post("/detect")
async def detect(req: Request, cam: str = "rover", track: int = 0, imgsz: int = 0):
    img = cv2.imdecode(np.frombuffer(await req.body(), np.uint8), 1)
    if img is None: raise HTTPException(400, "body is not a JPEG")
    return run(cam, img, bool(track), None, imgsz or None)

@app.get("/latest")
def get_latest(cam: str = "rover"):
    if cam not in latest: raise HTTPException(404, f"no frames from {cam} yet")
    return latest[cam]


if __name__ == "__main__":
    model_for("rover").predict(np.zeros((480, 640, 3), np.uint8), verbose=False, device=dev)   # load + warm up
    if PULL:
        for c, u in CAMS.items(): threading.Thread(target=puller, args=(c, u), daemon=True).start()
    print(f"detector up on :{PORT}  model={MODEL} device={dev}")
    uvicorn.run(app, host="0.0.0.0", port=PORT, log_level="warning")
