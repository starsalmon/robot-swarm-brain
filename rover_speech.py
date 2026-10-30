#!/usr/bin/env python3
"""Rover speech: names the object you hold up to Rover's AI cam, and relays the AI cam whistle detector.

  * YOLO11 (COCO) on /rover/camera/image_raw/compressed, ~SPEECH_FPS frames a second.
  * "Held up" = the biggest non-person box, conf >= SPEECH_CONF, covering >= SPEECH_MIN_AREA of the frame,
    and the top object in >= SPEECH_VOTE_NEED of the last SPEECH_VOTE_WIN frames.
  * Says "That's a cup." with Piper (local TTS) -> ffmpeg MP3 -> served from this node's HTTP server ->
    AI cam GET /play?url=... (ESP32-audioI2S streams it). No firmware change needed for speech.
  * Cooldown: the same object again only after SPEECH_SAME_S, anything after SPEECH_ANY_S.
  * /rover/speech/say (String) speaks any text; /rover/speech/said (String) reports what was spoken.
  * Whistle: polls AI cam GET /whistle (firmware Goertzel detector); a new whistle publishes /rover/whistle
    (Bool true) and Rover says "Coming!". Silent no-op until the AI cam firmware with /whistle is flashed.
"""
import http.server, json, os, socketserver, subprocess, threading, time, urllib.parse, urllib.request
from collections import Counter, deque

E = lambda k, d: type(d)(os.environ.get(k, d))
CAM = E("AICAM_URL", "http://192.168.3.128")
HOST_IP = E("SPEECH_HOST_IP", "192.168.3.172")
PORT = E("SPEECH_PORT", 8099)
VOICE = E("SPEECH_VOICE", "/voices/en_GB-alan-medium.onnx")
MODEL = E("SPEECH_MODEL", "/weights/yolo11s.pt")
FPS = E("SPEECH_FPS", 5.0)
CONF = E("SPEECH_CONF", 0.45)
MIN_AREA = E("SPEECH_MIN_AREA", 0.05)
NEED = E("SPEECH_VOTE_NEED", 4)
WIN = E("SPEECH_VOTE_WIN", 6)
SAME_S = E("SPEECH_SAME_S", 30.0)
ANY_S = E("SPEECH_ANY_S", 6.0)
OUT = E("SPEECH_DIR", "/tmp/rover_speech")
SKIP = {"person", "dining table", "bed", "couch", "chair", "tv", "refrigerator", "toilet", "bench", "oven", "sink"}
NICE = {"cell phone": "phone", "sports ball": "ball", "hair drier": "hair dryer", "potted plant": "pot plant",
        "wine glass": "wine glass", "tennis racket": "tennis racquet", "mouse": "computer mouse"}


def phrase(name):
    n = NICE.get(name, name)
    return f"That's {'an' if n[0] in 'aeiou' else 'a'} {n}."


class Quiet(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *a, **k): super().__init__(*a, directory=OUT, **k)
    def log_message(self, *a): pass


def cam_get(path, timeout=2.0):
    with urllib.request.urlopen(CAM + path, timeout=timeout) as r:
        return r.read().decode(errors="replace")


def main():
    import cv2, numpy as np, rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import Bool, String
    from ultralytics import YOLO

    os.makedirs(OUT, exist_ok=True)
    for f in os.listdir(OUT):
        if f.endswith((".mp3", ".wav")): os.remove(os.path.join(OUT, f))
    socketserver.TCPServer.allow_reuse_address = True
    srv = socketserver.ThreadingTCPServer(("0.0.0.0", PORT), Quiet)
    threading.Thread(target=srv.serve_forever, daemon=True).start()

    class S(Node):
        def __init__(s):
            super().__init__("rover_speech")
            s.model = YOLO(MODEL); s.img = None; s.t_det = 0.0
            s.hist = deque(maxlen=WIN); s.last_said = {}; s.t_any = -1e9; s.n = 0
            s.q = deque(); s.qlock = threading.Lock(); s.wcount = None
            s.create_subscription(CompressedImage, "/rover/camera/image_raw/compressed", lambda m: setattr(s, "img", m), qos_profile_sensor_data)
            s.create_subscription(String, "/rover/speech/say", lambda m: s.say(m.data), 10)
            s.psaid = s.create_publisher(String, "/rover/speech/said", 10)
            s.pseen = s.create_publisher(String, "/rover/speech/object", 10)
            s.pwh = s.create_publisher(Bool, "/rover/whistle", 10)
            s.create_timer(1.0 / FPS, s.detect)
            s.create_timer(0.2, s.poll_whistle)
            threading.Thread(target=s.speaker, daemon=True).start()
            s.get_logger().info(f"speech up: http://{HOST_IP}:{PORT}/ -> {CAM}/play")

        def say(s, text):
            text = (text or "").strip()
            if text:
                with s.qlock: s.q.append(text)

        def speaker(s):
            while True:
                with s.qlock: text = s.q.popleft() if s.q else None
                if text is None: time.sleep(0.05); continue
                try:
                    s.n += 1; base = os.path.join(OUT, f"say_{s.n % 20}")
                    subprocess.run(["piper", "-m", VOICE, "-f", base + ".wav"], input=text.encode(), check=True, capture_output=True, timeout=30)
                    subprocess.run(["ffmpeg", "-loglevel", "error", "-y", "-i", base + ".wav", "-ac", "1", "-ar", "22050", "-b:a", "48k",
                                    base + ".mp3"], check=True, timeout=30)
                    url = f"http://{HOST_IP}:{PORT}/{os.path.basename(base)}.mp3?n={s.n}"
                    cam_get("/play?url=" + urllib.parse.quote(url, safe=""))
                    m = String(); m.data = text; s.psaid.publish(m)
                    s.get_logger().info(f"said: {text}")
                    t0 = time.monotonic()                 # wait for playback to end so phrases don't cut each other off
                    time.sleep(0.8)
                    while time.monotonic() - t0 < 15:
                        try:
                            if not json.loads(cam_get("/sounds")).get("playing"): break
                        except Exception: break
                        time.sleep(0.25)
                except Exception as e:
                    s.get_logger().warning(f"say failed: {e}")

        def detect(s):
            m, s.img = s.img, None
            if m is None: return
            img = cv2.imdecode(np.frombuffer(m.data, np.uint8), 1)
            if img is None: return
            H, W = img.shape[:2]
            r = s.model.predict(img, conf=CONF, verbose=False)[0]
            best, barea = None, 0.0
            for b in r.boxes:
                name = s.model.names[int(b.cls)]
                if name in SKIP: continue
                x1, y1, x2, y2 = b.xyxy[0].tolist()
                area = (x2 - x1) * (y2 - y1) / (W * H)
                if area >= MIN_AREA and area > barea: best, barea = name, area
            s.hist.append(best)
            top, cnt = Counter(x for x in s.hist if x).most_common(1)[0] if any(s.hist) else (None, 0)
            if top is None or cnt < NEED: return
            now = time.monotonic()
            if now - s.t_any < ANY_S or now - s.last_said.get(top, -1e9) < SAME_S: return
            s.t_any = now; s.last_said[top] = now
            o = String(); o.data = top; s.pseen.publish(o)
            s.say(phrase(top))

        def poll_whistle(s):
            try:
                j = json.loads(cam_get("/whistle", 0.5))
            except Exception:
                return
            c = int(j.get("count", 0))
            if s.wcount is not None and c > s.wcount:
                b = Bool(); b.data = True; s.pwh.publish(b)
                s.get_logger().info(f"whistle {j.get('freq')} Hz {j.get('dur_ms')} ms")
                s.say("Coming!")
            s.wcount = c

    rclpy.init(); n = S()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally: n.destroy_node(); rclpy.try_shutdown(); srv.shutdown()


if __name__ == "__main__":
    main()
