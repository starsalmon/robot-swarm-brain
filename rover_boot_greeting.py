#!/usr/bin/env python3
"""Once per Rover boot, speak a time-of-day + battery greeting on the AI cam.

Uses the same Piper en_GB-alan-medium voice and ffmpeg chain as rover-speech,
then streams the MP3 via the cam's existing GET /play?url= endpoint.
"""
from __future__ import annotations

import http.server
import json
import os
import socketserver
import subprocess
import threading
import time
import urllib.parse
import urllib.request
from datetime import datetime
from zoneinfo import ZoneInfo

E = lambda k, d: type(d)(os.environ.get(k, d)) if not isinstance(d, bool) else os.environ.get(k, str(int(d)))  # noqa: E731

CAM = os.environ.get("AICAM_URL", "http://192.168.3.130").rstrip("/")
BATT_URL = os.environ.get("BATT_URL", "http://192.168.3.71/battery")
HOST_IP = os.environ.get("SPEECH_HOST_IP", "192.168.3.172")
PORT = int(os.environ.get("SPEECH_PORT", "8100"))
VOICE = os.environ.get("SPEECH_VOICE", "/voices/en_GB-alan-medium.onnx")
OUT = os.environ.get("GREET_DIR", "/tmp/rover_boot_greet")
TZ = ZoneInfo(os.environ.get("GREET_TZ", "Australia/Melbourne"))
V_EMPTY = float(os.environ.get("GREET_V_EMPTY", "6.2"))
V_FULL = float(os.environ.get("GREET_V_FULL", "8.3"))
DOWN_S = float(os.environ.get("GREET_DOWN_S", "2.0"))
BATT_WAIT_S = float(os.environ.get("GREET_BATT_WAIT_S", "5.0"))
POLL_S = float(os.environ.get("GREET_POLL_S", "1.0"))

SPEECH_AF = (
    f"acompressor=threshold=0.05:ratio=6:attack=3:release=60,volume={os.environ.get('SPEECH_BOOST_DB', '20')}dB,"
    "alimiter=limit=0.84:attack=2:release=60:level=false"
)

ONES = [
    "zero", "one", "two", "three", "four", "five", "six", "seven", "eight", "nine",
    "ten", "eleven", "twelve", "thirteen", "fourteen", "fifteen", "sixteen",
    "seventeen", "eighteen", "nineteen",
]
TENS = ["", "", "twenty", "thirty", "forty", "fifty", "sixty", "seventy", "eighty", "ninety"]


def num_words(n: int) -> str:
    if n < 20:
        return ONES[n]
    if n == 100:
        return "one hundred"
    t, u = divmod(n, 10)
    return TENS[t] if u == 0 else f"{TENS[t]}-{ONES[u]}"


def period_name(dt: datetime) -> str:
    hm = dt.hour * 60 + dt.minute
    if 5 * 60 <= hm < 12 * 60:
        return "morning"
    if 12 * 60 <= hm < 18 * 60:
        return "afternoon"
    return "evening"


def http_get(url: str, timeout: float = 2.0) -> str:
    with urllib.request.urlopen(url, timeout=timeout) as r:
        return r.read().decode("utf-8", "replace")


def cam_status():
    try:
        d = json.loads(http_get(CAM + "/status", 2.5))
        return int(d.get("uptime_s") or 0), d
    except Exception:
        return None, None


def battery_percent(deadline: float):
    while time.monotonic() < deadline:
        try:
            raw = http_get(BATT_URL, 1.2)
            d = json.loads(raw)
            if d.get("ok") and 0 <= int(d.get("percent", -1)) <= 100:
                return int(d["percent"]), float(d.get("voltage") or 0)
            v = float(d.get("voltage") or 0)
            if v > 0.5:
                p = int(round((v - V_EMPTY) / (V_FULL - V_EMPTY) * 100))
                return max(0, min(100, p)), v
        except Exception:
            pass
        time.sleep(0.4)
    return None, None


def build_sentence(pct):
    now = datetime.now(TZ)
    greet = period_name(now)
    text = f"Hi, good {greet}, I'm Rover."
    if pct is not None:
        text += f" My battery is {num_words(pct)} percent."
    return text, now


def speak(text: str, n: int) -> str:
    os.makedirs(OUT, exist_ok=True)
    base = os.path.join(OUT, "greet")
    wav, mp3 = base + ".wav", base + ".mp3"
    subprocess.run(["piper", "-m", VOICE, "-f", wav], input=text.encode(), check=True, capture_output=True, timeout=30)
    subprocess.run(
        ["ffmpeg", "-loglevel", "error", "-y", "-i", wav, "-af", SPEECH_AF, "-ac", "1", "-ar", "22050", "-b:a", "48k", mp3],
        check=True, timeout=30,
    )
    url = f"http://{HOST_IP}:{PORT}/greet.mp3?n={n}"
    http_get(CAM + "/play?url=" + urllib.parse.quote(url, safe=""), 5.0)
    return url


class Handler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *a, **k):
        super().__init__(*a, directory=OUT, **k)

    def log_message(self, *a):
        pass


def serve():
    os.makedirs(OUT, exist_ok=True)
    class Reuse(socketserver.TCPServer):
        allow_reuse_address = True
    httpd = Reuse(("", PORT), Handler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    print(f"rover-boot-greet HTTP :{PORT}", flush=True)


def greet_once(n: int, uptime: int):
    pct, volts = battery_percent(time.monotonic() + BATT_WAIT_S)
    text, now = build_sentence(pct)
    try:
        url = speak(text, n)
        batt = f"{pct}% {volts:.2f}V" if pct is not None else "skipped"
        print(
            f"greet #{n} uptime={uptime}s {now.strftime('%H:%M %Z')} battery={batt} said: {text} url={url}",
            flush=True,
        )
    except Exception as e:
        print(f"greet failed: {e}", flush=True)


def main():
    serve()
    up, _ = cam_status()
    last_up = up
    was_down_since = None if up is not None else time.monotonic()
    n = 0
    print(f"watching {CAM} initial_uptime={up}", flush=True)
    while True:
        time.sleep(POLL_S)
        up, _ = cam_status()
        if up is None:
            if was_down_since is None:
                was_down_since = time.monotonic()
            last_up = None
            continue
        down_long = was_down_since is not None and (time.monotonic() - was_down_since) >= DOWN_S
        rebooted = last_up is not None and up + 5 < last_up
        new_boot = (down_long or rebooted) and up < 60
        was_down_since = None
        last_up = up
        if new_boot:
            n += 1
            # Let Wi-Fi/HTTP settle; NTP is not required (we use the NUC clock).
            time.sleep(2.0)
            greet_once(n, up)


if __name__ == "__main__":
    main()
