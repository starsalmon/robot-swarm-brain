#!/usr/bin/env python3
"""Rover monocular metric depth: Depth Anything V2 Metric-Indoor-Small on the GPU.

Sub  /rover/camera/image_raw/compressed, /rover/camera/camera_info, /rover/tof/front (L8 centre, Range)
Pub  /rover/camera/depth (Image 32FC1, metres, rectified, camera_optical_frame, source stamp)
     /rover/camera/depth/camera_info (rectified: same K, zero distortion, same stamp)
     /rover/camera/image_rect/compressed (rectified JPEG matching the depth, same stamp)
     /rover/camera/depth_small (+/camera_info): 160x120 16UC1 mm (fixed size, K scaled from input) - use this across containers (UDP)
Env  DEPTH_MODEL (HF id), DEPTH_INPUT_H (multiple of 14, default 252 -> 336x252),
     DEPTH_L8_SCALE=1 to multiply depth by the smoothed median L8/depth ratio (off by default),
     DEPTH_SAMPLE_PATH (writes a colormap PNG once after 50 frames), DEPTH_FRAME_ID.
Frames are dropped while the GPU is busy (always the latest image is processed).
"""
import os, threading, time
from collections import deque
import cv2, numpy as np, rclpy, rclpy.executors, torch
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, Range
from transformers import AutoModelForDepthEstimation

MODEL = os.environ.get("DEPTH_MODEL", "depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf")
IN_H = int(os.environ.get("DEPTH_INPUT_H", "252"))
L8_SCALE = os.environ.get("DEPTH_L8_SCALE", "0") == "1"
SAMPLE = os.environ.get("DEPTH_SAMPLE_PATH", "")
FRAME = os.environ.get("DEPTH_FRAME_ID", "camera_optical_frame")
MEAN = np.array([0.485, 0.456, 0.406], np.float32); STD = np.array([0.229, 0.224, 0.225], np.float32)


class Depth(Node):
    def __init__(self):
        super().__init__("rover_depth")
        be = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.dev = "cuda" if torch.cuda.is_available() else "cpu"
        self.model = AutoModelForDepthEstimation.from_pretrained(MODEL).to(self.dev).eval()
        if self.dev == "cuda":
            self.model = self.model.half()
        self.get_logger().info(f"{MODEL} on {self.dev} ({torch.cuda.get_device_name(0) if self.dev=='cuda' else ''})")
        # Reliable out: a 300 KB 32FC1 frame fragments into ~5 UDP packets and best-effort loses most of them.
        rel = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self.pub_d = self.create_publisher(Image, "/rover/camera/depth", rel)
        self.pub_i = self.create_publisher(CameraInfo, "/rover/camera/depth/camera_info", rel)
        self.pub_r = self.create_publisher(CompressedImage, "/rover/camera/image_rect/compressed", rel)
        # Light topics for other hosts/containers over UDP: 160x120 16UC1 millimetres (38 KB) + matching info.
        self.pub_ds = self.create_publisher(Image, "/rover/camera/depth_small", rel)
        self.pub_dsi = self.create_publisher(CameraInfo, "/rover/camera/depth_small/camera_info", rel)
        self.create_subscription(CompressedImage, "/rover/camera/image_raw/compressed", self.on_img, be)
        self.create_subscription(CameraInfo, "/rover/camera/camera_info", self.on_info, be)
        self.create_subscription(Range, "/rover/tof/front", self.on_tof, 10)
        self.info = None; self.maps = None; self.latest = None
        self.tof = (None, 0.0)
        self.ratios = deque(maxlen=60); self.scale = 1.0
        self.n = self.done = 0; self.t0 = time.monotonic(); self.rx = 0; self.infer_ms = deque(maxlen=50)
        self.ev = threading.Event()
        threading.Thread(target=self.worker, daemon=True).start()
        self.create_timer(10.0, self.log)

    def on_info(self, m):
        if self.info is None or list(m.k) != list(self.info.k):
            self.info = m
            K = np.array(m.k).reshape(3, 3); D = np.array(m.d)
            self.maps = cv2.initUndistortRectifyMap(K, D, None, K, (m.width, m.height), cv2.CV_32FC1)
            ri = CameraInfo(); ri.width, ri.height = m.width, m.height
            ri.distortion_model = "plumb_bob"; ri.d = [0.0] * 5; ri.k = m.k
            ri.r = [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0]; ri.p = [K[0, 0], 0, K[0, 2], 0, 0, K[1, 1], K[1, 2], 0, 0, 0, 1, 0]
            self.rect_info = ri
            self.get_logger().info(f"camera_info fx={K[0,0]:.2f} -> rectification map ready")

    def on_tof(self, m):
        if m.min_range < m.range < m.max_range:
            self.tof = (m.range, time.monotonic())

    def on_img(self, m):
        self.latest = m; self.rx += 1; self.ev.set()

    def infer(self, rgb):
        h, w = rgb.shape[:2]
        iw = int(round(IN_H * w / h / 14)) * 14
        x = (cv2.resize(rgb, (iw, IN_H), interpolation=cv2.INTER_CUBIC).astype(np.float32) / 255 - MEAN) / STD
        t = torch.from_numpy(x.transpose(2, 0, 1))[None].to(self.dev)
        t = t.half() if self.dev == "cuda" else t
        with torch.inference_mode():
            d = self.model(pixel_values=t).predicted_depth
            d = torch.nn.functional.interpolate(d[:, None].float(), (h, w), mode="bilinear", align_corners=False)[0, 0]
        return d.cpu().numpy()

    def worker(self):
        while rclpy.ok():
            self.ev.wait(); self.ev.clear()
            m = self.latest
            if m is None or self.maps is None:
                continue
            bgr = cv2.imdecode(np.frombuffer(bytes(m.data), np.uint8), cv2.IMREAD_COLOR)
            if bgr is None:
                continue
            rect = cv2.remap(bgr, *self.maps, cv2.INTER_LINEAR)
            t = time.monotonic()
            d = self.infer(cv2.cvtColor(rect, cv2.COLOR_BGR2RGB))
            self.infer_ms.append((time.monotonic() - t) * 1000)
            self.check_scale(d)
            if L8_SCALE:
                d = d * self.scale
            out = Image(); out.header.stamp = m.header.stamp; out.header.frame_id = FRAME
            out.height, out.width = d.shape; out.encoding = "32FC1"; out.step = out.width * 4
            out.data = d.astype(np.float32).tobytes()
            self.pub_d.publish(out)
            self.rect_info.header = out.header; self.pub_i.publish(self.rect_info)
            sx = 160.0 / d.shape[1]; ds = cv2.resize(d, (160, int(round(d.shape[0] * sx))), interpolation=cv2.INTER_NEAREST)  # fixed 160 wide (120 at 4:3)
            sm = Image(); sm.header = out.header; sm.height, sm.width = ds.shape; sm.encoding = "16UC1"
            sm.step = sm.width * 2; sm.data = np.clip(ds * 1000, 0, 65535).astype(np.uint16).tobytes()
            self.pub_ds.publish(sm)
            si = CameraInfo(); si.header = out.header; si.width, si.height = sm.width, sm.height
            si.distortion_model = "plumb_bob"; si.d = [0.0] * 5
            k = [x * sx for x in self.rect_info.k]; k[8] = 1.0; si.k = k
            si.r = list(self.rect_info.r); si.p = [k[0], 0.0, k[2], 0.0, 0.0, k[4], k[5], 0.0, 0.0, 0.0, 1.0, 0.0]
            self.pub_dsi.publish(si)
            r = CompressedImage(); r.header = out.header; r.format = "jpeg"
            r.data = cv2.imencode(".jpg", rect, [cv2.IMWRITE_JPEG_QUALITY, 90])[1].tobytes(); self.pub_r.publish(r)
            self.n += 1
            self.done += 1
            if SAMPLE and self.done == 50:
                self.save_sample(rect, d)

    def check_scale(self, d):
        r, ts = self.tof
        if r is None or time.monotonic() - ts > 0.3:
            return
        h, w = d.shape
        c = d[int(h*0.4):int(h*0.6), int(w*0.4):int(w*0.6)]   # central patch ~ L8 centre zones
        med = float(np.median(c))
        if med > 0.05:
            self.ratios.append(r / med)
            self.scale = 0.9 * self.scale + 0.1 * float(np.median(self.ratios))
            self.last_cmp = (r, med)

    def save_sample(self, rect, d):
        v = np.clip(d / max(np.percentile(d, 98), 0.1), 0, 1)
        cm = cv2.applyColorMap((255 * (1 - v)).astype(np.uint8), cv2.COLORMAP_TURBO)
        bar = np.hstack([rect, cm])
        cv2.putText(bar, f"depth 0..{np.percentile(d,98):.1f} m (red=near)", (330, 16), 0, 0.45, (255, 255, 255), 1)
        cv2.imwrite(SAMPLE, bar)
        self.get_logger().info(f"wrote sample {SAMPLE}")

    def log(self):
        dt = time.monotonic() - self.t0
        ms = np.mean(self.infer_ms) if self.infer_ms else 0
        s = ""
        if self.ratios:
            l8, med = getattr(self, "last_cmp", (0, 0))
            s = f", L8/depth ratio median {np.median(self.ratios):.2f} (last L8 {l8:.2f} m vs depth {med:.2f} m, n={len(self.ratios)}), scale {'ON' if L8_SCALE else 'off'} {self.scale:.2f}"
        self.get_logger().info(f"{self.rx/dt:.1f} fps in, {self.n/dt:.1f} fps out, infer {ms:.1f} ms{s}")
        self.n = self.rx = 0; self.t0 = time.monotonic()


def main():
    rclpy.init(); n = Depth()
    ex = rclpy.executors.MultiThreadedExecutor(num_threads=2); ex.add_node(n)
    try: ex.spin()
    except KeyboardInterrupt: pass
    finally: n.destroy_node(); rclpy.try_shutdown()


if __name__ == "__main__":
    main()
