#!/usr/bin/env python3
"""Rover camera bridge: pull MJPEG from the AI Cam (rover-cam) and publish to ROS 2.

Publishes
  /rover/camera/image_raw/compressed  sensor_msgs/CompressedImage (JPEG, as sent by the camera)
  /rover/camera/camera_info           sensor_msgs/CameraInfo (from CAMERA_INFO_YAML; placeholder until calibrated)
  /rover/camera/image_raw             sensor_msgs/Image (bgr8) only if CAMERA_PUBLISH_RAW=1 and OpenCV is installed

Env
  CAMERA_URL          default http://rover-cam.local:81/stream
  CAMERA_FRAME_ID     default camera_link
  CAMERA_INFO_YAML    default /opt/fleet/config/rover_cam_info.yaml
  CAMERA_PUBLISH_RAW  default 0

Reconnects forever with backoff, so the container can run before the camera exists.
Compressed is the transport of choice; RTAB-Map / image_proc can consume it via
image_transport ("compressed") or a republish node.
"""
import os
import time
import urllib.request

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

try:
    import yaml
except ImportError:  # ros-base ships python3-yaml, but stay safe
    yaml = None

URL = os.environ.get("CAMERA_URL", "http://rover-cam.local:81/stream")
FRAME_ID = os.environ.get("CAMERA_FRAME_ID", "camera_link")
INFO_YAML = os.environ.get("CAMERA_INFO_YAML", "/opt/fleet/config/rover_cam_info.yaml")
PUBLISH_RAW = os.environ.get("CAMERA_PUBLISH_RAW", "0") == "1"


def load_camera_info(path):
    info = CameraInfo()
    info.distortion_model = "plumb_bob"
    if yaml is None or not os.path.exists(path):
        return info
    with open(path) as f:
        d = yaml.safe_load(f) or {}
    info.width = int(d.get("image_width", 0))
    info.height = int(d.get("image_height", 0))
    info.distortion_model = d.get("distortion_model", "plumb_bob")
    info.k = [float(x) for x in d.get("camera_matrix", {}).get("data", [0.0] * 9)]
    info.d = [float(x) for x in d.get("distortion_coefficients", {}).get("data", [0.0] * 5)]
    info.r = [float(x) for x in d.get("rectification_matrix", {}).get("data",
                                                                    [1, 0, 0, 0, 1, 0, 0, 0, 1])]
    info.p = [float(x) for x in d.get("projection_matrix", {}).get("data", [0.0] * 12)]
    return info


class CameraBridge(Node):
    def __init__(self):
        super().__init__("rover_camera_bridge")
        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.pub_jpeg = self.create_publisher(CompressedImage, "/rover/camera/image_raw/compressed", qos)
        self.pub_info = self.create_publisher(CameraInfo, "/rover/camera/camera_info", qos)
        self.pub_raw = None
        self.cv2 = self.np = None
        if PUBLISH_RAW:
            try:
                import cv2
                import numpy as np
                self.cv2, self.np = cv2, np
                self.pub_raw = self.create_publisher(Image, "/rover/camera/image_raw", qos)
            except ImportError:
                self.get_logger().warn("CAMERA_PUBLISH_RAW=1 but OpenCV missing; compressed only")
        self.info = load_camera_info(INFO_YAML)
        self.frames = 0
        self.t0 = time.monotonic()

    def publish_jpeg(self, jpg):
        stamp = self.get_clock().now().to_msg()
        m = CompressedImage()
        m.header.stamp = stamp
        m.header.frame_id = FRAME_ID
        m.format = "jpeg"
        m.data = jpg
        self.pub_jpeg.publish(m)
        self.info.header.stamp = stamp
        self.info.header.frame_id = FRAME_ID
        if self.pub_raw is not None:
            img = self.cv2.imdecode(self.np.frombuffer(jpg, self.np.uint8), self.cv2.IMREAD_COLOR)
            if img is not None:
                r = Image()
                r.header = m.header
                r.height, r.width = img.shape[:2]
                r.encoding = "bgr8"
                r.step = r.width * 3
                r.data = img.tobytes()
                self.pub_raw.publish(r)
                if not self.info.width:
                    self.info.width, self.info.height = r.width, r.height
        self.pub_info.publish(self.info)
        self.frames += 1
        now = time.monotonic()
        if now - self.t0 >= 10.0:
            self.get_logger().info(f"{self.frames / (now - self.t0):.1f} fps, {len(jpg)} B/frame")
            self.frames, self.t0 = 0, now

    def run(self):
        backoff = 0.5
        while rclpy.ok():
            try:
                self.get_logger().info(f"connecting {URL}")
                with urllib.request.urlopen(URL, timeout=1.5) as resp:   # 1.5 s stall detect (was 5)
                    backoff = 0.5
                    buf = b""
                    while rclpy.ok():
                        chunk = resp.read(8192)
                        if not chunk:
                            raise ConnectionError("stream closed")
                        buf += chunk
                        # Split on JPEG markers (robust to boundary/header formatting).
                        while True:
                            a = buf.find(b"\xff\xd8")
                            if a < 0:
                                buf = buf[-1:]
                                break
                            b = buf.find(b"\xff\xd9", a + 2)
                            if b < 0:
                                buf = buf[a:]
                                break
                            self.publish_jpeg(bytes(buf[a:b + 2]))
                            buf = buf[b + 2:]
                        if len(buf) > 2_000_000:
                            buf = b""
            except Exception as e:  # noqa: BLE001 - keep retrying until the camera exists
                self.get_logger().warn(f"camera stream: {e}; retry in {backoff:.1f}s")
                time.sleep(backoff)
                backoff = min(backoff * 2.0, 3.0)   # cam reboots in ~3 s; was 30 s cap


def main():
    rclpy.init()
    node = CameraBridge()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
