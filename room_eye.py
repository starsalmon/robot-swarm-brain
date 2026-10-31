#!/usr/bin/env python3
"""Overhead LifeCam on the brain. Finds the rover and the orange (fruit, YOLO COCO class 49, 4-of-6 frame vote) on the carpet.

Publishes /room/overhead as 8 floats, pixels:
  rover_x, rover_y, rover_w, rover_h, orange_x, orange_y, orange_radius, rover_confirmed
A 0 means that object was not seen this frame. Rover fields are only filled once the track is CONFIRMED by motion:
a moving foreground blob (OpenCV MOG2 background subtraction) that moves while /cmd_vel is commanding motion.
Static tubs/boxes melt into the background and are never confirmed; a confirmed track that stays still while
/cmd_vel says drive is dropped.
Also publishes the live picture on /room/lifecam/image_raw/compressed (~4 Hz, 960x540 JPEG)
for the web view (room-cam-web, port 8090).
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

from orange_vote import OrangeVote
from rover_track import RoverTrack

MODEL = os.environ.get("ROOM_ORANGE_MODEL", "/weights/yolo11s.pt")
CONF = float(os.environ.get("ORANGE_CONF", "0.40"))


class OrangeFinder:
    """The orange FRUIT, not orange-coloured things: YOLO11 COCO class 49 ('orange'), then a 4-of-6 frame vote
    (orange_vote.OrangeVote). Replaces the old HSV blob finder that latched onto tubs and boxes."""

    def __init__(self, model_path, conf):
        from ultralytics import YOLO
        import torch
        self.dev = 0 if torch.cuda.is_available() else "cpu"
        self.model = YOLO(model_path)
        self.conf = conf
        self.vote = OrangeVote(need=4, window=6, max_jump=0.08)
        self.raw = []
        import det_client as DC
        self.DC = DC; self.det = DC.Detector("room"); self.src = "local"

    def __call__(self, frame):
        h, w = frame.shape[:2]
        remote = None
        if self.DC.ENABLED and self.det.healthy():
            ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
            if ok: remote = self.det.detect(jpg.tobytes(), imgsz=1280)
        self.src = "remote" if remote is not None else "local"
        if remote is not None:            # YOLO-World: only 'orange fruit' counts, decoy classes are ignored
            self.raw = [(*d["bbox"], float(d["conf"])) for d in remote if d["cls"] in self.DC.ORANGE_NAMES and d["conf"] >= self.conf]
        else:
            self.raw = self.local(frame)
        hit = self.vote.update(self.raw, w, h)
        if hit is None:
            return None
        return hit["cx"] * w, hit["cy"] * h, hit["r"] * w

    def local(self, frame):
        r = self.model.predict(frame, classes=[49], conf=self.conf, imgsz=1280, device=self.dev,
                               half=self.dev != "cpu", verbose=False)[0]
        return [(*map(float, b), float(c)) for b, c in zip(r.boxes.xyxy.cpu().numpy(), r.boxes.conf.cpu().numpy())]


def rover_candidates(hsv, orange_xy):
    """Blobs on the carpet. The desk and keyboard sit in the bottom band and are ignored."""
    h, w = hsv.shape[:2]
    roi = np.zeros((h, w), np.uint8)
    roi[int(h * 0.10):int(h * 0.76), int(w * 0.04):int(w * 0.96)] = 255
    carpet = cv2.inRange(hsv, (0, 0, 60), (179, 45, 190))
    odd = cv2.bitwise_and(cv2.bitwise_not(carpet), roi)
    odd = cv2.morphologyEx(odd, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    odd = cv2.morphologyEx(odd, cv2.MORPH_CLOSE, np.ones((11, 11), np.uint8))
    cnts, _ = cv2.findContours(odd, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    found = []
    for c in cnts:
        area = cv2.contourArea(c)
        if area < 1800 or area > 16000:
            continue
        x, y, bw, bh = cv2.boundingRect(c)
        aspect = bw / max(bh, 1)
        if aspect < 0.45 or aspect > 2.2:
            continue
        M = cv2.moments(c)
        if M["m00"] < 1:
            continue
        cx, cy = M["m10"] / M["m00"], M["m01"] / M["m00"]
        if orange_xy is not None and (cx - orange_xy[0]) ** 2 + (cy - orange_xy[1]) ** 2 < 45 ** 2:
            continue
        found.append((area, cx, cy, float(bw), float(bh)))
    return found


class Eye(Node):
    def __init__(self):
        super().__init__("room_eye")
        self.pub = self.create_publisher(Float32MultiArray, "/room/overhead", 10)
        self.cap = None
        self.last_dbg = 0.0
        self.track = RoverTrack()
        self.t_cmd = -1e9; self.cmd_moving = False
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.img_pub = self.create_publisher(CompressedImage, "/room/lifecam/image_raw/compressed", 2)
        self.n = 0
        self.orange = OrangeFinder(MODEL, CONF)
        self.create_timer(0.12, self.tick)
        self.create_timer(10.0, lambda: self.get_logger().info(f"orange det={self.orange.src}({self.orange.det.why}) raw={[round(r[4], 2) for r in self.orange.raw]}"))
        self.get_logger().info(f"room eye up, LifeCam /dev/video0, orange = YOLO class 49 on {self.orange.dev}")

    def on_cmd(self, m):
        self.t_cmd = time.monotonic()
        self.cmd_moving = abs(m.linear.x) > 0.03 or abs(m.angular.z) > 0.05

    def camera(self):
        if self.cap is not None and self.cap.isOpened():
            return self.cap
        cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        if not cap.isOpened():
            self.get_logger().warn("LifeCam not open")
            return None
        self.cap = cap
        return cap

    def tick(self):
        cap = self.camera()
        msg = Float32MultiArray()
        blank = [0.0] * 8
        if cap is None:
            msg.data = blank
            self.pub.publish(msg)
            return
        frame = None
        for _ in range(2):
            ok, f = cap.read()
            if ok and f is not None:
                frame = f
        if frame is None:
            msg.data = blank
            self.pub.publish(msg)
            return
        self.n += 1
        if self.n % 2 == 0 and self.img_pub.get_subscription_count() > 0:
            ok, jpg = cv2.imencode(".jpg", cv2.resize(frame, (960, 540)), [cv2.IMWRITE_JPEG_QUALITY, 70])
            if ok:
                im = CompressedImage()
                im.header.stamp = self.get_clock().now().to_msg()
                im.header.frame_id = "lifecam"
                im.format = "jpeg"
                im.data = jpg.tobytes()
                self.img_pub.publish(im)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        orange = self.orange(frame)
        cands = rover_candidates(hsv, orange[:2] if orange else None)
        now = time.monotonic()
        rover = self.track.update(frame, cands, now - self.t_cmd < 0.5 and self.cmd_moving, now)
        data = [0.0] * 8
        if rover:
            _, cx, cy, bw, bh = rover
            data[0], data[1], data[2], data[3], data[7] = cx, cy, bw, bh, 1.0
        if orange:
            data[4], data[5], data[6] = orange
        msg.data = [float(x) for x in data]
        self.pub.publish(msg)
        if now - self.last_dbg > 2.0:
            self.last_dbg = now
            cv2.imwrite("/tmp/room_eye_raw.jpg", frame)
            vis = frame.copy()
            if rover:
                _, cx, cy, bw, bh = rover
                cv2.rectangle(vis, (int(cx - bw / 2), int(cy - bh / 2)), (int(cx + bw / 2), int(cy + bh / 2)), (255, 80, 0), 2)
            if orange:
                cv2.circle(vis, (int(orange[0]), int(orange[1])), int(max(8, orange[2])), (0, 80, 255), 2)
            cv2.imwrite("/tmp/room_eye.jpg", vis)


def main():
    rclpy.init()
    node = Eye()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.cap is not None:
            node.cap.release()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
