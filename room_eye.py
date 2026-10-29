#!/usr/bin/env python3
"""Overhead LifeCam on the brain. Finds the rover and the orange on the carpet.

Publishes /room/overhead as 7 floats, pixels:
  rover_x, rover_y, rover_w, rover_h, orange_x, orange_y, orange_radius
A 0 means that object was not seen this frame.
"""
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


def find_orange(hsv):
    mask = cv2.inRange(hsv, (5, 90, 90), (22, 255, 255))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    for c in cnts:
        area = cv2.contourArea(c)
        if area < 200 or area > 4000:
            continue
        peri = cv2.arcLength(c, True)
        circ = 4 * np.pi * area / max(peri * peri, 1)
        if circ < 0.55:
            continue
        score = circ * min(area, 1600)
        if best is None or score > best[0]:
            M = cv2.moments(c)
            if M["m00"] < 1:
                continue
            x, y, bw, bh = cv2.boundingRect(c)
            best = (score, M["m10"] / M["m00"], M["m01"] / M["m00"], 0.5 * max(bw, bh))
    if best is None:
        return None
    return best[1], best[2], best[3]


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
        self.last_rover = None
        self.create_timer(0.12, self.tick)
        self.get_logger().info("room eye up, LifeCam /dev/video0")

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
        blank = [0.0] * 7
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
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        orange = find_orange(hsv)
        cands = rover_candidates(hsv, orange[:2] if orange else None)
        rover = None
        if cands:
            if self.last_rover is not None:
                lx, ly = self.last_rover
                near = [c for c in cands if (c[1] - lx) ** 2 + (c[2] - ly) ** 2 < 220 ** 2]
                pool = near or cands
            else:
                pool = cands
            # Prefer the blob that moved, otherwise the one we already follow, else the largest.
            rover = max(pool, key=lambda c: c[0])
            if self.last_rover is not None:
                moved = max(pool, key=lambda c: (c[1] - self.last_rover[0]) ** 2 + (c[2] - self.last_rover[1]) ** 2)
                shift = (moved[1] - self.last_rover[0]) ** 2 + (moved[2] - self.last_rover[1]) ** 2
                if 8 ** 2 < shift < 180 ** 2:
                    rover = moved
            self.last_rover = (rover[1], rover[2])
        else:
            self.last_rover = None
        data = [0.0] * 7
        if rover:
            _, cx, cy, bw, bh = rover
            data[0], data[1], data[2], data[3] = cx, cy, bw, bh
        if orange:
            data[4], data[5], data[6] = orange
        msg.data = [float(x) for x in data]
        self.pub.publish(msg)
        now = time.monotonic()
        if now - self.last_dbg > 2.0:
            self.last_dbg = now
            vis = frame
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
