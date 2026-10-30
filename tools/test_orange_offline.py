#!/usr/bin/env python3
"""Offline check of the orange detector: old HSV blob finder vs YOLO class 49 + 4-of-6 vote, on still images.
Usage (in the rover-follow image): python3 tools/test_orange_offline.py img1.jpg img2.jpg ...
Writes <img>_yolo.jpg with the YOLO boxes drawn."""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import cv2, numpy as np
from ultralytics import YOLO
from orange_vote import OrangeVote


def old_hsv(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.morphologyEx(cv2.inRange(hsv, (5, 90, 90), (22, 255, 255)), cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    n = 0
    for c in cnts:
        a = cv2.contourArea(c); p = cv2.arcLength(c, True)
        if 200 <= a <= 4000 and 4 * np.pi * a / max(p * p, 1) >= 0.55: n += 1
    return n


m = YOLO(os.environ.get("ROOM_ORANGE_MODEL", "/weights/yolo11s.pt"))
for path in sys.argv[1:]:
    img = cv2.imread(path); h, w = img.shape[:2]
    v = OrangeVote(need=4, window=6, max_jump=0.08); acc = None; boxes = []
    for k in range(6):                                    # same still 6 times = best case for a false positive
        r = m.predict(img, classes=[49], conf=0.40, imgsz=1280, verbose=False)[0]
        boxes = [(*map(float, b), float(c)) for b, c in zip(r.boxes.xyxy.cpu().numpy(), r.boxes.conf.cpu().numpy())]
        acc = v.update(boxes, w, h) or acc
    print(f"{os.path.basename(path):14s} old HSV blobs={old_hsv(img)}  YOLO oranges={len(boxes)} conf={[round(b[4], 2) for b in boxes][:5]}  accepted={acc is not None}")
    for b in boxes: cv2.rectangle(img, (int(b[0]), int(b[1])), (int(b[2]), int(b[3])), (0, 255, 0), 3)
    cv2.imwrite(os.path.splitext(path)[0] + "_yolo.jpg", img)
