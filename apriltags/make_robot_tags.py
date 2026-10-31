#!/usr/bin/env python3
"""Printable tag36h11 robot ID tags (vector A4 PDF) + OpenCV detection check.
IDs 0-39 are already used by the room/dock sets (tags_100mm*, tags_80mm), so robots use 100+.
Stated size = black square edge (8 cells). White quiet zone = 2 cells each side.
Usage: python3 make_robot_tags.py [out.pdf]   (needs opencv-contrib>=4.7, reportlab, pdftoppm for --verify)"""
import sys, subprocess, cv2, numpy as np
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.pdfgen import canvas

ROBOTS = [("Rover", 100), ("Beeper", 101), ("Jaz", 102), ("Tracker", 103), ("Spare", 104)]
SIZES_MM = [30, 40, 50]
QUIET = 2          # cells of white around the black square
D = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)

def bits(i):  # 8x8, 1 = white
    return (cv2.aruco.generateImageMarker(D, i, 8) > 127)

def draw_tag(c, x, y, size, i, label):
    cell = size / 8; q = QUIET * cell
    c.setStrokeColorRGB(.7, .7, .7); c.setLineWidth(.3); c.setDash(1, 2)
    c.rect(x - q, y - q, size + 2 * q, size + 2 * q)           # cut line = outer edge of quiet zone
    c.setDash(); c.setFillColorRGB(0, 0, 0)
    b = bits(i)
    for r in range(8):
        for col in range(8):
            if not b[r, col]:
                c.rect(x + col * cell, y + (7 - r) * cell, cell, cell, stroke=0, fill=1)
    c.setFont("Helvetica", 6.5)
    c.drawString(x - q, y - q - 3 * mm, f"{label}  ID {i}  {size / mm:.0f} mm")

def ruler(c, x, y):
    c.setLineWidth(.5); c.line(x, y, x + 50 * mm, y)
    for k in range(6):
        h = 3 * mm if k % 5 == 0 else 1.5 * mm
        c.line(x + k * 10 * mm, y, x + k * 10 * mm, y + h)
    c.setFont("Helvetica", 7); c.drawString(x, y - 3.5 * mm, "50 mm calibration line: measure it. Print at 100% / actual size (no 'fit to page').")

def build(out):
    W, H = A4; c = canvas.Canvas(out, pagesize=A4)
    c.setTitle("tag36h11 robot tags")
    pages = [ROBOTS[:3], ROBOTS[3:]]
    for pn, rows in enumerate(pages):
        c.setFont("Helvetica-Bold", 10)
        c.drawString(15 * mm, H - 14 * mm, f"Robot swarm tags - AprilTag tag36h11 - size = black square edge - page {pn + 1}/2")
        ruler(c, 15 * mm, H - 24 * mm)
        ytop = H - 40 * mm
        for name, i in rows:
            x = 15 * mm
            ybase = ytop - 50 * mm
            for s in SIZES_MM:
                size = s * mm
                draw_tag(c, x, ybase, size, i, name)
                x += size + 2 * QUIET * size / 8 + 5 * mm
            ytop -= 50 * mm + 2 * QUIET * 50 * mm / 8 + 12 * mm
        c.showPage()
    c.save()

def verify(out, dpi=300):
    pre = out[:-4]
    subprocess.run(["pdftoppm", "-r", str(dpi), "-gray", "-png", out, pre + "_r"], check=True)
    det = cv2.aruco.ArucoDetector(D, cv2.aruco.DetectorParameters())
    want = {(i, s) for _, i in ROBOTS for s in SIZES_MM}; got = set(); ok = True
    import glob
    for f in sorted(glob.glob(pre + "_r-*.png")):
        g = cv2.imread(f, 0); cs, ids, _ = det.detectMarkers(g)
        for cc, i in zip(cs, ids.flatten() if ids is not None else []):
            e = np.mean([np.linalg.norm(cc[0][k] - cc[0][(k + 1) % 4]) for k in range(4)]) / dpi * 25.4
            s = min(SIZES_MM, key=lambda v: abs(v - e)); got.add((int(i), s))
            print(f"{f.split('/')[-1]}: id {int(i)} edge {e:.2f} mm")
            ok &= abs(e - s) < 0.5
    print("missing:", sorted(want - got), "extra:", sorted(got - want))
    return ok and got == want

if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else "robot_tags.pdf"
    build(out); print("PASS" if verify(out) else "FAIL")
