#!/usr/bin/env python3
"""Rotate left 45 deg, then drive straight ~0.8 m. Publishes /rover/session so the
T-Display will accept /cmd_vel. Does not touch allow_drive. No reverse."""
import math
import time
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

W = 0.40
TURN_RAD = math.pi / 4
V = 0.18
FWD_M = 0.80
HZ = 20

def main():
    rclpy.init()
    node = rclpy.create_node("corner_escape")
    cmd = node.create_publisher(Twist, "/cmd_vel", 10)
    ses = node.create_publisher(Bool, "/rover/session", 10)
    time.sleep(0.5)
    on = Bool(); on.data = True
    off = Twist()
    dt = 1.0 / HZ

    def hold(twist, seconds, label):
        print(label, flush=True)
        t0 = time.time()
        while time.time() - t0 < seconds:
            ses.publish(on)
            cmd.publish(twist)
            time.sleep(dt)

    tw = Twist(); tw.angular.z = W
    hold(tw, TURN_RAD / W, f"session on; turning left {math.degrees(TURN_RAD):.0f} deg")
    hold(off, 0.25, "pause")
    tw = Twist(); tw.linear.x = V
    hold(tw, FWD_M / V, f"driving straight {FWD_M} m")
    ses.publish(on)
    cmd.publish(off)
    time.sleep(0.2)
    print("done, stopped", flush=True)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
