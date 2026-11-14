#!/usr/bin/env python3
"""In-place left pivot only. Session armed. No reverse, no forward."""
import math, time
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

W = 0.45
TURN_RAD = math.radians(45)
HZ = 20

def main():
    rclpy.init()
    node = rclpy.create_node("corner_pivot")
    cmd = node.create_publisher(Twist, "/cmd_vel", 10)
    ses = node.create_publisher(Bool, "/rover/session", 10)
    time.sleep(0.4)
    on = Bool(); on.data = True
    tw = Twist(); tw.angular.z = W
    dt = 1.0 / HZ
    dur = TURN_RAD / W
    print(f"pivoting left {math.degrees(TURN_RAD):.0f} deg for {dur:.1f}s", flush=True)
    t0 = time.time()
    while time.time() - t0 < dur:
        ses.publish(on)
        cmd.publish(tw)
        time.sleep(dt)
    cmd.publish(Twist())
    ses.publish(on)
    time.sleep(0.2)
    print("stopped", flush=True)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
