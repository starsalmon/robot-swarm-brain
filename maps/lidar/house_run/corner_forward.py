#!/usr/bin/env python3
import time
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

V = 0.18
FWD_M = 0.80
HZ = 20

def main():
    rclpy.init()
    node = rclpy.create_node("corner_forward")
    cmd = node.create_publisher(Twist, "/cmd_vel", 10)
    ses = node.create_publisher(Bool, "/rover/session", 10)
    time.sleep(0.3)
    on = Bool(); on.data = True
    tw = Twist(); tw.linear.x = V
    dt = 1.0 / HZ
    dur = FWD_M / V
    print(f"driving straight {FWD_M} m for {dur:.1f}s", flush=True)
    t0 = time.time()
    while time.time() - t0 < dur:
        ses.publish(on)
        cmd.publish(tw)
        time.sleep(dt)
    cmd.publish(Twist())
    time.sleep(0.2)
    print("stopped", flush=True)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
