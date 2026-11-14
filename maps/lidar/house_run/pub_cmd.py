#!/usr/bin/env python3
import sys, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, UInt8

# /rover/button has two subscribers (rover_behaviour and rover_brain).
# Publishing after only one match can miss behaviour entirely.
BUTTON_SUBS = 2

def main():
    kind = sys.argv[1] if len(sys.argv) > 1 else "button"
    rclpy.init()
    n = Node("house_map_pub")
    if kind == "button":
        pub = n.create_publisher(Bool, "/rover/button", 10)
        msg = Bool(); msg.data = True
        need = BUTTON_SUBS
    elif kind == "estop":
        pub = n.create_publisher(UInt8, "/rover/button_event", 10)
        msg = UInt8(); msg.data = 3
        need = 1
    else:
        print("usage: pub_cmd.py button|estop")
        return 2
    t0 = time.time()
    count = 0
    while time.time() - t0 < 10:
        rclpy.spin_once(n, timeout_sec=0.1)
        count = pub.get_subscription_count()
        if count >= need:
            time.sleep(0.4)
            rclpy.spin_once(n, timeout_sec=0.1)
            count = pub.get_subscription_count()
            if count >= need:
                break
    pub.publish(msg)
    for _ in range(20):
        rclpy.spin_once(n, timeout_sec=0.05)
    print(f"published {kind} subs={count} need={need}")
    n.destroy_node()
    rclpy.shutdown()
    return 0 if count >= need else 2

if __name__ == "__main__":
    raise SystemExit(main())
