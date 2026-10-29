#!/usr/bin/env python3
"""Rover wheel odometry + IMU restamp + static TFs (feeds robot_localization).

Sub  /rover/wheel/left_ticks, right_ticks (UInt32, unsigned, RISING edges), /cmd_vel (for wheel direction),
     /imu/data (ESP clock stamps)
Pub  /rover/odom/wheel (nav_msgs/Odometry, odom->base_link, no TF; EKF owns TF)
     /rover/imu/stamped (Imu restamped to brain clock, covariances set)
     static TF from config/rover_frames.yaml (+ camera_link->camera_optical_frame)
Ticks are unsigned, so each wheel's sign comes from the commanded wheel speed in /cmd_vel
(v -/+ w*track/2), held for ROVER_ODOM_SIGN_HOLD_S after the command drops to 0 (coasting).
Ticks arriving with no non-zero command in that window are rejected (ROVER_ODOM_GATE_IDLE=1): the
unsigned ticks carry no direction without a command, so hand-pushes are ignored rather than guessed.
"""
import math, os, time
import yaml, rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import UInt32
from tf2_ros import StaticTransformBroadcaster

CFG = os.environ.get("ROVER_FRAMES_YAML", "/opt/fleet/config/rover_frames.yaml")
HOLD = float(os.environ.get("ROVER_ODOM_SIGN_HOLD_S", "1.5"))
GATE = os.environ.get("ROVER_ODOM_GATE_IDLE", "1") == "1"  # drop ticks with no recent drive cmd


def quat(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r/2), math.sin(r/2), math.cos(p/2), math.sin(p/2), math.cos(y/2), math.sin(y/2)
    return (sr*cp*cy - cr*sp*sy, cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy, cr*cp*cy + sr*sp*sy)


class RoverOdom(Node):
    def __init__(self):
        super().__init__("rover_odom")
        c = yaml.safe_load(open(CFG))
        w = c["wheel"]
        self.m_tick = math.pi * w["diameter_m"] / w["slits_per_rev"]
        self.track = w["track_m"]
        self.tf_static = StaticTransformBroadcaster(self)
        ts = []
        for child, f in c["frames"].items():
            ts.append(self.tf(f["parent"], child, f["xyz"], [math.radians(a) for a in f["rpy_deg"]]))
        ts.append(self.tf("camera_link", "camera_optical_frame", [0, 0, 0], [-math.pi/2, 0, -math.pi/2]))
        self.tf_static.sendTransform(ts)
        self.pub = self.create_publisher(Odometry, "/rover/odom/wheel", 10)
        self.pub_imu = self.create_publisher(Imu, "/rover/imu/stamped", 20)
        self.create_subscription(UInt32, "/rover/wheel/left_ticks", lambda m: self.on_tick(0, m.data), 20)
        self.create_subscription(UInt32, "/rover/wheel/right_ticks", lambda m: self.on_tick(1, m.data), 20)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.create_subscription(Imu, "/imu/data", self.on_imu, 20)
        self.last = [None, None]
        self.sign = [1.0, 1.0]
        self.sign_t = [0.0, 0.0]
        self.acc = [0.0, 0.0]          # signed metres since last odom publish
        self.x = self.y = self.th = 0.0
        self.t_prev = time.monotonic()
        self.vx = self.wz = 0.0
        self.total = [0, 0]
        self.rejected = [0, 0]
        self.create_timer(0.05, self.step)
        self.create_timer(10.0, self.log)
        self.get_logger().info(f"{self.m_tick*1000:.2f} mm/tick, track {self.track*1000:.0f} mm, frames from {CFG}")

    def tf(self, parent, child, xyz, rpy):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = parent, child
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, xyz)
        q = quat(*rpy)
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
        return t

    def on_cmd(self, m):
        now = time.monotonic()
        for i, v in enumerate((m.linear.x - m.angular.z * self.track / 2, m.linear.x + m.angular.z * self.track / 2)):
            if abs(v) > 1e-3:
                self.sign[i] = 1.0 if v > 0 else -1.0
                self.sign_t[i] = now

    def on_tick(self, i, n):
        if self.last[i] is not None:
            d = n - self.last[i]
            if 0 < d < 500:          # negative = ESP reset/wrap; huge = glitch
                if GATE and time.monotonic() - self.sign_t[i] > HOLD:
                    self.rejected[i] += d   # no drive command: phantom ticks (sensor noise / stray IR)
                else:
                    self.acc[i] += self.sign[i] * d * self.m_tick
                    self.total[i] += d
        self.last[i] = n

    def on_imu(self, m):
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "imu_link"
        m.angular_velocity_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.0004]
        m.orientation_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.05]
        m.linear_acceleration_covariance = [0.2, 0, 0, 0, 0.2, 0, 0, 0, 0.2]
        self.pub_imu.publish(m)

    def step(self):
        now = time.monotonic()
        dt = max(1e-3, now - self.t_prev); self.t_prev = now
        dl, dr = self.acc; self.acc = [0.0, 0.0]
        ds, dth = (dl + dr) / 2, (dr - dl) / self.track
        self.x += ds * math.cos(self.th + dth / 2); self.y += ds * math.sin(self.th + dth / 2); self.th += dth
        a = 0.4  # light smoothing: ticks arrive at 10 Hz
        self.vx = (1 - a) * self.vx + a * ds / dt
        self.wz = (1 - a) * self.wz + a * dth / dt
        o = Odometry()
        o.header.stamp = self.get_clock().now().to_msg()
        o.header.frame_id, o.child_frame_id = "odom", "base_link"
        o.pose.pose.position.x, o.pose.pose.position.y = self.x, self.y
        q = quat(0, 0, self.th)
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = q[2], q[3]
        o.pose.covariance = [0.0] * 36
        for k, v in ((0, 0.01), (7, 0.01), (14, 1e3), (21, 1e3), (28, 1e3), (35, 0.1)):
            o.pose.covariance[k] = v
        o.twist.twist.linear.x, o.twist.twist.angular.z = self.vx, self.wz
        o.twist.covariance = [0.0] * 36
        for k, v in ((0, 0.002), (7, 1e-4), (14, 1e3), (21, 1e3), (28, 1e3), (35, 0.02)):
            o.twist.covariance[k] = v
        self.pub.publish(o)

    def log(self):
        self.get_logger().info(f"ticks L{self.total[0]} R{self.total[1]} (idle-rejected L{self.rejected[0]} R{self.rejected[1]}) pose x={self.x:.3f} y={self.y:.3f} th={math.degrees(self.th):.1f}deg")


def main():
    rclpy.init(); n = RoverOdom()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally: n.destroy_node(); rclpy.try_shutdown()


if __name__ == "__main__":
    main()
