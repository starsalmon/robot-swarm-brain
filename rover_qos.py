"""QoS profiles for Rover micro-ROS topics."""
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

# Match micro-ROS — RELIABLE proved delivery; best_effort --once often drops
CMD_VEL_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE,
)

# T-Display telemetry is BEST_EFFORT (IMU, ticks, drive_diag, battery, IR).
# A best-effort subscriber also receives today's reliable firmware.
TELEMETRY_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE,
)
