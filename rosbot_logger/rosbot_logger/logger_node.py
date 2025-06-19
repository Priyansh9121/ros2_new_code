#!/usr/bin/env python3
"""
logger_node.py – logs /goal_pose and /detected_objects events
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from datetime import datetime


class LoggerNode(Node):
    """Simple event logger."""

    def __init__(self) -> None:
        super().__init__("logger_node")

        # ─── parameters ────────────────────────────────────────────────────
        self.declare_parameter("log_to_file", True)
        self.log_to_file: bool = self.get_parameter("log_to_file").value

        # ─── subscriptions ────────────────────────────────────────────────
        self.create_subscription(PoseStamped,
                                 "/goal_pose",
                                 self.goal_callback,
                                 10)
        self.create_subscription(String,
                                 "/detected_objects",
                                 self.detect_callback,
                                 10)

        # ─── file sink ────────────────────────────────────────────────────
        self.logfile = None
        if self.log_to_file:
            self.logfile = open("/tmp/rosbot_events.log", "a")
            self.get_logger().info("Writing log to /tmp/rosbot_events.log")

    # ──────────────────────────────────────────────────────────────────────
    def _write(self, text: str) -> None:
        ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        line = f"[{ts}] {text}"
        if self.logfile:
            self.logfile.write(line + "\n")
            self.logfile.flush()
        self.get_logger().info(line)

    # ─── callbacks ───────────────────────────────────────────────────────
    def goal_callback(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        self._write(f"New goal received: ({p.x:.2f}, {p.y:.2f})")

    def detect_callback(self, msg: String) -> None:
        self._write(f"Objects detected: {msg.data}")

    # ──────────────────────────────────────────────────────────────────────
    def close(self) -> None:
        if self.logfile:
            self.logfile.close()


# ───────────────────────── entry-point for colcon ────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = LoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()

