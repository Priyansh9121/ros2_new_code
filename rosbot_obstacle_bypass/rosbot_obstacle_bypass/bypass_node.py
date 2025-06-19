#!/usr/bin/env python3
"""
Obstacle-bypass node that works even when tf_transformations
is not installed on the robot.
"""

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

# ───────────────────────── tf fallback ──────────────────────────
try:
    from tf_transformations import euler_from_quaternion, quaternion_from_euler
except ImportError:                         # yaw-only replacements
    def quaternion_from_euler(_r: float, _p: float, yaw: float):
        half = yaw * 0.5
        return (0.0, 0.0, math.sin(half), math.cos(half))

    def euler_from_quaternion(q):
        x, y, z, w = q
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return (0.0, 0.0, math.atan2(siny_cosp, cosy_cosp))
# ─────────────────────────────────────────────────────────────────

class ObstacleBypassNode(Node):
    def __init__(self):
        super().__init__("obstacle_bypass_node")

        self.declare_parameter("obstacle_threshold", 0.5)
        self.declare_parameter("base_detour_offset", 1.0)
        self.obstacle_threshold = self.get_parameter("obstacle_threshold").value
        self.base_detour_offset = self.get_parameter("base_detour_offset").value

        self.current_pose = None
        self.state = "following_goal"
        self.original_goal = None
        self.detour_goal = None
        self.last_detour_time = 0.0

        self.create_subscription(LaserScan, "/scan", self.scan_cb, 10)
        self.create_subscription(Odometry, "/odom", self.odom_cb, 10)
        self.create_subscription(PoseStamped, "/goal_pose", self.goal_cb, 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)

    # ──────────── callbacks ─────────────
    def goal_cb(self, msg: PoseStamped):
        self.original_goal = msg
        self.get_logger().info(
            "Received original goal" if self.state == "following_goal"
            else "Stored new original goal while detouring")

    def odom_cb(self, msg: Odometry):
        self.current_pose = msg.pose.pose
        if self.state == "detouring" and self.detour_goal:
            dx = self.detour_goal.pose.position.x - self.current_pose.position.x
            dy = self.detour_goal.pose.position.y - self.current_pose.position.y
            if math.hypot(dx, dy) < 0.3:
                self.get_logger().info("Detour reached → resume original goal")
                self.state = "following_goal"
                if self.original_goal:
                    self.goal_pub.publish(self.original_goal)
                self.detour_goal = None

    def scan_cb(self, scan: LaserScan):
        if self.current_pose is None or self.state == "detouring":
            return
        mid = len(scan.ranges) // 2
        if scan.ranges and min(scan.ranges[mid - 10: mid + 10]) < self.obstacle_threshold:
            width = self.estimate_width(scan)
            self.send_detour(self.base_detour_offset + width / 2.0)

    # ───────── helper routines ──────────
    def estimate_width(self, scan: LaserScan) -> float:
        hits = sum(1 for r in scan.ranges if r < self.obstacle_threshold)
        w = hits * scan.angle_increment * self.obstacle_threshold
        self.get_logger().info(f"Obstacle width ≈ {w:.2f} rad")
        return w

    def send_detour(self, offset: float):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()

        x = self.current_pose.position.x
        y = self.current_pose.position.y
        _, _, yaw = euler_from_quaternion([
            self.current_pose.orientation.x,
            self.current_pose.orientation.y,
            self.current_pose.orientation.z,
            self.current_pose.orientation.w])

        pose.pose.position.x = x + offset * math.cos(yaw)
        pose.pose.position.y = y + offset * math.sin(yaw + math.pi / 2.0)

        q = quaternion_from_euler(0.0, 0.0, yaw)
        pose.pose.orientation.x, pose.pose.orientation.y, \
        pose.pose.orientation.z, pose.pose.orientation.w = q

        self.get_logger().info(f"Publishing detour (offset {offset:.2f} m)")
        self.goal_pub.publish(pose)
        self.detour_goal = pose
        self.state = "detouring"
        self.last_detour_time = time.time()

# ──────────── main ─────────────
def main(args=None):
    rclpy.init(args=args)
    node = ObstacleBypassNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

