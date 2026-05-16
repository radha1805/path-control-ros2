#!/usr/bin/env python3
"""
Trajectory Tracking Controller Node.
Uses proportional controller to follow smooth path.
Logs cross-track error for controller evaluation.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker
import numpy as np


class TrajectoryController(Node):

    def __init__(self):
        super().__init__('trajectory_controller')

        self.path_sub = self.create_subscription(
            Path, '/smooth_path', self.path_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_pub = self.create_publisher(Marker, '/robot_trace', 10)
        self.robot_pub = self.create_publisher(Marker, '/robot_marker', 10)

        self.path_points = []
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.current_target = 0
        self.trace_points = []
        self.goal_reached = False

        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info('Trajectory controller started.')

    def path_callback(self, msg):
        if self.goal_reached:
            return
        self.path_points = [
            (p.pose.position.x, p.pose.position.y)
            for p in msg.poses
        ]

    def publish_trace(self):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'trace'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.04
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        pt = Point()
        pt.x = self.current_x
        pt.y = self.current_y
        pt.z = 0.0
        self.trace_points.append(pt)
        marker.points = self.trace_points
        self.marker_pub.publish(marker)

    def publish_robot_marker(self):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'robot'
        marker.id = 1
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = self.current_x
        marker.pose.position.y = self.current_y
        marker.pose.position.z = 0.0
        marker.pose.orientation.z = float(np.sin(self.current_theta / 2.0))
        marker.pose.orientation.w = float(np.cos(self.current_theta / 2.0))
        marker.scale.x = 0.3
        marker.scale.y = 0.08
        marker.scale.z = 0.08
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        self.robot_pub.publish(marker)

    def control_loop(self):
        if len(self.path_points) == 0:
            return
        if self.current_target >= len(self.path_points):
            if not self.goal_reached:
                self.goal_reached = True
                self.get_logger().info('Goal reached!')
                self.cmd_pub.publish(Twist())
            return

        tx, ty = self.path_points[self.current_target]
        dx = tx - self.current_x
        dy = ty - self.current_y
        distance = np.sqrt(dx**2 + dy**2)

        target_angle = np.arctan2(dy, dx)
        angle_error = target_angle - self.current_theta
        angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))

        linear_vel = 0.15
        angular_vel = 0.8 * angle_error

        cmd = Twist()
        cmd.linear.x = linear_vel
        cmd.angular.z = angular_vel
        self.cmd_pub.publish(cmd)

        dt = 0.05
        self.current_x += linear_vel * np.cos(self.current_theta) * dt
        self.current_y += linear_vel * np.sin(self.current_theta) * dt
        self.current_theta += angular_vel * dt

        # Cross-track error logging
        if self.current_target > 0:
            x1, y1 = self.path_points[self.current_target - 1]
            x2, y2 = self.path_points[self.current_target]
            seg_len = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
            if seg_len > 1e-6:
                cte = abs(
                    (y2 - y1) * self.current_x
                    - (x2 - x1) * self.current_y
                    + x2 * y1
                    - y2 * x1
                ) / seg_len
                if self.current_target % 20 == 0:
                    self.get_logger().info(
                        f'Target {self.current_target} | CTE: {cte:.4f} m')

        if distance < 0.15:
            self.current_target += 1

        self.publish_trace()
        self.publish_robot_marker()


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
