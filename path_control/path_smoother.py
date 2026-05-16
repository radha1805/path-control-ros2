#!/usr/bin/env python3
"""
Path Smoother Node
Reads discrete waypoints, fits a cubic spline, publishes smooth path.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
import numpy as np
from scipy.interpolate import CubicSpline


class PathSmootherNode(Node):
    def __init__(self):
        super().__init__('path_smoother')

        # Publisher: smooth path
        self.path_pub = self.create_publisher(Path, '/smooth_path', 10)
        # Publisher: original waypoints as markers (red spheres)
        self.waypoint_pub = self.create_publisher(MarkerArray, '/waypoints_viz', 10)

        # Timer: publish at 1 Hz (once per second)
        self.timer = self.create_timer(1.0, self.publish_path)

        # ---- DEFINE YOUR WAYPOINTS HERE ----
        # These are (x, y) positions in meters
        self.waypoints = [
            (0.0, 0.0),
            (1.0, 0.5),
            (2.0, 2.0),
            (3.5, 2.5),
            (5.0, 1.5),
            (6.0, 3.0),
            (7.0, 4.0),
        ]

        # Generate smooth path once
        self.smooth_path = self.smooth_waypoints(self.waypoints, num_points=200)
        self.get_logger().info(f'Path smoother ready. {len(self.smooth_path)} smooth points generated.')

    def smooth_waypoints(self, waypoints, num_points=200):
        """
        Takes discrete waypoints, returns smooth (x,y) points using cubic spline.
        Algorithm: Parametric cubic spline (t = cumulative arc length).
        """
        wp = np.array(waypoints)
        n = len(wp)

        # Parameterize by cumulative distance (arc-length parameterization)
        diffs = np.diff(wp, axis=0)
        distances = np.sqrt((diffs ** 2).sum(axis=1))
        t = np.concatenate([[0], np.cumsum(distances)])

        # Fit independent cubic splines for x(t) and y(t)
        cs_x = CubicSpline(t, wp[:, 0])
        cs_y = CubicSpline(t, wp[:, 1])

        # Sample the spline at evenly spaced t values
        t_fine = np.linspace(t[0], t[-1], num_points)
        x_smooth = cs_x(t_fine)
        y_smooth = cs_y(t_fine)

        return list(zip(x_smooth, y_smooth))

    def publish_path(self):
        """Publish the smooth path as nav_msgs/Path"""
        now = self.get_clock().now().to_msg()
        frame = 'map'

        # --- Publish smooth path (green line in RViz) ---
        path_msg = Path()
        path_msg.header.stamp = now
        path_msg.header.frame_id = frame

        for (x, y) in self.smooth_path:
            pose = PoseStamped()
            pose.header.stamp = now
            pose.header.frame_id = frame
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

        # --- Publish original waypoints as red spheres ---
        marker_array = MarkerArray()
        for i, (x, y) in enumerate(self.waypoints):
            m = Marker()
            m.header.stamp = now
            m.header.frame_id = frame
            m.ns = 'waypoints'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0.0
            m.pose.orientation.w = 1.0
            m.scale.x = 0.15
            m.scale.y = 0.15
            m.scale.z = 0.15
            m.color.r = 1.0
            m.color.g = 0.0
            m.color.b = 0.0
            m.color.a = 1.0
            marker_array.markers.append(m)

        self.waypoint_pub.publish(marker_array)
        self.get_logger().info('Published smooth path and waypoints.')


def main(args=None):
    rclpy.init(args=args)
    node = PathSmootherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
