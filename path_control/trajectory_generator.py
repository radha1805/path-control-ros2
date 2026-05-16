#!/usr/bin/env python3
"""
Trajectory Generator Node
Subscribes to smooth path, generates time-stamped trajectory with trapezoidal velocity profile.
Publishes trajectory as a MarkerArray (arrows showing direction + speed).
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from scipy.interpolate import CubicSpline


class TrajectoryGeneratorNode(Node):
    def __init__(self):
        super().__init__('trajectory_generator')

        # Parameters
        self.max_velocity = 0.5      # m/s — peak speed
        self.accel_frac = 0.2        # fraction of path spent accelerating
        self.decel_frac = 0.2        # fraction of path spent decelerating

        # Subscribe to smooth path
        self.path_sub = self.create_subscription(
            Path, '/smooth_path', self.path_callback, 10)

        # Publishers
        self.traj_pub = self.create_publisher(MarkerArray, '/trajectory_viz', 10)

        # Store trajectory for other nodes
        self.trajectory = []  # list of (x, y, t)
        self.get_logger().info('Trajectory generator waiting for smooth path...')

    def path_callback(self, path_msg):
        """Called when smooth path arrives. Generate time-stamped trajectory."""
        if not path_msg.poses:
            return

        points = [(p.pose.position.x, p.pose.position.y) for p in path_msg.poses]
        self.trajectory = self.generate_trajectory(points)
        self.publish_trajectory_viz()
        self.get_logger().info(
            f'Trajectory generated: {len(self.trajectory)} points, '
            f'total time = {self.trajectory[-1][2]:.2f}s'
        )

    def generate_trajectory(self, points):
        """
        Given smooth path points, assign timestamps using trapezoidal velocity profile.
        Returns: list of (x, y, t)
        """
        pts = np.array(points)

        # Compute cumulative arc length
        diffs = np.diff(pts, axis=0)
        seg_lengths = np.sqrt((diffs ** 2).sum(axis=1))
        cum_dist = np.concatenate([[0], np.cumsum(seg_lengths)])
        total_dist = cum_dist[-1]

        # Trapezoidal velocity profile
        # Phase 1: accelerate from 0 to max_vel
        # Phase 2: cruise at max_vel
        # Phase 3: decelerate from max_vel to 0
        d_accel = self.accel_frac * total_dist
        d_decel = self.decel_frac * total_dist
        d_cruise = total_dist - d_accel - d_decel

        def velocity_at_distance(d):
            """Returns velocity (m/s) at distance d along path."""
            if d <= d_accel:
                return self.max_velocity * (d / d_accel) if d_accel > 0 else self.max_velocity
            elif d <= d_accel + d_cruise:
                return self.max_velocity
            else:
                remaining = total_dist - d
                return self.max_velocity * (remaining / d_decel) if d_decel > 0 else 0.0

        # Integrate time: dt = ds / v
        timestamps = [0.0]
        for i in range(1, len(cum_dist)):
            ds = cum_dist[i] - cum_dist[i - 1]
            v_mid = velocity_at_distance((cum_dist[i] + cum_dist[i - 1]) / 2.0)
            v_mid = max(v_mid, 0.05)  # avoid division by zero
            dt = ds / v_mid
            timestamps.append(timestamps[-1] + dt)

        trajectory = [(pts[i, 0], pts[i, 1], timestamps[i]) for i in range(len(pts))]
        return trajectory

    def publish_trajectory_viz(self):
        """Publish trajectory as blue arrows in RViz (every 10th point)."""
        if not self.trajectory:
            return

        now = self.get_clock().now().to_msg()
        marker_array = MarkerArray()

        step = max(1, len(self.trajectory) // 40)  # show ~40 arrows

        for i in range(0, len(self.trajectory) - step, step):
            x0, y0, t0 = self.trajectory[i]
            x1, y1, t1 = self.trajectory[i + step]

            m = Marker()
            m.header.stamp = now
            m.header.frame_id = 'map'
            m.ns = 'trajectory'
            m.id = i
            m.type = Marker.ARROW
            m.action = Marker.ADD

            # Arrow from current to next point
            from geometry_msgs.msg import Point
            start = Point(x=x0, y=y0, z=0.0)
            end = Point(x=x1, y=y1, z=0.0)
            m.points = [start, end]

            m.scale.x = 0.04   # shaft diameter
            m.scale.y = 0.08   # head diameter
            m.scale.z = 0.0

            # Color: blue
            m.color.r = 0.0
            m.color.g = 0.3
            m.color.b = 1.0
            m.color.a = 0.8

            marker_array.markers.append(m)

        self.traj_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryGeneratorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
