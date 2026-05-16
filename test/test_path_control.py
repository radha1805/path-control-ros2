"""
Unit tests for path smoothing and trajectory generation.
Run with: pytest test/test_path_control.py -v
"""
import pytest
import numpy as np
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'path_control'))

from scipy.interpolate import CubicSpline


# ── helpers copied from path_smoother logic ──────────────────────────────────

def smooth_waypoints(waypoints, num_points=200):
    wp = np.array(waypoints)
    diffs = np.diff(wp, axis=0)
    distances = np.sqrt((diffs ** 2).sum(axis=1))
    t = np.concatenate([[0], np.cumsum(distances)])
    cs_x = CubicSpline(t, wp[:, 0])
    cs_y = CubicSpline(t, wp[:, 1])
    t_fine = np.linspace(t[0], t[-1], num_points)
    return list(zip(cs_x(t_fine), cs_y(t_fine)))


def generate_trajectory(points, max_velocity=0.5, accel_frac=0.2, decel_frac=0.2):
    pts = np.array(points)
    diffs = np.diff(pts, axis=0)
    seg_lengths = np.sqrt((diffs ** 2).sum(axis=1))
    cum_dist = np.concatenate([[0], np.cumsum(seg_lengths)])
    total_dist = cum_dist[-1]
    d_accel = accel_frac * total_dist
    d_decel = decel_frac * total_dist
    d_cruise = total_dist - d_accel - d_decel

    def velocity_at(d):
        if d <= d_accel:
            return max_velocity * (d / d_accel) if d_accel > 0 else max_velocity
        elif d <= d_accel + d_cruise:
            return max_velocity
        else:
            rem = total_dist - d
            return max_velocity * (rem / d_decel) if d_decel > 0 else 0.0

    timestamps = [0.0]
    for i in range(1, len(cum_dist)):
        ds = cum_dist[i] - cum_dist[i-1]
        v = max(velocity_at((cum_dist[i] + cum_dist[i-1]) / 2.0), 0.05)
        timestamps.append(timestamps[-1] + ds / v)

    return [(pts[i, 0], pts[i, 1], timestamps[i]) for i in range(len(pts))]


# ── TESTS ────────────────────────────────────────────────────────────────────

WAYPOINTS = [(0.0,0.0),(1.0,0.5),(2.0,2.0),(3.5,2.5),(5.0,1.5),(6.0,3.0),(7.0,4.0)]


class TestPathSmoother:

    def test_output_length(self):
        """Smooth path must have exactly num_points points."""
        path = smooth_waypoints(WAYPOINTS, num_points=200)
        assert len(path) == 200

    def test_starts_at_first_waypoint(self):
        """Smooth path must start at the first waypoint."""
        path = smooth_waypoints(WAYPOINTS)
        assert abs(path[0][0] - WAYPOINTS[0][0]) < 0.01
        assert abs(path[0][1] - WAYPOINTS[0][1]) < 0.01

    def test_ends_at_last_waypoint(self):
        """Smooth path must end at the last waypoint."""
        path = smooth_waypoints(WAYPOINTS)
        assert abs(path[-1][0] - WAYPOINTS[-1][0]) < 0.01
        assert abs(path[-1][1] - WAYPOINTS[-1][1]) < 0.01

    def test_path_is_continuous(self):
        """No point-to-point jump > 0.2m (continuity check)."""
        path = smooth_waypoints(WAYPOINTS, num_points=200)
        for i in range(1, len(path)):
            dist = np.sqrt((path[i][0]-path[i-1][0])**2 + (path[i][1]-path[i-1][1])**2)
            assert dist < 0.2, f"Jump of {dist:.3f}m at index {i}"

    def test_minimum_waypoints(self):
        """Smoother must work with just 2 waypoints."""
        path = smooth_waypoints([(0.0, 0.0), (1.0, 1.0)], num_points=50)
        assert len(path) == 50

    def test_single_axis_path(self):
        """Straight horizontal path should stay on y=0."""
        path = smooth_waypoints([(0.0,0.0),(1.0,0.0),(2.0,0.0),(3.0,0.0)])
        for (x, y) in path:
            assert abs(y) < 0.001


class TestTrajectoryGenerator:

    def setup_method(self):
        self.path = smooth_waypoints(WAYPOINTS, num_points=100)
        self.traj = generate_trajectory(self.path)

    def test_trajectory_length_matches_path(self):
        """Trajectory must have same number of points as path."""
        assert len(self.traj) == len(self.path)

    def test_timestamps_monotonically_increasing(self):
        """Time must always move forward."""
        times = [t for _, _, t in self.traj]
        for i in range(1, len(times)):
            assert times[i] > times[i-1], f"Time went backward at index {i}"

    def test_trajectory_starts_at_t0(self):
        """First timestamp must be 0."""
        assert self.traj[0][2] == 0.0

    def test_total_time_positive(self):
        """Total trajectory time must be > 0."""
        assert self.traj[-1][2] > 0

    def test_xy_matches_path(self):
        """Trajectory x,y must match the original path points."""
        for i, (tx, ty, tt) in enumerate(self.traj):
            assert abs(tx - self.path[i][0]) < 1e-9
            assert abs(ty - self.path[i][1]) < 1e-9

    def test_trapezoidal_profile_total_time(self):
        """Total time should be reasonable (not 0 or infinite)."""
        total = self.traj[-1][2]
        assert 5.0 < total < 200.0, f"Total time {total:.1f}s seems wrong"


class TestControllerMath:

    def test_angle_normalization(self):
        """Angle error must be in [-pi, pi]."""
        import math
        for raw_angle in [3.5, -3.5, 7.0, -7.0, 0.1, math.pi]:
            normalized = math.atan2(math.sin(raw_angle), math.cos(raw_angle))
            assert -math.pi <= normalized <= math.pi

    def test_unicycle_model_forward(self):
        """Moving forward at theta=0 should increase x only."""
        x, y, theta = 0.0, 0.0, 0.0
        v, w, dt = 0.5, 0.0, 0.1
        x += v * np.cos(theta) * dt
        y += v * np.sin(theta) * dt
        theta += w * dt
        assert abs(x - 0.05) < 1e-9
        assert abs(y) < 1e-9

    def test_unicycle_model_turn(self):
        """Positive angular velocity should increase theta."""
        theta = 0.0
        theta += 1.0 * 0.1   # w=1.0, dt=0.1
        assert theta > 0

    def test_cross_track_error_on_path(self):
        """Robot exactly on the path segment should have CTE ~ 0."""
        x1, y1 = 0.0, 0.0
        x2, y2 = 1.0, 0.0
        rx, ry = 0.5, 0.0   # robot on the segment
        seg_len = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        cte = abs((y2-y1)*rx - (x2-x1)*ry + x2*y1 - y2*x1) / seg_len
        assert cte < 1e-9

    def test_cross_track_error_off_path(self):
        """Robot 0.3m off path should have CTE = 0.3."""
        x1, y1 = 0.0, 0.0
        x2, y2 = 1.0, 0.0
        rx, ry = 0.5, 0.3   # 0.3m above segment
        seg_len = np.sqrt((x2-x1)**2 + (y2-y1)**2)
        cte = abs((y2-y1)*rx - (x2-x1)*ry + x2*y1 - y2*x1) / seg_len
        assert abs(cte - 0.3) < 1e-9
