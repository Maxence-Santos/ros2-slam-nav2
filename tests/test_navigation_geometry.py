#!/usr/bin/env python3
"""Unit tests for pure navigation / threat-detection geometry (no ROS required)."""
from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

import numpy as np

# Allow importing package modules without a full colcon install
PKG_ROOT = Path(__file__).resolve().parents[1] / "src" / "slam_robot"
sys.path.insert(0, str(PKG_ROOT))

from slam_robot.navigation_geometry import (  # noqa: E402
    astar_grid,
    is_flank_threat,
    is_oncoming,
    is_rear_threat,
    is_remerge_blocked,
    lateral_evasion_target,
    select_evasion_side,
    sector_distances_from_scan,
    should_end_evasion,
    update_closing_speed,
)


class TestSectorDistances(unittest.TestCase):
    def test_empty_scan_returns_defaults(self):
        sectors = sector_distances_from_scan([], 0.0, 0.01)
        self.assertEqual(sectors.front, 999.0)
        self.assertEqual(sectors.left, 999.0)
        self.assertEqual(sectors.right, 999.0)
        self.assertEqual(sectors.rear, 999.0)

    def test_four_sector_minima(self):
        # angles: 0° front, 90° left, -90° right, 180° rear
        angles = [0.0, math.pi / 2, -math.pi / 2, math.pi]
        ranges = [1.2, 0.8, 0.5, 0.3]
        # Build dense scan: only set those angle indices via angle_min/inc
        # Use angle_min = -pi, increment such that we can place values by index
        angle_min = -math.pi
        angle_inc = math.pi / 180.0  # 1°
        n = int(2 * math.pi / angle_inc) + 1
        full_ranges = [float("inf")] * n

        def set_range(angle_rad: float, value: float) -> None:
            idx = int(round((angle_rad - angle_min) / angle_inc))
            idx = max(0, min(n - 1, idx))
            full_ranges[idx] = value

        for a, r in zip(angles, ranges):
            set_range(a, r)

        sectors = sector_distances_from_scan(full_ranges, angle_min, angle_inc)
        self.assertAlmostEqual(sectors.front, 1.2, places=5)
        self.assertAlmostEqual(sectors.left, 0.8, places=5)
        self.assertAlmostEqual(sectors.right, 0.5, places=5)
        self.assertAlmostEqual(sectors.rear, 0.3, places=5)

    def test_ignores_nan_and_below_min(self):
        angle_min = -0.5
        angle_inc = 0.5
        ranges = [float("nan"), 0.01, 2.0]  # mid below range_min=0.05
        sectors = sector_distances_from_scan(
            ranges, angle_min, angle_inc, range_min=0.05
        )
        self.assertAlmostEqual(sectors.front, 2.0, places=5)


class TestClosingSpeed(unittest.TestCase):
    def test_first_sample_keeps_previous_speed(self):
        speed, last = update_closing_speed(None, 1.0, 0.0)
        self.assertEqual(speed, 0.0)
        self.assertEqual(last, 1.0)

    def test_closing_in_is_negative(self):
        # distance drops from 2.0 to 1.0 in 0.05s => raw rate -20 m/s
        speed, _ = update_closing_speed(2.0, 1.0, 0.0, dt=0.05, alpha=1.0)
        self.assertLess(speed, 0.0)
        self.assertAlmostEqual(speed, -20.0, places=5)


class TestThreatDetectors(unittest.TestCase):
    def test_oncoming(self):
        self.assertTrue(is_oncoming(1.0, -0.2))
        self.assertFalse(is_oncoming(2.5, -0.2))
        self.assertFalse(is_oncoming(1.0, 0.1))

    def test_rear_threat(self):
        self.assertTrue(is_rear_threat(0.5, -0.1))
        self.assertFalse(is_rear_threat(1.0, -0.1))

    def test_flank_threat_either_side(self):
        self.assertTrue(is_flank_threat(0.5, -0.1, 2.0, 0.0))
        self.assertTrue(is_flank_threat(2.0, 0.0, 0.4, -0.2))
        self.assertFalse(is_flank_threat(2.0, -0.2, 2.0, -0.2))

    def test_remerge_blocked(self):
        self.assertTrue(is_remerge_blocked(False, 0.5, 2.0, 1.0))
        self.assertFalse(is_remerge_blocked(True, 0.5, 2.0, 1.0))  # still evading
        self.assertFalse(is_remerge_blocked(False, 2.0, 2.0, 1.0))


class TestEvasionSide(unittest.TestCase):
    def test_prefers_left_when_goal_left_and_clear(self):
        # path forward +x, goal to the left (+y) => cross > 0
        side, label = select_evasion_side(1.0, 0.0, 0.0, 1.0, left_clearance=1.0, right_clearance=1.0)
        self.assertEqual(side, -1.0)
        self.assertIn("LEFT", label)

    def test_prefers_right_when_goal_right_and_clear(self):
        side, label = select_evasion_side(1.0, 0.0, 0.0, -1.0, left_clearance=1.0, right_clearance=1.0)
        self.assertEqual(side, 1.0)
        self.assertIn("RIGHT", label)

    def test_fallback_right_when_preferred_side_blocked(self):
        # Goal left but left blocked; right open => fallback right
        side, label = select_evasion_side(
            1.0, 0.0, 0.0, 1.0, left_clearance=0.1, right_clearance=1.0
        )
        self.assertEqual(side, 1.0)
        self.assertIn("fallback", label)


class TestEvasionGeometry(unittest.TestCase):
    def test_right_shift_moves_positive_y_for_path_along_x(self):
        # path +x, right normal = (0, -1) wait: n_r = (uy, -ux) = (0, -1) => right is -y
        ex, ey = lateral_evasion_target(
            0.0, 0.0, 1.0, 0.0, evasion_side=1.0,
            left_clearance=2.0, right_clearance=2.0,
            max_lateral=0.35, min_lateral=0.35, wall_margin=0.0,
        )
        self.assertGreater(ex, 0.0)  # still advances forward
        self.assertAlmostEqual(ey, -0.35, places=5)

    def test_left_shift_moves_positive_y(self):
        ex, ey = lateral_evasion_target(
            0.0, 0.0, 1.0, 0.0, evasion_side=-1.0,
            left_clearance=2.0, right_clearance=2.0,
            max_lateral=0.35, min_lateral=0.35, wall_margin=0.0,
        )
        self.assertAlmostEqual(ey, 0.35, places=5)

    def test_end_evasion_wall_hazard(self):
        end, reason = should_end_evasion(0.1, False, False, front_dist=0.2, left_dist=1.0, right_dist=1.0)
        self.assertTrue(end)
        self.assertEqual(reason, "Wall hazard")

    def test_end_evasion_requires_hysteresis(self):
        end, _ = should_end_evasion(0.5, True, True, front_dist=2.0, left_dist=2.0, right_dist=2.0)
        self.assertFalse(end)
        end, reason = should_end_evasion(1.0, True, True, front_dist=2.0, left_dist=2.0, right_dist=2.0)
        self.assertTrue(end)
        self.assertEqual(reason, "Flank clear")


class TestAStar(unittest.TestCase):
    def test_finds_path_around_wall(self):
        grid = np.zeros((10, 10), dtype=np.uint8)
        grid[:, 5] = 1  # vertical wall
        grid[5, 5] = 0  # doorway
        path = astar_grid(grid, (0, 0), (0, 9), clear_radius=0)
        self.assertTrue(path)
        self.assertEqual(path[-1], (0, 9))

    def test_no_path_when_blocked(self):
        grid = np.zeros((5, 5), dtype=np.uint8)
        grid[:, 2] = 1  # solid wall, no door
        path = astar_grid(grid, (0, 0), (0, 4), clear_radius=0)
        self.assertEqual(path, [])


if __name__ == "__main__":
    unittest.main()
