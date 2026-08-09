"""
Pure geometry & threat-detection helpers for social navigation.

Kept free of ROS imports so unit tests can run without a ROS 2 environment.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, List, Sequence, Tuple


# LiDAR sector half-angles (radians) matching nav_controller.scan_callback
FRONT_HALF_ANGLE = 0.61   # ±35°
LEFT_MAX_ANGLE = 1.92     # +110°
RIGHT_MIN_ANGLE = -1.92   # -110°
REAR_ABS_ANGLE = 2.35     # ±135°


@dataclass(frozen=True)
class SectorDistances:
    """Minimum range (m) in each 360° sector. 999.0 means no return."""

    front: float
    left: float
    right: float
    rear: float


def sector_distances_from_scan(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    range_min: float = 0.0,
    empty_value: float = 999.0,
) -> SectorDistances:
    """
    Split a LaserScan-like array into four sector minima.

    Sectors (robot frame, 0 = forward, CCW positive):
      - front : |angle| <= 35°
      - left  :  35° < angle <= 110°
      - right : -110° <= angle < -35°
      - rear  : |angle| > 135°
    """
    front_min = empty_value
    left_min = empty_value
    right_min = empty_value
    rear_min = empty_value

    for i, r in enumerate(ranges):
        if r is None or math.isinf(r) or math.isnan(r) or r < range_min:
            continue
        angle = angle_min + i * angle_increment

        if -FRONT_HALF_ANGLE <= angle <= FRONT_HALF_ANGLE:
            front_min = min(front_min, r)
        elif FRONT_HALF_ANGLE < angle <= LEFT_MAX_ANGLE:
            left_min = min(left_min, r)
        elif RIGHT_MIN_ANGLE <= angle < -FRONT_HALF_ANGLE:
            right_min = min(right_min, r)
        elif abs(angle) > REAR_ABS_ANGLE:
            rear_min = min(rear_min, r)

    return SectorDistances(
        front=front_min,
        left=left_min,
        right=right_min,
        rear=rear_min,
    )


def update_closing_speed(
    previous_distance: float | None,
    current_distance: float,
    previous_speed: float,
    dt: float = 0.05,
    alpha: float = 0.3,
) -> Tuple[float, float]:
    """
    EMA-smoothed range rate dD/dt (m/s).

    Negative speed => obstacle closing in.
    Returns (new_speed, current_distance_for_next_step).
    """
    if previous_distance is None or dt <= 0.0:
        return previous_speed, current_distance
    raw_rate = (current_distance - previous_distance) / dt
    new_speed = (1.0 - alpha) * previous_speed + alpha * raw_rate
    return new_speed, current_distance


def is_oncoming(
    front_dist: float,
    closing_speed: float,
    dist_thresh: float = 1.80,
    speed_thresh: float = -0.08,
) -> bool:
    """True if a front obstacle is close and approaching."""
    return front_dist < dist_thresh and closing_speed < speed_thresh


def is_rear_threat(
    rear_dist: float,
    rear_closing_speed: float,
    dist_thresh: float = 0.75,
    speed_thresh: float = -0.08,
) -> bool:
    return rear_dist < dist_thresh and rear_closing_speed < speed_thresh


def is_flank_threat(
    left_dist: float,
    left_closing_speed: float,
    right_dist: float,
    right_closing_speed: float,
    dist_thresh: float = 0.70,
    speed_thresh: float = -0.08,
) -> bool:
    left_hit = left_dist < dist_thresh and left_closing_speed < speed_thresh
    right_hit = right_dist < dist_thresh and right_closing_speed < speed_thresh
    return left_hit or right_hit


def is_remerge_blocked(
    evading: bool,
    front_dist: float,
    left_dist: float,
    path_progress_dist: float,
    front_thresh: float = 0.85,
    left_thresh: float = 0.65,
    min_path_dist: float = 0.40,
) -> bool:
    """Yield-on-re-merge: stop if corridor blocked after / outside evasion."""
    if evading or path_progress_dist <= min_path_dist:
        return False
    return front_dist < front_thresh or left_dist < left_thresh


def select_evasion_side(
    path_dir_x: float,
    path_dir_y: float,
    to_goal_x: float,
    to_goal_y: float,
    left_clearance: float,
    right_clearance: float,
    prefer_clearance: float = 0.45,
    fallback_clearance: float = 0.40,
) -> Tuple[float, str]:
    """
    Goal-aware left/right lane selection.

    cross_product = u × d (2D). Positive => goal is to the LEFT of path direction.
    Returns (side, label) where side = +1.0 (right) or -1.0 (left).
    """
    cross_prod = path_dir_x * to_goal_y - path_dir_y * to_goal_x
    prefer_left = cross_prod > 0.0 and left_clearance > prefer_clearance
    prefer_right = cross_prod <= 0.0 and right_clearance > prefer_clearance

    if prefer_left:
        return -1.0, "LEFT (towards goal)"
    if prefer_right:
        return 1.0, "RIGHT (towards goal)"
    if right_clearance > fallback_clearance:
        return 1.0, "RIGHT (fallback)"
    return -1.0, "LEFT (fallback)"


def lateral_evasion_target(
    robot_x: float,
    robot_y: float,
    path_ux: float,
    path_uy: float,
    evasion_side: float,
    left_clearance: float,
    right_clearance: float,
    forward_offset: float = 0.80,
    max_lateral: float = 0.35,
    min_lateral: float = 0.15,
    wall_margin: float = 0.40,
) -> Tuple[float, float]:
    """
    Virtual waypoint shifted laterally for closed-loop evasion.

    evasion_side > 0 => shift along right normal (uy, -ux).
    """
    # Right normal to path direction
    n_rx = path_uy
    n_ry = -path_ux

    if evasion_side > 0:
        safe_offset = min(max_lateral, max(min_lateral, right_clearance - wall_margin))
        evade_x = robot_x + forward_offset * path_ux + safe_offset * n_rx
        evade_y = robot_y + forward_offset * path_uy + safe_offset * n_ry
    else:
        safe_offset = min(max_lateral, max(min_lateral, left_clearance - wall_margin))
        evade_x = robot_x + forward_offset * path_ux - safe_offset * n_rx
        evade_y = robot_y + forward_offset * path_uy - safe_offset * n_ry
    return evade_x, evade_y


def should_end_evasion(
    time_in_evasion: float,
    timer_expired: bool,
    flank_clear: bool,
    front_dist: float,
    left_dist: float,
    right_dist: float,
    min_evasion_time: float = 0.8,
    wall_front: float = 0.50,
    wall_side: float = 0.30,
) -> Tuple[bool, str]:
    """Dual safety lock: hysteresis + flank clearance, or wall hazard abort."""
    is_wall_danger = (
        front_dist < wall_front
        or right_dist < wall_side
        or left_dist < wall_side
    )
    if is_wall_danger:
        return True, "Wall hazard"
    if time_in_evasion >= min_evasion_time and timer_expired and flank_clear:
        return True, "Flank clear"
    return False, ""


def inflate_occupancy(
    occupied_mask: "object",
    inflation_cells: int,
) -> "object":
    """
    Binary inflate True cells by chebyshev radius inflation_cells.

    occupied_mask: 2D numpy bool/uint8 array.
    Returns uint8 grid (1 = blocked, 0 = free).
    """
    import numpy as np

    h, w = occupied_mask.shape
    grid = np.zeros((h, w), dtype=np.uint8)
    rows, cols = np.where(occupied_mask)
    r = max(0, int(inflation_cells))
    for row, col in zip(rows, cols):
        r0 = max(0, row - r)
        r1 = min(h, row + r + 1)
        c0 = max(0, col - r)
        c1 = min(w, col + r + 1)
        grid[r0:r1, c0:c1] = 1
    return grid


def astar_grid(
    grid,
    start: Tuple[int, int],
    goal: Tuple[int, int],
    clear_radius: int = 3,
) -> List[Tuple[int, int]]:
    """
    8-connected A* on a binary occupancy grid (1 = blocked).

    Returns list of (row, col) from start+1 step to goal, or empty if no path.
    """
    import heapq

    import numpy as np

    height, width = grid.shape
    work = grid.copy()
    s_row, s_col = start
    g_row, g_col = goal
    s_row = max(0, min(height - 1, s_row))
    s_col = max(0, min(width - 1, s_col))
    g_row = max(0, min(height - 1, g_row))
    g_col = max(0, min(width - 1, g_col))

    for dr in range(-clear_radius, clear_radius + 1):
        for dc in range(-clear_radius, clear_radius + 1):
            for r, c in ((s_row + dr, s_col + dc), (g_row + dr, g_col + dc)):
                if 0 <= r < height and 0 <= c < width:
                    work[r, c] = 0

    open_set: list = []
    heapq.heappush(open_set, (0.0, (s_row, s_col)))
    came_from = {}
    g_score = {(s_row, s_col): 0.0}

    def heuristic(r: int, c: int) -> float:
        return math.hypot(r - g_row, c - g_col)

    neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
    found = False
    iterations = 0
    max_iterations = width * height

    while open_set and iterations < max_iterations:
        iterations += 1
        _, current = heapq.heappop(open_set)
        if current == (g_row, g_col):
            found = True
            break
        r, c = current
        for dr, dc in neighbors:
            nr, nc = r + dr, c + dc
            if not (0 <= nr < height and 0 <= nc < width):
                continue
            if work[nr, nc] == 1:
                continue
            tentative = g_score[current] + math.hypot(dr, dc)
            if (nr, nc) not in g_score or tentative < g_score[(nr, nc)]:
                g_score[(nr, nc)] = tentative
                heapq.heappush(open_set, (tentative + heuristic(nr, nc), (nr, nc)))
                came_from[(nr, nc)] = current

    if not found:
        return []

    path_cells: List[Tuple[int, int]] = []
    curr = (g_row, g_col)
    while curr in came_from:
        path_cells.append(curr)
        curr = came_from[curr]
    path_cells.reverse()
    return path_cells
