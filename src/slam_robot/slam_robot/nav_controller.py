#!/usr/bin/env python3
"""
Unified Autonomous Navigator & Map Server Node.
Subscribes to /clock and stamps /map OccupancyGrid with live sim time
so RViz2 MapDisplay receives valid timestamps and renders the map instantly.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import numpy as np
import math
import heapq
import os
import yaml
import sys
from PIL import Image

class UnifiedNavigatorNode(Node):
    def __init__(self, map_yaml_path):
        super().__init__(
            'autonomous_navigator',
            parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)]
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Clock tracking
        self.current_clock = None
        self.clock_count = 0

        # Robot state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.front_obstacle_dist = 999.0
        self.left_obstacle_dist = 999.0
        self.right_obstacle_dist = 999.0
        self.rear_obstacle_dist = 999.0
        self.last_front_dist = None
        self.last_rear_dist = None
        self.last_left_dist = None
        self.last_right_dist = None
        self.obstacle_closing_speed = 0.0  # dD/dt (negative = closing in, positive = moving away)
        self.rear_closing_speed = 0.0
        self.left_closing_speed = 0.0
        self.right_closing_speed = 0.0
        self.evading_dynamic_obstacle = False
        self.person_yield_start_time = None
        self.evasion_start_time = 0.0
        self.evasion_end_time = 0.0
        self.evasion_side = 1.0  # +1.0 for right shift, -1.0 for left shift

        # Navigation state
        self.target_x = None
        self.target_y = None
        self.path_points = []
        self.current_idx = 0
        self.is_navigating = False

        # Transient Local QoS profile for /map
        map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        # Publishers
        self.static_map_pub = self.create_publisher(OccupancyGrid, '/static_map', map_qos)
        self.live_map_pub = self.create_publisher(OccupancyGrid, '/map', map_qos)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd_vel_model_pub = self.create_publisher(Twist, '/model/slam_robot/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/plan', 10)

        # Load map from disk
        self.map_grid = None
        self.map_msg = None
        self.load_map_from_disk(map_yaml_path)

        # Subscribers
        self.create_subscription(Clock, '/clock', self.clock_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Motion control loop (20Hz)
        self.create_timer(0.05, self.control_loop)

        # Startup grace period (ignore stale goals for first 5 seconds)
        self.startup_time = self.get_clock().now()
        self.startup_grace_seconds = 5.0

        self.get_logger().info('🚀 Unified Autonomous Navigator & Map Server initialized!')

    def clock_callback(self, msg):
        self.current_clock = msg.clock
        self.clock_count += 1

        # Publish map on every 10th clock tick (~10Hz) stamped with live sim time
        if self.map_msg is not None and self.clock_count % 10 == 0:
            self.map_msg.header.stamp.sec = msg.clock.sec
            self.map_msg.header.stamp.nanosec = msg.clock.nanosec
            self.static_map_pub.publish(self.map_msg)
            self.live_map_pub.publish(self.map_msg)

    def load_map_from_disk(self, yaml_path):
        if not os.path.exists(yaml_path):
            self.get_logger().warning(f'⚠️ Map file not found at {yaml_path}. Waiting for /map subscriber...')
            return

        try:
            with open(yaml_path, 'r') as f:
                map_yaml = yaml.safe_load(f)

            image_name = map_yaml['image']
            yaml_dir = os.path.dirname(yaml_path)
            pgm_path = os.path.join(yaml_dir, image_name)

            resolution = float(map_yaml['resolution'])
            origin = map_yaml['origin']

            img = Image.open(pgm_path)
            # Flip image vertically so row 0 aligns with ROS bottom-left origin
            raw = np.flipud(np.array(img, dtype=np.uint8))

            h, w = raw.shape
            self.width = w
            self.height = h
            self.resolution = resolution
            self.origin_x = float(origin[0])
            self.origin_y = float(origin[1])

            grid = np.zeros((h, w), dtype=np.int8)
            inflation = max(1, int(0.45 / resolution))

            # Step 1: Inflate walls only (black pixels <= 50)
            walls = (raw <= 50)
            wall_rows, wall_cols = np.where(walls)
            for r, c in zip(wall_rows, wall_cols):
                r_min = max(0, r - inflation)
                r_max = min(self.height, r + inflation + 1)
                c_min = max(0, c - inflation)
                c_max = min(self.width, c + inflation + 1)
                grid[r_min:r_max, c_min:c_max] = 1

            # Step 2: Mark unknown cells as impassable (gray pixels 50 < raw < 220)
            unknown = (raw > 50) & (raw < 220)
            grid[unknown] = 1

            self.map_grid = grid

            # Build ROS OccupancyGrid message for RViz2 display (0=free white, 100=walls black, -1=unknown gray)
            display_data = np.full((h, w), -1, dtype=np.int8)
            display_data[raw >= 220] = 0     # White floor (254) -> Free space (0)
            display_data[raw <= 50] = 100    # Black walls (0)   -> Wall (100)

            msg = OccupancyGrid()
            msg.header.frame_id = 'map'
            msg.info.resolution = resolution
            msg.info.width = w
            msg.info.height = h
            msg.info.origin.position.x = float(origin[0])
            msg.info.origin.position.y = float(origin[1])
            msg.info.origin.position.z = 0.0
            msg.data = display_data.flatten().tolist()
            self.map_msg = msg

            # Initial publish
            self.static_map_pub.publish(self.map_msg)

            n_free = int(np.sum(raw == 0))
            n_walls = int(np.sum(raw == 100))
            n_unknown = int(np.sum(raw < 0))
            n_grid_blocked = int(np.sum(grid == 1))
            n_grid_free = int(np.sum(grid == 0))
            self.get_logger().info(
                f'🗺️ Map loaded: {w}x{h} ({resolution}m/cell)\n'
                f'   Raw: Free={n_free}, Walls={n_walls}, Unknown={n_unknown}\n'
                f'   A* Grid: Navigable={n_grid_free}, Blocked={n_grid_blocked}'
            )

        except Exception as e:
            self.get_logger().error(f'❌ Failed to load map from disk: {e}')

    def scan_callback(self, msg):
        front_min = 999.0
        left_min = 999.0
        right_min = 999.0
        rear_min = 999.0
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment

        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r) or r < msg.range_min:
                continue
            angle = angle_min + i * angle_inc

            # Front cone (-35° to +35°)
            if -0.61 <= angle <= 0.61:
                if r < front_min:
                    front_min = r
            # Left sector (+35° to +110°)
            elif 0.61 < angle <= 1.92:
                if r < left_min:
                    left_min = r
            # Right sector (-110° to -35°)
            elif -1.92 <= angle < -0.61:
                if r < right_min:
                    right_min = r
            # Rear sector (|angle| > 2.35 rad, i.e., -180° to -135° and +135° to +180°)
            elif abs(angle) > 2.35:
                if r < rear_min:
                    rear_min = r

        self.front_obstacle_dist = front_min
        self.left_obstacle_dist = left_min
        self.right_obstacle_dist = right_min
        self.rear_obstacle_dist = rear_min

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def map_callback(self, msg):
        if self.map_grid is not None:
            return

        self.resolution = msg.info.resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        self.width = msg.info.width
        self.height = msg.info.height

        raw = np.array(msg.data, dtype=np.int8).reshape((self.height, self.width))
        inflation = max(1, int(0.20 / self.resolution))
        grid = np.zeros((self.height, self.width), dtype=np.uint8)
        occupied = (raw > 50) | (raw < 0)
        occ_rows, occ_cols = np.where(occupied)
        for r, c in zip(occ_rows, occ_cols):
            r_min = max(0, r - inflation)
            r_max = min(self.height, r + inflation + 1)
            c_min = max(0, c - inflation)
            c_max = min(self.width, c + inflation + 1)
            grid[r_min:r_max, c_min:c_max] = 1

        self.map_grid = grid

    def get_robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            return x, y, yaw
        except Exception:
            return self.robot_x, self.robot_y, self.robot_yaw

    def world_to_grid(self, x, y):
        col = int((x - self.origin_x) / self.resolution)
        row = int((y - self.origin_y) / self.resolution)
        return row, col

    def grid_to_world(self, row, col):
        x = self.origin_x + (col + 0.5) * self.resolution
        y = self.origin_y + (row + 0.5) * self.resolution
        return x, y

    def plan_path_astar(self, start_x, start_y, goal_x, goal_y):
        if self.map_grid is None:
            self.get_logger().error('❌ Map not loaded!')
            return []

        s_row, s_col = self.world_to_grid(start_x, start_y)
        g_row, g_col = self.world_to_grid(goal_x, goal_y)

        s_row = max(0, min(self.height - 1, s_row))
        s_col = max(0, min(self.width - 1, s_col))
        g_row = max(0, min(self.height - 1, g_row))
        g_col = max(0, min(self.width - 1, g_col))

        self.get_logger().info(f'🔎 Planning path: Start=({start_x:.2f}, {start_y:.2f}) -> Goal=({goal_x:.2f}, {goal_y:.2f})')

        grid = self.map_grid.copy()

        # Clear start and goal cell clearance area (radius 3)
        for dr in range(-3, 4):
            for dc in range(-3, 4):
                sr, sc = s_row+dr, s_col+dc
                gr, gc = g_row+dr, g_col+dc
                if 0 <= sr < self.height and 0 <= sc < self.width:
                    grid[sr, sc] = 0
                if 0 <= gr < self.height and 0 <= gc < self.width:
                    grid[gr, gc] = 0

        open_set = []
        heapq.heappush(open_set, (0.0, (s_row, s_col)))
        came_from = {}
        g_score = {(s_row, s_col): 0.0}

        def heuristic(r, c):
            return math.hypot(r - g_row, c - g_col)

        neighbors = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]

        found = False
        iterations = 0
        max_iterations = self.width * self.height

        while open_set and iterations < max_iterations:
            iterations += 1
            _, current = heapq.heappop(open_set)
            if current == (g_row, g_col):
                found = True
                break

            r, c = current
            for dr, dc in neighbors:
                nr, nc = r + dr, c + dc
                if 0 <= nr < self.height and 0 <= nc < self.width:
                    if grid[nr, nc] == 1:
                        continue
                    cost = math.hypot(dr, dc)
                    tentative = g_score[current] + cost
                    if (nr, nc) not in g_score or tentative < g_score[(nr, nc)]:
                        g_score[(nr, nc)] = tentative
                        heapq.heappush(open_set, (tentative + heuristic(nr, nc), (nr, nc)))
                        came_from[(nr, nc)] = current

        if not found:
            self.get_logger().error('❌ A* could not find a valid path! Goal may be inside a wall or unmapped area.')
            return []

        # Reconstruct path
        path_cells = []
        curr = (g_row, g_col)
        while curr in came_from:
            path_cells.append(curr)
            curr = came_from[curr]
        path_cells.reverse()

        # Downsample waypoints (every 4 cells = ~8cm steps)
        sub = path_cells[::4]
        if path_cells[-1] not in sub:
            sub.append(path_cells[-1])

        return [self.grid_to_world(r, c) for r, c in sub]

    def goal_callback(self, msg):
        elapsed = (self.get_clock().now() - self.startup_time).nanoseconds / 1e9
        if elapsed < self.startup_grace_seconds:
            self.get_logger().warning(f'⏳ Ignoring goal during startup ({elapsed:.1f}s < {self.startup_grace_seconds}s)')
            return

        self.target_x = msg.pose.position.x
        self.target_y = msg.pose.position.y

        rx, ry, _ = self.get_robot_pose()
        self.get_logger().info(f'🎯 Goal received: Target=({self.target_x:.2f}, {self.target_y:.2f}) | Robot=({rx:.2f}, {ry:.2f})')

        path = self.plan_path_astar(rx, ry, self.target_x, self.target_y)
        if path:
            self.path_points = path
            self.current_idx = 0
            self.is_navigating = True
            self.publish_path(self.path_points)
            self.get_logger().info(f'✅ Path planned around obstacles with {len(self.path_points)} waypoints!')
        else:
            self.is_navigating = False

    def publish_path(self, points):
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        if self.current_clock is not None:
            path_msg.header.stamp.sec = self.current_clock.sec
            path_msg.header.stamp.nanosec = self.current_clock.nanosec
        else:
            path_msg.header.stamp = self.get_clock().now().to_msg()

        for pt_x, pt_y in points:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = path_msg.header.stamp
            pose.pose.position.x = pt_x
            pose.pose.position.y = pt_y
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

    def send_cmd(self, cmd):
        self.cmd_vel_pub.publish(cmd)
        self.cmd_vel_model_pub.publish(cmd)

    def control_loop(self):
        if not self.is_navigating or not self.path_points:
            return

        rx, ry, ryaw = self.get_robot_pose()

        # Final target arrival check
        final_dist = math.hypot(self.target_x - rx, self.target_y - ry)
        if final_dist < 0.25:
            self.send_cmd(Twist())
            self.get_logger().info('🎉 Target reached successfully!')
            self.is_navigating = False
            return

        # Emergency stop only: halt if about to physically collide (< 20cm)
        if self.front_obstacle_dist < 0.20:
            self.send_cmd(Twist())
            self.get_logger().warning(f'🛑 Emergency stop! Obstacle at {self.front_obstacle_dist:.2f}m')
            return

        # Target waypoint
        wp_x, wp_y = self.path_points[self.current_idx]
        dx = wp_x - rx
        dy = wp_y - ry
        dist = math.hypot(dx, dy)

        # Monotonic forward advancement: advance to next waypoint if current waypoint is passed or close
        while self.current_idx < len(self.path_points) - 1:
            wp_x, wp_y = self.path_points[self.current_idx]
            next_x, next_y = self.path_points[self.current_idx + 1]
            dx = wp_x - rx
            dy = wp_y - ry
            dist = math.hypot(dx, dy)

            sv_x = next_x - wp_x
            sv_y = next_y - wp_y
            seg_len_sq = sv_x**2 + sv_y**2
            proj = ((rx - wp_x) * sv_x + (ry - wp_y) * sv_y) / (seg_len_sq + 1e-6)

            if dist < 0.30 or proj > 0.8:
                self.current_idx += 1
            else:
                break

        wp_x, wp_y = self.path_points[self.current_idx]
        dx = wp_x - rx
        dy = wp_y - ry
        dist = math.hypot(dx, dy)

        # Current path segment direction unit vector
        wp_x, wp_y = self.path_points[self.current_idx]
        if self.current_idx < len(self.path_points) - 1:
            next_x, next_y = self.path_points[self.current_idx + 1]
        else:
            next_x, next_y = wp_x, wp_y

        u_x = next_x - wp_x
        u_y = next_y - wp_y
        u_len = math.hypot(u_x, u_y)
        if u_len > 1e-4:
            u_x /= u_len
            u_y /= u_len
        else:
            u_x, u_y = math.cos(ryaw), math.sin(ryaw)

        # Right normal vector to the path direction
        n_rx = u_y
        n_ry = -u_x

        # Track relative obstacle approach speed dD/dt (m/s)
        if self.last_front_dist is not None:
            raw_rate = (self.front_obstacle_dist - self.last_front_dist) / 0.05
            self.obstacle_closing_speed = 0.7 * self.obstacle_closing_speed + 0.3 * raw_rate
        self.last_front_dist = self.front_obstacle_dist

        now_sec = self.get_clock().now().nanoseconds / 1e9

        # Target yaw to A* waypoint by default
        target_yaw = math.atan2(dy, dx)

        # DYNAMIC SOCIAL NAVIGATION (Bi-Directional Dual-Flank Evasion & Yield-on-Re-Merge)
        is_oncoming = (self.front_obstacle_dist < 1.80) and (self.obstacle_closing_speed < -0.08)

        if is_oncoming and not self.evading_dynamic_obstacle:
            self.evading_dynamic_obstacle = True
            self.evasion_start_time = now_sec
            self.evasion_end_time = now_sec + 2.2
            # Goal-Aware Intelligent Side Selection:
            # Check if target waypoint is to the LEFT or RIGHT of path direction
            # cross_prod > 0 -> Goal is on the LEFT, cross_prod <= 0 -> Goal is on the RIGHT
            cross_prod = u_x * dy - u_y * dx
            prefer_left = (cross_prod > 0.0) and (self.left_obstacle_dist > 0.45)
            prefer_right = (cross_prod <= 0.0) and (self.right_obstacle_dist > 0.45)

            if prefer_left:
                self.evasion_side = -1.0  # Shift left towards goal!
                side_str = "LEFT (towards goal)"
            elif prefer_right:
                self.evasion_side = 1.0   # Shift right towards goal!
                side_str = "RIGHT (towards goal)"
            elif self.right_obstacle_dist > 0.40:
                self.evasion_side = 1.0   # Fallback right
                side_str = "RIGHT (fallback)"
            else:
                self.evasion_side = -1.0  # Fallback left
                side_str = "LEFT (fallback)"

            self.get_logger().info(f'⚡ Goal-Aware Evasion triggered! Oncoming pedestrian at {self.front_obstacle_dist:.2f}m. Accelerating {side_str}...')

        # If in evasion mode
        if self.evading_dynamic_obstacle:
            time_in_evasion = now_sec - self.evasion_start_time

            # Flank clearance check based on evasion side
            if self.evasion_side > 0:
                is_flank_clear = (self.left_obstacle_dist >= 1.00)
            else:
                is_flank_clear = (self.right_obstacle_dist >= 1.00)

            is_timer_expired = (now_sec >= self.evasion_end_time)
            is_wall_danger = (self.front_obstacle_dist < 0.50) or (self.right_obstacle_dist < 0.30) or (self.left_obstacle_dist < 0.30)

            if (time_in_evasion >= 0.8 and is_timer_expired and is_flank_clear) or is_wall_danger:
                self.evading_dynamic_obstacle = False
                reason = "Wall hazard" if is_wall_danger else "Flank clear"
                self.get_logger().info(f'✅ Evasion completed ({reason})! Merging back onto A* path at waypoint {self.current_idx + 1}/{len(self.path_points)}')
            else:
                # Calculate safe lateral offset bounded by wall distance
                if self.evasion_side > 0:
                    safe_offset = min(0.35, max(0.15, self.right_obstacle_dist - 0.40))
                    evade_x = rx + 0.80 * u_x + safe_offset * n_rx
                    evade_y = ry + 0.80 * u_y + safe_offset * n_ry
                else:
                    safe_offset = min(0.35, max(0.15, self.left_obstacle_dist - 0.40))
                    evade_x = rx + 0.80 * u_x - safe_offset * n_rx
                    evade_y = ry + 0.80 * u_y - safe_offset * n_ry
                target_yaw = math.atan2(evade_y - ry, evade_x - rx)

        yaw_err = math.atan2(math.sin(target_yaw - ryaw), math.cos(target_yaw - ryaw))

        # Track relative rear & flank obstacle approach speeds (m/s)
        if self.last_rear_dist is not None:
            raw_rear_rate = (self.rear_obstacle_dist - self.last_rear_dist) / 0.05
            self.rear_closing_speed = 0.7 * self.rear_closing_speed + 0.3 * raw_rear_rate
        self.last_rear_dist = self.rear_obstacle_dist

        if self.last_left_dist is not None:
            raw_left_rate = (self.left_obstacle_dist - self.last_left_dist) / 0.05
            self.left_closing_speed = 0.7 * self.left_closing_speed + 0.3 * raw_left_rate
        self.last_left_dist = self.left_obstacle_dist

        if self.last_right_dist is not None:
            raw_right_rate = (self.right_obstacle_dist - self.last_right_dist) / 0.05
            self.right_closing_speed = 0.7 * self.right_closing_speed + 0.3 * raw_right_rate
        self.last_right_dist = self.right_obstacle_dist

        # REAR THREAT ESCAPE SYSTEM (Trigger ONLY if pedestrian is actively closing in from behind < 0.75m)
        is_rear_threat = (self.rear_obstacle_dist < 0.75) and (self.rear_closing_speed < -0.08)

        # PERPENDICULAR FLANK THREAT ESCAPE (Trigger if pedestrian approaches from left/right < 0.70m and is closing in)
        is_flank_threat = ((self.left_obstacle_dist < 0.70 and self.left_closing_speed < -0.08) or
                           (self.right_obstacle_dist < 0.70 and self.right_closing_speed < -0.08))

        # RE-MERGING YIELD CHECK: If pedestrian is standing/walking in the re-merge path ahead (< 0.85m), STOP & YIELD
        is_remerge_blocked = (not self.evading_dynamic_obstacle) and (self.front_obstacle_dist < 0.85 or self.left_obstacle_dist < 0.65) and (dist > 0.40)

        cmd = Twist()
        if self.evading_dynamic_obstacle:
            # Drive forward in safe evasive lane at 0.35 m/s with responsive steering
            cmd.linear.x = 0.35
            cmd.angular.z = max(min(1.8 * yaw_err, 1.2), -1.2)
        elif (is_rear_threat or is_flank_threat) and self.front_obstacle_dist > 0.60:
            # Accelerate forward to 0.42 m/s to clear perpendicular intersection or escape rear pursuer
            threat_type = "Perpendicular Flank" if is_flank_threat else "Rear"
            self.get_logger().info(f'⚡ {threat_type} threat detected! Accelerating forward (0.42 m/s) to clear collision zone...')
            cmd.linear.x = 0.42
            cmd.angular.z = max(min(1.0 * yaw_err, 0.5), -0.5)
        elif is_remerge_blocked:
            # Stop and yield right-of-way cleanly until pedestrian clears the re-merge corridor (Zero spin loop!)
            self.get_logger().warning(f'🛑 Pedestrian blocking re-merge corridor ({self.front_obstacle_dist:.2f}m). Stopping & yielding...')
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        else:
            if abs(yaw_err) > 0.30:
                cmd.linear.x = 0.0
                cmd.angular.z = max(min(1.2 * yaw_err, 0.6), -0.6)
            else:
                cmd.linear.x = max(min(0.30 * dist, 0.25), 0.08)
                cmd.angular.z = max(min(1.0 * yaw_err, 0.5), -0.5)

        self.send_cmd(cmd)

def main():
    rclpy.init()
    
    if len(sys.argv) >= 2 and not sys.argv[1].startswith('--'):
        map_path = sys.argv[1]
    else:
        map_path = os.path.expanduser(
            '~/Documents/robotics-portfolio/ros2-slam-nav2/src/slam_robot/maps/warehouse_map.yaml'
        )

    node = UnifiedNavigatorNode(map_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
