#!/usr/bin/env python3
"""
Static Map Server: loads a saved PGM+YAML map and publishes it on /static_map.
Subscribes to /clock to stamp messages with sim time for RViz2 compatibility.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from rosgraph_msgs.msg import Clock
import numpy as np
import yaml
import os
import sys


class StaticMapServer(Node):
    def __init__(self, yaml_path):
        super().__init__('static_map_server')

        # Track sim time from /clock
        self.sim_time_sec = 0
        self.sim_time_nanosec = 0
        self.clock_received = False
        self.create_subscription(Clock, '/clock', self.clock_callback, 10)

        # Load YAML metadata
        with open(yaml_path, 'r') as f:
            meta = yaml.safe_load(f)

        map_dir = os.path.dirname(os.path.abspath(yaml_path))
        pgm_path = os.path.join(map_dir, meta['image'])
        resolution = float(meta['resolution'])
        origin = meta['origin']
        free_thresh = float(meta.get('free_thresh', 0.25))
        occupied_thresh = float(meta.get('occupied_thresh', 0.65))
        negate = int(meta.get('negate', 0))

        # Load PGM image
        with open(pgm_path, 'rb') as f:
            magic = f.readline().strip()
            assert magic == b'P5', f'Expected P5 PGM, got {magic}'
            line = f.readline()
            while line.startswith(b'#'):
                line = f.readline()
            width, height = map(int, line.split())
            max_val = int(f.readline().strip())
            pixels = np.frombuffer(f.read(), dtype=np.uint8).reshape((height, width))

        pixels = np.flipud(pixels)

        if negate:
            normalized = pixels.astype(float) / max_val
        else:
            normalized = (max_val - pixels.astype(float)) / max_val

        occupancy_data = np.full((height, width), -1, dtype=np.int8)
        occupancy_data[normalized <= free_thresh] = 0
        occupancy_data[normalized >= occupied_thresh] = 100

        # Build base message
        self.map_msg = OccupancyGrid()
        self.map_msg.header.frame_id = 'map'
        self.map_msg.info.resolution = resolution
        self.map_msg.info.width = width
        self.map_msg.info.height = height
        self.map_msg.info.origin.position.x = float(origin[0])
        self.map_msg.info.origin.position.y = float(origin[1])
        self.map_msg.info.origin.position.z = 0.0
        self.map_msg.data = occupancy_data.flatten().tolist()

        # Publisher with Transient Local QoS
        map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        self.map_pub = self.create_publisher(OccupancyGrid, '/static_map', map_qos)

        # Publish every 1s
        self.create_timer(1.0, self.publish_map)

        n_free = int(np.sum(occupancy_data == 0))
        n_occ = int(np.sum(occupancy_data == 100))
        self.get_logger().info(
            f'🗺️ Map loaded from {pgm_path}\n'
            f'   Size: {width}x{height}, Resolution: {resolution}m\n'
            f'   Free: {n_free}, Occupied: {n_occ}\n'
            f'   Waiting for /clock to start publishing...'
        )

    def clock_callback(self, msg):
        self.sim_time_sec = msg.clock.sec
        self.sim_time_nanosec = msg.clock.nanosec
        if not self.clock_received:
            self.clock_received = True
            self.get_logger().info(f'⏰ Clock received! Publishing map with sim timestamps.')
            self.publish_map()

    def publish_map(self):
        if not self.clock_received:
            return
        self.map_msg.header.stamp.sec = self.sim_time_sec
        self.map_msg.header.stamp.nanosec = self.sim_time_nanosec
        self.map_pub.publish(self.map_msg)


def main():
    rclpy.init()

    if len(sys.argv) >= 2:
        yaml_path = sys.argv[1]
    else:
        yaml_path = os.path.expanduser(
            '~/Documents/robotics-portfolio/ros2-slam-nav2/src/slam_robot/maps/warehouse_map.yaml'
        )

    if not os.path.exists(yaml_path):
        print(f'❌ Map file not found: {yaml_path}')
        sys.exit(1)

    node = StaticMapServer(yaml_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
