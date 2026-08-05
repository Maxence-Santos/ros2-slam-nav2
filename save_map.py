#!/usr/bin/env python3
"""
Custom ROS 2 Map Saver.
Calls slam_toolbox native service via subprocess or saves directly from /map topic,
requiring zero external Python ROS 2 service imports.
"""
import subprocess
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import os
import sys

def main():
    if len(sys.argv) < 2:
        prefix = os.path.expanduser('~/Documents/robotics-portfolio/ros2-slam-nav2/src/slam_robot/maps/warehouse_map')
    else:
        prefix = sys.argv[1]

    output_dir = os.path.dirname(prefix)
    if output_dir and not os.path.exists(output_dir):
        os.makedirs(output_dir, exist_ok=True)

    print(f"🗺️ Attempting map export to {prefix}.yaml/.pgm...")

    # Try calling slam_toolbox native map saving service via CLI (no python module import needed)
    cmd = [
        "ros2", "service", "call",
        "/slam_toolbox/save_map",
        "slam_toolbox/srv/SaveMap",
        f"{{name: {{data: '{prefix}'}}}}"
    ]

    try:
        res = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
        if res.returncode == 0 and "result=True" in res.stdout:
            print(f"✅ Map saved successfully via slam_toolbox native service!\n  - Location: {prefix}.yaml/.pgm")
            return
        elif res.returncode == 0 and "result=" in res.stdout:
            print(f"✅ Map saved via slam_toolbox native service (Output: {res.stdout.strip()})")
            return
    except Exception as e:
        print(f"ℹ️ Native service call skipped ({e}). Falling back to /map topic subscriber...")

    # Fallback: Subscribe to /map topic directly using rclpy
    rclpy.init()
    node = Node('map_saver_client')

    received_map = [None]
    def map_cb(msg):
        received_map[0] = msg

    sub = node.create_subscription(OccupancyGrid, '/map', map_cb, 10)
    print("Waiting for /map topic message...")
    while rclpy.ok() and received_map[0] is None:
        rclpy.spin_once(node, timeout_sec=0.2)

    if received_map[0] is None:
        print("❌ Failed to receive /map topic message.")
        node.destroy_node()
        rclpy.shutdown()
        return

    msg = received_map[0]
    width = msg.info.width
    height = msg.info.height
    resolution = msg.info.resolution
    origin_x = msg.info.origin.position.x
    origin_y = msg.info.origin.position.y

    raw_data = np.array(msg.data, dtype=np.int8).reshape((height, width))
    img_data = np.full((height, width), 205, dtype=np.uint8)
    img_data[raw_data == 0] = 254
    img_data[raw_data == 100] = 0

    img_data = np.flipud(img_data)

    pgm_path = f"{prefix}.pgm"
    yaml_path = f"{prefix}.yaml"

    with open(pgm_path, 'wb') as f:
        header = f"P5\n# CREATED BY Custom ROS 2 Map Saver\n{width} {height}\n255\n".encode('ascii')
        f.write(header)
        f.write(img_data.tobytes())

    pgm_filename = os.path.basename(pgm_path)
    yaml_content = f"""image: {pgm_filename}
mode: trinary
resolution: {resolution}
origin: [{origin_x}, {origin_y}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
"""
    with open(yaml_path, 'w') as f:
        f.write(yaml_content)

    print(f"✅ Map saved successfully from /map topic!\n  - Image: {pgm_path}\n  - Metadata: {yaml_path}")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
