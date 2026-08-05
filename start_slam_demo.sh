#!/bin/bash
# ==============================================================================
# Autonomous SLAM & Navigation Demo Launcher
# Automatically builds, launches Gazebo + RViz2, SLAM Toolbox, and Teleop
# ==============================================================================

set -e

# Project root directory
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$PROJECT_DIR"

# Clean Snap environment variables if running inside a Snap IDE
unset SNAP SNAP_NAME SNAP_REVISION SNAP_ARCH SNAP_LIBRARY_PATH SNAP_USER_DATA SNAP_USER_COMMON SNAP_DATA SNAP_COMMON
export LD_LIBRARY_PATH=/opt/ros/lyrical/lib
export PATH=$(echo $PATH | tr ":" "\n" | grep -v "/snap" | paste -sd:)
export XDG_DATA_DIRS=/usr/share/ubuntu:/usr/share/gnome:/usr/local/share/:/usr/share/

# Source ROS 2 Lyrical
source /opt/ros/lyrical/setup.bash

echo "🔨 1/4 - Compiling ROS 2 workspace with colcon..."
colcon build --symlink-install

source "$PROJECT_DIR/install/setup.bash"

# Cleanup function to kill all background processes when stopping (Ctrl+C)
cleanup() {
    echo ""
    echo "🛑 Stopping all simulation & SLAM processes..."
    kill 0 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "🚀 2/4 - Launching Gazebo & RViz2 simulation (Static Warehouse for SLAM)..."
ros2 launch slam_robot sim.launch.py world_type:=static &
sleep 5

echo "🗺️ 3/4 - Starting SLAM Toolbox & Activating Lifecycle Node..."
ros2 launch slam_robot slam.launch.py &

# Wait until slam_toolbox service is listed in ROS graph
until ros2 service list 2>/dev/null | grep -q "/slam_toolbox/change_state"; do
    sleep 0.5
done

echo "⚙️ Configuring SLAM Toolbox..."
ros2 lifecycle set /slam_toolbox configure
sleep 1
echo "⚡ Activating SLAM Toolbox..."
ros2 lifecycle set /slam_toolbox activate
sleep 2

echo "🎮 4/4 - Starting Keyboard Teleoperation!"
echo "=========================================================="
echo " Use keys to drive your robot in Gazebo & build the map:"
echo "   i : Forward"
echo "   k : Stop"
echo "   j : Turn Left"
echo "   l : Turn Right"
echo "   , : Backward"
echo " Press Ctrl+C to stop all processes when finished."
echo "=========================================================="

ros2 run teleop_twist_keyboard teleop_twist_keyboard
