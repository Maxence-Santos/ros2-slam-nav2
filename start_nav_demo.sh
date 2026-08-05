#!/usr/bin/env bash
# ==============================================================================
# ROS 2 Autonomous Navigation Launcher (Phase 3)
# Launches Gazebo, SLAM Toolbox (for dynamic localization), RViz2, and Unified Map/A* Navigator
# ==============================================================================

# Purge snap LD_LIBRARY_PATH pollution for clean RViz2/Gazebo launch
export LD_LIBRARY_PATH=$(echo $LD_LIBRARY_PATH | tr ":" "\n" | grep -v "snap" | paste -sd:)
export PATH=$(echo $PATH | tr ":" "\n" | grep -v "snap" | paste -sd:)

# Force OpenGL 3.3 Core Profile for clean RViz2 rendering
export MESA_GL_VERSION_OVERRIDE=3.3
export MESA_GLSL_VERSION_OVERRIDE=330

# Source ROS 2 environment
source /opt/ros/lyrical/setup.bash

# Get current script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
cd "$SCRIPT_DIR"

# Source workspace install
if [ -f "$SCRIPT_DIR/install/setup.bash" ]; then
    source "$SCRIPT_DIR/install/setup.bash"
else
    echo "🔨 Workspace not built yet. Building now..."
    colcon build --packages-select slam_robot
    source "$SCRIPT_DIR/install/setup.bash"
fi

MAP_YAML="$SCRIPT_DIR/src/slam_robot/maps/warehouse_map.yaml"

if [ ! -f "$MAP_YAML" ]; then
    echo "❌ No saved map found at $MAP_YAML"
    echo "   Run start_slam_demo.sh first, explore the environment, then save with:"
    echo "   python3 save_map.py"
    exit 1
fi

echo "=========================================================="
echo "🤖 Starting Phase 3: Autonomous Navigation (Dynamic Localization)"
echo "   Loaded Map: $MAP_YAML"
echo "=========================================================="

# Cleanup previous background processes on exit
cleanup() {
    echo ""
    echo "🛑 Shutting down Autonomous Navigation nodes..."
    kill $(jobs -p) 2>/dev/null || true
    pkill -f "gz sim" 2>/dev/null || true
    pkill -f "rviz2" 2>/dev/null || true
    pkill -f "slam_toolbox" 2>/dev/null || true
    pkill -f "nav_controller" 2>/dev/null || true
    echo "✨ Cleanup complete!"
    exit 0
}

trap cleanup SIGINT SIGTERM EXIT

echo "🧹 Purging lingering background processes..."
pkill -9 -f "gz sim" 2>/dev/null || true
pkill -9 -f "ruby" 2>/dev/null || true
pkill -9 -f "ros_gz" 2>/dev/null || true
pkill -9 -f "nav_controller" 2>/dev/null || true
pkill -9 -f "rviz2" 2>/dev/null || true
sleep 1

echo "🚀 1/3 - Launching Gazebo & RViz2 simulation..."
ros2 launch slam_robot sim.launch.py &
sleep 5

echo "🔗 2/3 - Publishing static transform (map -> odom)..."
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --yaw 0 --pitch 0 --roll 0 --frame-id map --child-frame-id odom --ros-args -p use_sim_time:=true &
sleep 1

echo "🎯 3/3 - Starting Autonomous Navigator & Map Server..."
ros2 run slam_robot nav_controller "$MAP_YAML" --ros-args -p use_sim_time:=true &
sleep 2

echo "=========================================================="
echo "🎉 Autonomous Navigation Ready!"
echo "👉 Instructions:"
echo " 1. Go to RViz2 window."
echo " 2. Click the '2D Goal Pose' button in the top toolbar."
echo " 3. Click anywhere on the WHITE areas of the saved map to navigate!"
echo " 4. The robot will plan around walls and drive to the target!"
echo "=========================================================="

wait
