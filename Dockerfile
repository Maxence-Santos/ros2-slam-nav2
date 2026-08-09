# Build & test image for ros2-slam-nav2.
# Full Gazebo+GUI demos are intended to run on the host (see README).
# This image validates package build and pure-Python unit tests in CI/CD.
FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    python3-numpy \
    python3-pil \
    python3-yaml \
    ros-jazzy-slam-toolbox \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-xacro \
    ros-jazzy-tf2-ros \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY src/slam_robot /ws/src/slam_robot
COPY tests /ws/tests
COPY save_map.py /ws/save_map.py

RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build --packages-select slam_robot --symlink-install"

# Default: unit tests (no GPU / no display required)
CMD ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && python3 /ws/tests/test_navigation_geometry.py -v"]
