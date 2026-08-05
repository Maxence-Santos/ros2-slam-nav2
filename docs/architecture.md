# System Architecture

This document describes the software architecture and node graph of the `slam_robot` ROS 2 system.

## High-Level Architecture

The system is composed of several key functional blocks:
1. **Simulation (Gazebo)**: Simulates physics, collisions, and generates sensor data.
2. **Robot Base (ros2_control)**: Interfaces with the simulated hardware to drive the wheels and read encoders.
3. **State Estimation (robot_localization)**: Fuses Odom and IMU for accurate localization.
4. **Perception & Mapping (SLAM Toolbox)**: Uses LiDAR data to map the environment and localize against it.
5. **Navigation (Nav2)**: Plans global and local paths, avoids obstacles, and sends velocity commands to the base.

## ROS 2 Node and Topic Graph

```mermaid
graph TD
    %% Define styles
    classDef node fill:#e1f5fe,stroke:#01579b,stroke-width:2px;
    classDef topic fill:#fff3e0,stroke:#e65100,stroke-width:2px,stroke-dasharray: 5 5;
    classDef tf fill:#e8f5e9,stroke:#1b5e20,stroke-width:2px,stroke-dasharray: 5 5;

    %% Nodes
    Gazebo[("Gazebo Sim\n(gazebo_ros)")]:::node
    ControllerServer["Nav2 Controller\n(dwb_core)"]:::node
    PlannerServer["Nav2 Planner\n(navfn/smac)"]:::node
    BTNavigator["Nav2 BT Navigator"]:::node
    CostmapGlobal["Nav2 Global Costmap"]:::node
    CostmapLocal["Nav2 Local Costmap"]:::node
    SLAMToolbox["SLAM Toolbox"]:::node
    EKF["EKF Node\n(robot_localization)"]:::node
    RSP["Robot State Publisher"]:::node
    DiffDrive["ros2_control\nDiff Drive Controller"]:::node
    JointState["Joint State Broadcaster"]:::node
    WaypointFollower["Waypoint Follower Node"]:::node

    %% Topics & Data Flow
    Gazebo -->|/scan| SLAMToolbox
    Gazebo -->|/scan| CostmapLocal
    Gazebo -->|/scan| CostmapGlobal
    Gazebo -->|/imu/data| EKF
    
    DiffDrive -->|/odom| EKF
    ControllerServer -->|/cmd_vel| DiffDrive
    DiffDrive -.->|hardware_interface| Gazebo
    JointState -.->|hardware_interface| Gazebo
    
    JointState -->|/joint_states| RSP
    
    SLAMToolbox -->|/map| CostmapGlobal
    SLAMToolbox -->|/map| CostmapLocal
    
    WaypointFollower -->|Action: NavigateToPose| BTNavigator
    BTNavigator -->|Action: ComputePathToPose| PlannerServer
    BTNavigator -->|Action: FollowPath| ControllerServer

    %% TF Tree
    subgraph "TF Tree (/tf & /tf_static)"
        tf_map((map)):::tf
        tf_odom((odom)):::tf
        tf_base((base_link)):::tf
        tf_sensors((sensors links)):::tf
    end

    SLAMToolbox -.->|publishes| tf_map
    tf_map -.->|transforms to| tf_odom
    
    EKF -.->|publishes| tf_odom
    tf_odom -.->|transforms to| tf_base
    
    RSP -.->|publishes| tf_base
    tf_base -.->|transforms to| tf_sensors
```

## Description of Key Components

- **EKF Node**: Subscribes to wheel odometry (from the `diff_drive_controller`) and IMU data (from Gazebo). It outputs a filtered odometry message and broadcasts the `odom -> base_link` transform.
- **SLAM Toolbox**: Subscribes to `/scan` (LiDAR) and uses the `odom -> base_link` transform to perform scan-matching. It builds an occupancy grid (`/map`) and broadcasts the drift-correction transform `map -> odom`.
- **Nav2 Stack**:
  - Uses the static `/map` for the Global Costmap.
  - Uses live `/scan` data to populate the Local Costmap with dynamic obstacles.
  - The `Planner Server` computes a high-level path from A to B.
  - The `Controller Server` generates velocity commands (`/cmd_vel`) to follow the path locally while avoiding sudden obstacles.
