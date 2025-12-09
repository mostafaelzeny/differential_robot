# Autonomous Differential Drive Robot

A comprehensive ROS 2 robotics project implementing SLAM, localization, and path planning for a differential drive robot in simulated environments using Gazebo.

## Overview

This project demonstrates a complete autonomous navigation stack for a differential drive robot, featuring simultaneous localization and mapping (SLAM), adaptive Monte Carlo localization (AMCL), and multiple path planning algorithms. The robot operates in realistic residential and warehouse environments using AWS RoboMaker worlds.

## Features

- **SLAM Implementation**: Real-time mapping using SLAM Toolbox
- **Localization**: AMCL-based global localization for accurate pose estimation
- **Path Planning**: Multiple algorithms including A*, Dijkstra, and PD motion control
- **Simulation**: High-fidelity Gazebo simulation with detailed environment models
- **ROS 2 Integration**: Full compatibility with ROS 2 ecosystem

## Project Structure

```
differential_robot/
├── robot_controller/          # Robot control and sensor interfaces
├── robot_description/         # URDF models and Gazebo worlds
│   ├── models/               # AWS RoboMaker residential & warehouse models
│   ├── urdf/                 # Robot URDF and Xacro files
│   └── worlds/               # Simulation environments
├── robot_localization/        # AMCL configuration and launch files
├── robot_mapping/             # Costmap configuration and saved maps
├── robot_path_planning/       # Path planning algorithm implementations
└── robot_slam/               # SLAM Toolbox integration
```

## Prerequisites

- **ROS 2** (Humble/Iron recommended)
- **Gazebo** (Classic or Ignition)
- **Navigation2** stack
- **SLAM Toolbox**
- **AWS RoboMaker assets** (included in repository)

### Installation

```bash
# Install ROS 2 dependencies
sudo apt update
sudo apt install ros-$ROS_DISTRO-navigation2 \
                 ros-$ROS_DISTRO-slam-toolbox \
                 ros-$ROS_DISTRO-gazebo-ros-pkgs

# Clone the repository
cd ~/ros2_ws/src
git clone git@github.com:mostafaelzeny/differential_robot_ws.git

# Build the workspace
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Usage

### 1. Launch SLAM and Mapping

Start the robot in Gazebo and begin mapping:

```bash
ros2 launch robot_slam slam.launch.py
```

Navigate the robot using teleop to build the map:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 2. Save the Map

Once mapping is complete, save your map:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

### 3. Launch Localization

Use the saved map for localization:

```bash
ros2 launch robot_localization global_localization.launch.py
```

### 4. Autonomous Navigation

Run path planning with the Navigation2 stack:

```bash
ros2 launch nav2_bringup navigation_launch.py
```

Set navigation goals via RViz or programmatically.

## Path Planning Algorithms

The project includes three path planning implementations:

- **A* Planner** (`a_star_planner.cpp`): Heuristic-based optimal pathfinding
- **Dijkstra Planner** (`dijkstra_planner.cpp`): Guaranteed shortest path algorithm
- **PD Motion Planner** (`pd_motion_planner.cpp`): Proportional-derivative controller for smooth motion

## Simulation Environments

Two detailed environments are available:

- **Small House** (`small_house.world`): Residential environment with furniture and realistic home layouts
- **Small Warehouse** (`small_warehouse.world`): Industrial setting with shelves, pallets, and warehouse equipment

## Configuration Files

- **AMCL**: `robot_localization/config/amcl.yaml`
- **SLAM Toolbox**: `robot_slam/config/slam_toolbox.yaml`
- **Costmap**: `robot_mapping/config/costmap.yaml`

## Robot Description

The robot is defined using URDF/Xacro files with:
- Differential drive kinematics
- Lidar sensor for mapping
- ROS 2 Control integration
- Gazebo physics simulation

## Troubleshooting

**Issue**: Robot not moving in simulation
- Check that `ros2_control` is properly loaded
- Verify wheel joint names match URDF configuration

**Issue**: SLAM not producing good maps
- Reduce robot speed during mapping
- Adjust SLAM Toolbox parameters in config file
- Ensure lidar data is being published

**Issue**: Localization accuracy poor
- Increase particle count in AMCL config
- Verify saved map quality
- Check initial pose estimate

## Contributing

Contributions are welcome! Please feel free to submit pull requests or open issues for bugs and feature requests.

## License

[Specify your license here - MIT, Apache 2.0, GPL, etc.]

## Acknowledgments

- AWS RoboMaker for simulation assets
- SLAM Toolbox developers
- ROS 2 and Navigation2 communities

## Contact

Moustafa Elzeiny
elzenymostafa8@gmail.com

---

**Note**: This project is intended for educational and research purposes in mobile robotics and autonomous navigation.