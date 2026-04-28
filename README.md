# ROS 2 Autonomous Mapping & Exploration System

A ROS 2 Jazzy workspace for autonomous robot mapping, localization, and exploration using Clearpath A200 simulation in Gazebo.

## Overview

This project implements a complete autonomous navigation stack including:
- **ICP-based Localization**: Corrects odometry drift using point-to-line ICP with KD-Tree
- **Occupancy Grid Mapping**: Real-time map building with log-odds update
- **Frontier-based Exploration**: Autonomous exploration of unknown environments
- **A* Path Planning**: Optimal pathfinding with wall penalty zones
- **Pure Pursuit Path Following**: Smooth path tracking with obstacle avoidance

## Features

- Real-time SLAM using 2D LiDAR
- Procedurally generated maze environments
- Stuck detection and recovery maneuvers
- Adaptive velocity profiling
- Wall-aware path planning with penalty zones
- Complete simulation integration with Clearpath Gazebo

## Tech Stack

| Category | Technology |
|----------|------------|
| Framework | ROS 2 Jazzy |
| Languages | C++17, Python 3 |
| Simulation | Gazebo (gz-sim) |
| Build | CMake, colcon |
| Robot | Clearpath A200 |

## Dependencies

### System Requirements
- ROS 2 Jazzy
- Gazebo gz-sim
- colcon build tool

### ROS 2 Packages
- `rclcpp`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `std_srvs`
- `clearpath_gz` (Clearpath simulation)

## Installation

```bash
# Clone the repository
cd ~/ros2_ws/src
git clone <repository-url> map

# Source ROS 2
source /opt/ros/jazzy/setup.bash
source ~/clearpath/setup.bash

# Build
cd ~/ros2_ws
colcon build --symlink-install
```

## Usage

### Quick Start
```bash
./run.sh
```

### Manual Launch
```bash
source /opt/ros/jazzy/setup.bash
source ~/clearpath/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch mapping launch.py
```

### Generate Custom Maze
```bash
python a.py --cells-x 6  # Generates a 6x6 maze
```

## Nodes

### localization_node
- **Purpose**: ICP-based pose correction
- **Input**: `/a200_0000/sensors/lidar2d_0/scan`, `/a200_0000/platform/odom/filtered`
- **Output**: `/corrected_pose`

### mapping_node
- **Purpose**: Occupancy grid map building
- **Input**: `/a200_0000/sensors/lidar2d_0/scan`, `/corrected_pose`
- **Output**: `/map`

### frontier_node
- **Purpose**: Autonomous exploration and path planning
- **Input**: `/map`, `/corrected_pose`
- **Output**: `/plan`, `/frontier_costmap`

### path_node
- **Purpose**: Path following with obstacle avoidance
- **Input**: `/plan`, `/corrected_pose`, `/a200_0000/sensors/lidar2d_0/scan`
- **Output**: `/a200_0000/cmd_vel`

## Project Structure

```
map/
├── src/mapping/           # Main package
│   ├── launch/launch.py   # System launch file
│   └── src/               # Node implementations
├── worlds/                # Gazebo world files
├── a.py                   # Maze generator
├── run.sh                 # Build & launch script
└── legacy/                # Previous implementation
```

## Testing

```bash
colcon test --packages-select mapping
```

## License

Apache-2.0

