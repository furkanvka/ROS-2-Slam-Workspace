# ROS 2 Mapping Workspace

A ROS 2 Jazzy workspace for robot mapping using occupancy grid mapping with Clearpath simulation.

## Overview

This workspace contains a mapping package that processes sensor data to build an occupancy grid map. It integrates with Clearpath's Gazebo simulation environment.

## Package Structure

```
src/
└── mapping/
    ├── CMakeLists.txt          # Build configuration
    ├── package.xml             # Package manifest
    ├── include/                # Header files
    └── src/
        └── Occupancy.cpp       # Occupancy grid mapping node
```

## Dependencies

- ROS 2 Jazzy
- Clearpath packages
- Gazebo (for simulation)
- colcon (build tool)

### ROS 2 Packages

- `rclcpp` - ROS 2 C++ client library
- `geometry_msgs` - Geometric message types
- `nav_msgs` - Navigation message types (OccupancyGrid, etc.)
- `sensor_msgs` - Sensor message types (LaserScan, etc.)
- `clearpath_gz` - Clearpath Gazebo simulation

## Building

```bash
# Source ROS 2
source /opt/ros/jazzy/setup.bash

# Build the workspace
colcon build --symlink-install
```

## Running

Use the provided script for easy launch:

```bash
./run.sh
```

Or manually:

```bash
# Source ROS 2 and workspace
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Launch simulation
ros2 launch clearpath_gz simulation.launch.py \
  x:=-3.3 y:=3.3 \
  world:=/home/furkan/Documents/new/worlds/generated_maze \
  rviz:=true \
  use_sim_time:=true

# Run mapping node (in another terminal)
ros2 run mapping Occupancy
```

## Nodes

### Occupancy

Processes sensor data to generate an occupancy grid map.

**Subscriptions:**
- Laser scan data (for obstacle detection)
- Odometry (for robot pose)

**Publications:**
- Occupancy grid map (`/map` topic)

## Simulation

The workspace includes integration with Clearpath's Gazebo simulation:
- Simulated robot with LiDAR sensors
- Generated maze world environment
- RViz visualization support

## Development

### Linting

The package includes ament lint tools:
- `cppcheck` - Static code analysis
- `lint_cmake` - CMake linting
- `uncrustify` - Code formatting
- `xmllint` - XML validation

To run linters:
```bash
colcon test --packages-select mapping
```

## License

[Add your license information here]

## Author

[Add author information here]
