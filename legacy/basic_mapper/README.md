# Wallfollower - Maze Explorer

ROS 2 package for autonomous maze exploration and navigation using 2D LiDAR and odometry.

## Overview

This package implements a wall-following maze exploration algorithm with three main components:
- **maze_decision_node**: Planning and exploration logic using DFS for discovery and BFS for pathfinding
- **maze_motion_node**: Low-level motion control using PID controllers for rotation and wall alignment
- **goal_bridge_node**: Bridge between RViz clicked points and grid coordinates

## Nodes

### maze_decision_node

Handles maze exploration and path planning:
- Uses **DFS (Depth-First Search)** for autonomous exploration
- Uses **BFS (Breadth-First Search)** for path planning to user-defined goals
- Maps walls and visited cells in a grid representation
- Publishes visualization markers for RViz

**Subscriptions:**
- `/a200_0000/sensors/lidar2d_0/scan` - LiDAR sensor data
- `/a200_0000/platform/odom/filtered` - Odometry
- `/maze_motion_status` - Motion completion status
- `/maze_goal` - User-defined goal point

**Publications:**
- `/maze_motion_cmd` - Motion commands (target grid + yaw)
- `/maze_viz` - RViz visualization markers

### maze_motion_node

Handles robot movement execution:
- **PID-controlled rotation** for precise heading alignment
- **PID-controlled forward motion** with wall alignment
- **LiDAR-Odometry fusion** for accurate distance estimation
- Emergency stop on obstacle detection

**Subscriptions:**
- `/maze_motion_cmd` - Motion commands from decision node
- `/a200_0000/sensors/lidar2d_0/scan` - LiDAR for obstacle detection
- `/a200_0000/platform/odom/filtered` - Odometry

**Publications:**
- `/a200_0000/cmd_vel` - Velocity commands
- `/maze_motion_status` - Motion status ("DONE" or "FAILED")

### goal_bridge_node

Converts RViz clicked points to grid goals:
- Snaps clicked world coordinates to grid cells
- Publishes goal for decision node
- Shows preview marker in RViz

**Subscriptions:**
- `/clicked_point` - RViz point selection

**Publications:**
- `/maze_goal` - Grid goal coordinates
- `/maze_viz_goal_preview` - Visual preview marker

## Configuration

Key parameters (in source files):
- `CELL_SIZE = 2.05` - Grid cell size in meters
- `WALL_THRESHOLD = 1.6` - Distance threshold for wall detection
- `ANGLE_TOLERANCE = 0.5` - Rotation tolerance in degrees
- `MOVE_TOLERANCE = 0.01` - Position tolerance in meters
- `EMERGENCY_STOP_DIST = 0.3` - Emergency brake distance

## Usage

```bash
# Launch all nodes
ros2 run wallfollower maze_decision_node
ros2 run wallfollower maze_motion_node
ros2 run wallfollower goal_bridge_node

# Or use a launch file
ros2 launch wallfollower maze_explore.launch.py
```

To set a goal in RViz:
1. Click "Publish Point" in RViz toolbar
2. Click on the map where you want the robot to go
3. The robot will navigate to that grid cell using BFS pathfinding

## Dependencies

- `rclcpp` - ROS 2 C++ client library
- `geometry_msgs` - Pose and twist messages
- `sensor_msgs` - LiD scan messages
- `nav_msgs` - Odometry messages
- `visualization_msgs` - RViz markers
- `tf2` - Transform utilities

## Build

```bash
colcon build --packages-select wallfollower
```

## License

TODO: License declaration

## Maintainer

furkan <hamleniyap@gmail.com>
