#!/bin/bash
set -e
clear

echo "🔧 Sourcing ROS 2..."
source /opt/ros/jazzy/setup.bash
source ~/clearpath/setup.bash

echo "🔨 Building workspace..."
colcon build --symlink-install

echo "📦 Sourcing workspace..."
source install/setup.bash

echo "🤖 Launching simulation..."
ros2 launch clearpath_gz simulation.launch.py \
  x:=-3.3 y:=3.3 \
  world:=/home/furkan/Documents/new/worlds/generated_maze \
  rviz:=true\
  use_sim_time:=true &
LAUNCH_PID=$!



ros2 run mapping Occupancy &
MAPP=$!



trap "echo '🛑 Durduruluyor...'; kill $LAUNCH_PID $MAPP 2>/dev/null; exit 0" SIGINT SIGTERM

wait $LAUNCH_PID