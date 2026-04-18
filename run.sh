#!/bin/bash
set -e
clear

echo "🔧 Sourcing ROS 2..."
source /opt/ros/jazzy/setup.bash
source ~/clearpath/setup.bash

echo "🔨 Building workspace..."
# Sadece mapping paketinde değişiklik yaptıysanız hızlı derleme için 
# colcon build --symlink-install --packages-select mapping de kullanabilirsiniz.
colcon build --symlink-install

echo "📦 Sourcing workspace..."
source install/setup.bash

echo "🚀 Launching system..."
# Launch dosyasını başlatıyoruz
ros2 launch mapping launch.py