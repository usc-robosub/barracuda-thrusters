#!/bin/bash
set -e

echo "Installing dependencies..."
apt-get update
rosdep install --from-paths src --ignore-src -y --skip-keys="python3-jetson-gpio"

echo "Building ROS 2 workspace..."
colcon build --symlink-install

source install/setup.bash

echo "=========================================="
echo " Barracuda Thrusters Workspace Ready! "
echo "=========================================="

exec "$@"
