#!/bin/bash
set -e

if [ "$DEPLOY_ENV" = "jetson" ]; then
    echo "Hardware deployment detected: Installing Jetson.GPIO via pip..."
    pip install Jetson.GPIO
else
    echo "Local dev environment detected: Skipping hardware-specific GPIO library."
fi

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

echo "=========================================="
echo " Barracuda Thrusters Workspace Ready! "
echo "=========================================="

exec "$@"
