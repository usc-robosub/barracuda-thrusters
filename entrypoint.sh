#!/bin/bash
source /opt/ros/noetic/setup.bash
cd /opt/barracuda-thruster-output-controller/catkin_ws
catkin_make
source devel/setup.bash
roslaunch barracuda_thruster_output_controller thruster_controller.launch
