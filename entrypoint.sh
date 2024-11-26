#!/usr/bin/bash
source /opt/ros/noetic/setup.bash

# Build & Source catkin_ws
cd barracuda-thruster-output-server/catkin_ws
catkin_make
source devel/setup.bash

# Start the launch file (add --wait after other node starts roscore)
roslaunch barracuda_thruster_output_server barracuda_thruster_output_server.launch --wait

# Following is for debugging purposes

# Source ros and catkin_ws in bashrc
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "source /opt/barracuda-thruster-output-server/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Start interactive shell session in /opt/barracuda-thruster-output-server directory
cd ..
exec /bin/bash