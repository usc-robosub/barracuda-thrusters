source /opt/ros/noetic/setup.bash
cd /opt/barracuda-thruster-output-controller/catkin_ws/ && catkin_make
source /opt/barracuda-thruster-output-controller/catkin_ws/devel/setup.bash

# add args to this entrypoint to do different execs (I think CMD gets overrided)
# exec tail -f /dev/null
roslaunch barracuda_thruster_output_controller thruster_controller.launch