source /opt/ros/noetic/setup.bash
source /opt/barracuda-thruster-output-controller/catkin_ws/devel/setup.bash
# cd /opt/barracuda-thruster-output-controller/catkin_ws/

# add args to this entrypoint to do different execs (I think CMD gets overrided)
# exec tail -f /dev/null
# roslaunch --wait barracuda_thruster_output_controller thruster_controller.launch
roslaunch barracuda_thruster_output_controller thruster_controller.launch