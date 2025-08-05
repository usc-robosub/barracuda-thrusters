source /opt/ros/noetic/setup.bash
[ -f /opt/barracuda-thruster-output-controller/catkin_ws/devel/setup.bash ] && source /opt/barracuda-thruster-output-controller/catkin_ws/devel/setup.bash
# exec tail -f /dev/null
roslaunch barracuda_thruster_output_controller thruster_controller.launch