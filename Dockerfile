FROM ros:jazzy-ros-base-noble
# SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y \
    git \
    openssh-client \
    vim \
    i2c-tools \
#    python3-rospy \
    python3-smbus \
    python3-rpi.gpio \
    && rm -rf /var/lib/apt/lists/*

# COPY catkin_ws /opt/barracuda-thruster-output-controller/catkin_ws
# COPY entrypoint.sh /opt/barracuda-thruster-output-controller/entrypoint.sh

# RUN chmod +x /opt/barracuda-thruster-output-controller/entrypoint.sh \
#     && chmod +x /opt/barracuda-thruster-output-controller/catkin_ws/src/barracuda_thruster_output_controller/scripts/F2PWM.py \
#     && chmod +x /opt/barracuda-thruster-output-controller/catkin_ws/src/barracuda_thruster_output_controller/scripts/rpi_thruster_controller.py \
#     && source /opt/ros/noetic/setup.bash && echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc \
#     && echo "[ -f /opt/barracuda-thruster-output-controller/catkin_ws/devel/setup.bash ] && source /opt/barracuda-thruster-output-controller/catkin_ws/devel/setup.bash" >> /root/.bashrc \ 
#     && echo "cd /opt/barracuda-thruster-output-controller/catkin_ws" >> /root/.bashrc \
#     && cd /opt/barracuda-thruster-output-controller/catkin_ws && catkin_make
#     
WORKDIR /opt/barracuda-thrusters/dev_ws
# CMD ["/opt/barracuda-thruster-output-controller/entrypoint.sh"]
