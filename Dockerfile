FROM ros:noetic-ros-base-focal
COPY catkin_ws /opt/barracuda-thruster-output-controller/catkin_ws
COPY entrypoint.sh /opt/barracuda-thruster-output-controller/entrypoint.sh
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y \
    git \
    openssh-client \
    vim \
    i2c-tools \
    python3-rospy \
    python3-smbus \
    # python3-rpi.gpio \
    && rm -rf /var/lib/apt/lists/* \
    && chmod +x /opt/barracuda-thruster-output-controller/entrypoint.sh \
    && echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc \
    && echo "[ -f /opt/barracuda-thruster-output-controller/catkin_ws/devel/setup.bash ] && source /opt/barracuda-thruster-output-controller/catkin_ws/devel/setup.bash" >> /root/.bashrc \ 
    && echo "cd /opt/barracuda-thruster-output-controller/catkin_ws" >> /root/.bashrc
    
WORKDIR /opt/barracuda-thruster-output-controller/catkin_ws/
CMD ["/opt/barracuda-thruster-output-controller/entrypoint.sh"]