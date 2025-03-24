FROM ros:noetic-ros-base-focal
COPY . /opt/barracuda-thruster-output-controller
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y \
    git \
    openssh-client \
    vim \
    && rm -rf /var/lib/apt/lists/* \
    && chmod +x /opt/barracuda-thruster-output-controller/entrypoint.sh \
    && mkdir /root/.ssh && chmod 700 /root/.ssh \
    && ssh-keyscan github.com >> /root/.ssh/known_hosts \
    && git config --global url."git@github.com:".insteadOf "https://github.com/" \
    && echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc \
    && echo "[ -f /opt/barracuda-thruster-output-controller/catkin_ws/devel/setup.bash ] && source /opt/barracuda-thruster-output-controller/catkin_ws/devel/setup.bash" >> /root/.bashrc \ 
    && echo "cd /opt/barracuda-thruster-output-controller/catkin_ws" >> /root/.bashrc
    
WORKDIR /opt/barracuda-thruster-output-controller/catkin_ws/
CMD ["/opt/barracuda-thruster-output-controller/entrypoint.sh"]