FROM ros:humble-ros-base-jammy

RUN apt-get update && apt-get install -y \
    git \
    openssh-client \
    vim \
    i2c-tools \
    python3-colcon-common-extensions \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep update

WORKDIR /ros2_ws

COPY barracuda_thrusters /ros2_ws/src/barracuda_thrusters

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
    && echo "[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash" >> ~/.bashrc \
    && sed -i '1iforce_color_prompt=yes' ~/.bashrc

ENTRYPOINT ["/entrypoint.sh"]

CMD ["bash"]
