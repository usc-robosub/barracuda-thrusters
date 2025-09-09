FROM ros:jazzy-ros-base-noble

RUN apt-get update && apt-get install -y \
    git \
    openssh-client \
    vim \
    i2c-tools \
    python3-smbus \
    python3-rpi.gpio \
    && rm -rf /var/lib/apt/lists/*

COPY . /opt/barracuda-thrusters

RUN . /opt/ros/jazzy/setup.sh && \
  cd /opt/barracuda-thrusters/dev_ws && \
  colcon build --symlink-install

WORKDIR /opt/barracuda-thrusters/dev_ws
CMD ["/bin/bash", "/opt/barracuda-thrusters/entrypoint.sh"]
