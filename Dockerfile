FROM ros:noetic-ros-base-focal
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y \
    vim \
    i2c-tools \
    python3-pandas \
    python3-numpy \
    python3-rospy \
    python3-pip \
    python3-rpi.gpio \
    && pip3 install --no-cache-dir smbus2 \
    && apt-get purge -y python3-pip \
    && apt-get autoremove -y \
    && rm -rf /var/lib/apt/lists/*
    
COPY . /opt/barracuda-thruster-output-controller
WORKDIR /opt
CMD ["/bin/bash", "/opt/barracuda-thruster-output-controller/entrypoint.sh"]
