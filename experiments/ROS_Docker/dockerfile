FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip python3-colcon-common-extensions bash-completion \
 && python3 -m pip install --no-cache-dir --upgrade "pip<21.3" \
 && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /ws/src
ENV ROS_DOMAIN_ID=42
SHELL ["/bin/bash", "-c"]
RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
WORKDIR /ws
CMD ["bash"]
