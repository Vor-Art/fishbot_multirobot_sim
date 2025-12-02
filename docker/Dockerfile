FROM osrf/ros:humble-desktop
SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive \
    COLCON_WS=/fishbot_ws

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        git \
        wget \
        python3-rosdep \
        python3-vcstool \
        python3-colcon-common-extensions

# DO NOT CLEAR THE `rm -rf /var/lib/apt/lists/*`
# it leads to the errors in rosdep install

# Source ROS and workspace by default
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source $COLCON_WS/install/setup.bash" >> ~/.bashrc
