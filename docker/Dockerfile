# syntax=docker/dockerfile:1

FROM osrf/ros:humble-desktop
SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive \
    COLCON_WS=/fishbot_ws \
    ROS_DOMAIN_ID=0

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        git \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-vcstool \
        wget \
        ros-humble-gazebo-ros-pkgs \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /fishbot_ws
COPY . /fishbot_ws

# TODO: this is hot-fix to not rebuild docker (remove rm -rf /var/lib... in the future)
RUN apt-get update 

RUN rosdep update
RUN rosdep install --rosdistro humble --from-paths src --ignore-src -y
RUN source /opt/ros/humble/setup.bash && colcon build --merge-install

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
