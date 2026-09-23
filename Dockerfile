FROM ros:noetic

ARG DEBIAN_FRONTEND=noninteractive
ARG CATKIN_BUILD_JOBS=2

ENV ROS_DISTRO=noetic \
    ROS_WS=/home/ubuntu/share/tmp/ArmBot

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
WORKDIR ${ROS_WS}

ADD https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc /tmp/ros.asc

RUN gpg --dearmor --batch --yes -o /usr/share/keyrings/ros1-latest-archive-keyring.gpg /tmp/ros.asc \
    && rm /tmp/ros.asc \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        libdw-dev \
        libudev-dev \
        libusb-1.0-0-dev \
        alsa-utils \
        python3-dev \
        python3-numpy \
        python3-opencv \
        python3-pil \
        python3-pip \
        python3-pyqt5 \
        python3-serial \
        python3-smbus \
        python3-yaml \
        ros-noetic-catkin \
        ros-noetic-camera-info-manager \
        ros-noetic-diagnostic-updater \
        ros-noetic-gazebo-ros-control \
        ros-noetic-gazebo-ros-pkgs \
        ros-noetic-image-geometry \
        ros-noetic-image-transport \
        ros-noetic-joint-state-publisher \
        ros-noetic-joint-state-publisher-gui \
        ros-noetic-moveit \
        ros-noetic-robot-state-publisher \
        ros-noetic-ros-control \
        ros-noetic-ros-controllers \
        ros-noetic-rviz \
        ros-noetic-tf2-geometry-msgs \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt ./requirements.txt
RUN python3 -m pip install --no-cache-dir -r requirements.txt

COPY src/ ./src/
RUN source /opt/ros/noetic/setup.bash \
    && catkin_make -j"${CATKIN_BUILD_JOBS}" -l"${CATKIN_BUILD_JOBS}"

CMD ["bash"]
