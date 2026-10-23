FROM ros:noetic

ARG DEBIAN_FRONTEND=noninteractive
ARG CATKIN_BUILD_JOBS=2

ENV ROS_DISTRO=noetic \
    ROS_WS=/home/ubuntu/share/tmp/ArmBot

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
WORKDIR ${ROS_WS}

COPY ros1-latest-archive-keyring.gpg /usr/share/keyrings/ros1-latest-archive-keyring.gpg

RUN sed -i 's|http://ports.ubuntu.com/ubuntu-ports|https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports|g' /etc/apt/sources.list \
    && sed -i 's|http://packages.ros.org/ros/ubuntu|https://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu|g' /etc/apt/sources.list.d/ros1-latest.list \
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
        python3-tk \
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
RUN python3 -m pip install --no-cache-dir \
    --index-url https://pypi.tuna.tsinghua.edu.cn/simple \
    -r requirements.txt

RUN python3 -m pip install --no-cache-dir --no-deps \
    --index-url https://pypi.tuna.tsinghua.edu.cn/simple \
    --target=/opt/opencv-4.9 opencv-python-headless==4.9.0.80

COPY src/ ./src/
RUN source /opt/ros/noetic/setup.bash \
    && catkin_make -j"${CATKIN_BUILD_JOBS}" -l"${CATKIN_BUILD_JOBS}"

# FastMCP uses Python >=3.10; Noetic and the other nodes keep system Python 3.8.
RUN bash src/AI_pkg/scripts/install_ai_runtime.sh

CMD ["bash"]
