FROM nvcr.io/nvidia/isaac-sim:5.1.0

# ROS2 Jazzy CLI 설치 (Ubuntu 24.04 Noble 기반, ros2 명령어용)
USER root

RUN apt-get update && apt-get install -y \
    curl \
    gnupg \
    lsb-release \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" > /etc/apt/sources.list.d/ros2.list \
    && apt-get update && apt-get install -y \
    ros-jazzy-ros-base \
    ros-jazzy-rmw-fastrtps-cpp \
    && rm -rf /var/lib/apt/lists/*

# FastDDS 설정 (Shared Memory 대신 UDP 사용 - 호스트↔컨테이너 통신용)
COPY fastdds.xml /isaac-sim/fastdds.xml

# ros2 CLI용: jazzy 환경 + FastDDS UDP 설정 (bash 세션)
RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc \
    && echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> /etc/bash.bashrc \
    && echo "export FASTRTPS_DEFAULT_PROFILES_FILE=/isaac-sim/fastdds.xml" >> /etc/bash.bashrc

# 볼륨 디렉토리 권한 미리 설정
RUN mkdir -p /isaac-sim/.cache /isaac-sim/.local /isaac-sim/.nvidia-omniverse /isaac-sim/.nv \
    && chown -R isaac-sim:isaac-sim /isaac-sim/.cache /isaac-sim/.local /isaac-sim/.nvidia-omniverse /isaac-sim/.nv

USER isaac-sim
WORKDIR /isaac-sim

ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/isaac-sim/fastdds.xml
