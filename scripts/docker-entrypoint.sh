#!/bin/bash
# IsaacSim Docker 컨테이너 entrypoint
# ROS2 Bridge 환경변수를 자동 설정합니다.

# ROS2 Bridge 환경변수
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.bridge/humble/lib:${LD_LIBRARY_PATH}
export FASTRTPS_DEFAULT_PROFILES_FILE=/isaac-sim/fastdds.xml

# .bashrc에도 추가 (docker exec bash 시 자동 적용)
cat >> /etc/bash.bashrc << 'ROSENV'
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.bridge/humble/lib:${LD_LIBRARY_PATH}
export FASTRTPS_DEFAULT_PROFILES_FILE=/isaac-sim/fastdds.xml
ROSENV

exec "$@"
