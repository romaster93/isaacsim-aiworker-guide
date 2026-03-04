# Step 7: Navigation System

## Overview
FFW-SG2 로봇의 완전한 네비게이션 시스템을 구축합니다.
- ROBOTIS 제공 Docker 기반 네비게이션
- 텔레옵 제어 (Teleoperation)
- SLAM 지도 작성
- ai_worker 소스 코드 구조 이해

## Prerequisites
- [x] IsaacSim 5.1.0 설치 (Step 1)
- [x] URDF 임포트 (Step 2)
- [x] 센서 구성 (Step 3)
- [x] 관절 제어 (Step 4)
- [x] Swerve Drive (Step 5)
- [x] ROS2 Jazzy 환경

## Step 7.1: ai_worker 소스 코드 구조

ROBOTIS ai_worker 프로젝트의 주요 패키지 구조:

```
ai_worker/
├── ffw_description/          # 로봇 설명 (URDF, 메시, 환경설정)
│   ├── meshes/              # 로봇 메시 파일
│   ├── urdf/                # URDF 정의
│   │   └── ffw_sg2_rev1_follower/
│   │       └── ffw_sg2_follower.urdf
│   └── config/              # 로봇 매개변수
│
├── ffw_bringup/             # 로봇 부팅 및 시작
│   ├── launch/              # ROS2 Launch 파일
│   │   ├── ffw_bringup.launch.py
│   │   └── rviz.launch.py
│   └── config/              # 설정 파일
│
├── ffw_navigation/          # 네비게이션 스택
│   ├── config/              # Nav2 설정
│   │   ├── nav2_params.yaml
│   │   ├── costmap_common_params.yaml
│   │   └── planner_params.yaml
│   ├── launch/              # 네비게이션 실행
│   │   └── navigation_launch.py
│   ├── maps/                # 저장된 지도
│   │   ├── warehouse.yaml
│   │   └── warehouse.pgm
│   └── rviz/                # Rviz 설정
│
├── ffw_swerve_drive_controller/  # Swerve 드라이브 제어
│   ├── src/
│   │   ├── swerve_drive_controller.cpp
│   │   ├── swerve_odometry.cpp
│   │   └── speed_limiter.cpp
│   └── include/
│
├── ffw_teleop/              # 텔레옵 제어
│   └── ffw_teleop/
│       └── mobile_teleop.py  # 키보드 텔레옵 스크립트
│
└── ai_worker_sim_pkg/       # IsaacSim 통합 (선택적)
```

## Step 7.2: 텔레옵 제어 (Keyboard)

### 7.2.1 ROBOTIS 제공 텔레옵 스크립트

ai_worker는 Python 기반 텔레옵 스크립트를 제공합니다:

```bash
# ROS2 환경 활성화
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# 텔레옵 실행
python3 ~/ros2_ws/src/ai_worker/ffw_teleop/ffw_teleop/mobile_teleop.py
```

### 7.2.2 텔레옵 스크립트 (Python)

대안으로, 직접 구현한 텔레옵 스크립트:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import tty
import termios

class MobileTeleop(Node):
    def __init__(self):
        super().__init__('mobile_teleop')

        self.settings = termios.tcgetattr(sys.stdin)
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # 속도 설정
        self.linear_speed = 0.5  # m/s
        self.angular_speed = 0.5  # rad/s

        # 키맵
        self.key_map = {
            'w': (1, 0, 0),      # Forward
            'a': (0, 0, 1),      # Rotate left
            's': (-1, 0, 0),     # Backward
            'd': (0, 0, -1),     # Rotate right
            'q': (0, 1, 0),      # Strafe left
            'e': (0, -1, 0),     # Strafe right
        }

        self.get_logger().info("Teleop started. Controls:")
        self.get_logger().info("  W: Forward, S: Backward")
        self.get_logger().info("  A: Rotate left, D: Rotate right")
        self.get_logger().info("  Q: Strafe left, E: Strafe right")
        self.get_logger().info("  I/K: Increase/Decrease linear speed")
        self.get_logger().info("  U/O: Increase/Decrease angular speed")
        self.get_logger().info("  SPACE: Stop, CTRL-C: Quit")

    def get_key(self):
        """논블로킹 키 입력"""
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            key = sys.stdin.read(1)
            return key.lower()
        return None

    def run(self):
        """메인 루프"""
        try:
            while rclpy.ok():
                key = self.get_key()

                if key is None:
                    continue

                if key == 'c':  # CTRL-C 처리
                    break

                # 속도 조정
                if key == 'i':
                    self.linear_speed = min(2.0, self.linear_speed + 0.1)
                    self.get_logger().info(f"Linear speed: {self.linear_speed:.1f}")
                elif key == 'k':
                    self.linear_speed = max(0.1, self.linear_speed - 0.1)
                    self.get_logger().info(f"Linear speed: {self.linear_speed:.1f}")
                elif key == 'u':
                    self.angular_speed = min(2.0, self.angular_speed + 0.1)
                    self.get_logger().info(f"Angular speed: {self.angular_speed:.1f}")
                elif key == 'o':
                    self.angular_speed = max(0.1, self.angular_speed - 0.1)
                    self.get_logger().info(f"Angular speed: {self.angular_speed:.1f}")
                elif key == ' ':  # 정지
                    self.publish_velocity(0, 0, 0)
                    continue
                elif key in self.key_map:
                    lin_x, lin_y, ang_z = self.key_map[key]
                    self.publish_velocity(
                        lin_x * self.linear_speed,
                        lin_y * self.linear_speed,
                        ang_z * self.angular_speed
                    )

        except Exception as e:
            self.get_logger().error(f"Error: {e}")
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def publish_velocity(self, vx, vy, omega):
        """속도 명령 발행"""
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(omega)

        self.cmd_vel_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    teleop = MobileTeleop()

    try:
        tty.setraw(sys.stdin.fileno())
        teleop.run()
    finally:
        teleop.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 7.2.3 텔레옵 실행

```bash
# 스크립트 저장
cat > /home/cho/ms_AIworker/scripts/mobile_teleop.py << 'EOF'
# 위의 Python 코드를 여기에 붙여넣기
EOF

chmod +x /home/cho/ms_AIworker/scripts/mobile_teleop.py

# ROS2 환경 활성화
source /opt/ros/jazzy/setup.bash

# 실행
python3 /home/cho/ms_AIworker/scripts/mobile_teleop.py
```

## Step 7.3: SLAM 지도 작성

SLAM (Simultaneous Localization and Mapping)을 사용하여 환경 지도를 작성합니다.

### 7.3.1 SLAM 패키지 설치

```bash
# ROS2 SLAM Toolbox 설치
sudo apt-get install ros-jazzy-slam-toolbox
sudo apt-get install ros-jazzy-nav2-bringup
sudo apt-get install ros-jazzy-rqt-nav2
```

### 7.3.2 SLAM 실행

```bash
# 터미널 1: ROS2 환경 및 Swerve 제어
source /opt/ros/jazzy/setup.bash
python3 /home/cho/ms_AIworker/scripts/swerve_controller.py

# 터미널 2: SLAM 시작
source /opt/ros/jazzy/setup.bash
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=slam_params.yaml

# 터미널 3: Rviz 시각화
source /opt/ros/jazzy/setup.bash
rviz2
```

### 7.3.3 Rviz에서 지도 작성

1. **Rviz 설정**
   - Fixed Frame: `odom` 또는 `map`
   - Add → Map (Topic: `/map`)
   - Add → RobotModel
   - Add → LaserScan (Topic: `/scan` - LiDAR)

2. **텔레옵으로 로봇 이동**
   - 터미널 3에서 텔레옵 스크립트 실행
   - 키보드로 로봇 제어
   - LiDAR가 환경을 스캔

3. **지도 저장**
   ```bash
   # 지도 저장
   mkdir -p /home/cho/ms_AIworker/maps
   ros2 run nav2_map_server map_saver_cli -f /home/cho/ms_AIworker/maps/warehouse
   ```

## Step 7.4: Navigation2 (Nav2) 설정

### 7.4.1 Nav2 설정 파일 생성

```bash
# Nav2 설정 디렉토리 생성
mkdir -p /home/cho/ms_AIworker/nav2_config
```

**nav2_params.yaml**:
```yaml
amcl:
  ros__parameters:
    use_sim_time: true
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_link"
    beam_search_angle: 0.545
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    sigma_short: 0.05
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05

bt_navigator:
  ros__parameters:
    use_sim_time: true
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_nav_to_pose_bt_xml: nav2_navigate_w_replanning_and_recovery.xml
    navigators: ["navigate_to_pose", "navigate_through_poses"]
    navigate_to_pose:
      plugin: "nav2_bt_navigator::NavigateToPoseNavigator"
    navigate_through_poses:
      plugin: "nav2_bt_navigator::NavigateThroughPosesNavigator"
    default_nav_through_poses_bt_xml: navigate_through_poses.xml
    action_server_result_timeout: 900.0
    navigators: ["navigate_to_pose", "navigate_through_poses"]
    use_sim_time: true

controller_server:
  ros__parameters:
    use_sim_time: true
    controller_frequency: 10.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      failure_tolerance: 5.0
      looking_ahead_window: 5.0
    goal_checker:
      plugins: ["general_goal_checker"]
      general_goal_checker:
        stateful: True
        plugin: "nav2_controller::SimpleGoalChecker"
        xy_goal_tolerance: 0.1
        yaw_goal_tolerance: 0.05
    local_costmap:
      local_costmap:
        update_frequency: 10.0
        publish_frequency: 10.0
        global_frame: odom
        robot_base_frame: base_link
        use_sim_time: true
        rolling_window: true
        width: 3
        height: 3
        resolution: 0.05
        robot_radius: 0.3
        plugins: ["obstacle_layer", "inflation_layer"]
        inflation_layer:
          plugin: "nav2_costmap_2d::InflationLayer"
          cost_scaling_factor: 3.0
          inflation_radius: 0.55
        obstacle_layer:
          plugin: "nav2_costmap_2d::ObstacleLayer"
          observation_sources: scan
          scan:
            topic: /scan
            max_obstacle_height: 2.0
            clearing: True
            marking: True
            data_type: "LaserScan"
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      desired_linear_vel: 0.5
      lookahead_dist: 0.6
      min_lookahead_dist: 0.3
      max_lookahead_dist: 0.9
      lookahead_time: 1.5
      rotate_to_heading_angular_vel: 1.8
      transform_tolerance: 0.1
      use_velocity_scaled_lookahead_dist: false
      min_amcl_pose_receive_count: 3
      use_regulated_linear_velocity_scaling: true
      use_cost_regulated_linear_velocity_scaling: false
      regulated_linear_scaling_min_radius: 0.9
      regulated_linear_scaling_min_speed: 0.25
      use_rotate_to_heading: true
      rotate_to_heading_min_angle: 0.785
      max_allowed_time_error_to_skip_extrapolation: 1.0
      use_interpolation: false
```

### 7.4.2 Nav2 실행

```bash
# ROS2 환경 활성화
source /opt/ros/jazzy/setup.bash

# Nav2 시작
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=true map:=/home/cho/ms_AIworker/maps/warehouse.yaml params_file:=/home/cho/ms_AIworker/nav2_config/nav2_params.yaml
```

## Step 7.5: ai_worker 소스 코드 구조 상세

### ffw_description
```
ffw_description/
├── meshes/
│   ├── base/
│   │   └── base.stl
│   ├── arm_l/
│   │   ├── link1.stl, link2.stl, ...
│   ├── gripper_l/
│   └── wheels/
├── urdf/
│   └── ffw_sg2_rev1_follower/
│       ├── ffw_sg2_follower.urdf
│       └── materials.xacro
└── config/
    ├── joint_limits.yaml
    └── robot_params.yaml
```

### ffw_swerve_drive_controller
```
ffw_swerve_drive_controller/
├── src/
│   ├── swerve_drive_controller.cpp
│   │   - SwerveController 클래스 (역운동학)
│   │   - update() 함수 (제어 루프)
│   ├── swerve_odometry.cpp
│   │   - 오도메트리 계산
│   ├── speed_limiter.cpp
│   │   - 최대 속도 제한
│   └── parameter_manager.cpp
└── include/
    └── ffw_swerve_drive_controller/
        ├── swerve_controller.hpp
        └── speed_limiter.hpp
```

### ffw_navigation
```
ffw_navigation/
├── config/
│   ├── nav2_params.yaml        # Nav2 매개변수
│   ├── costmap_common_params.yaml
│   ├── planner_params.yaml     # 경로 계획 매개변수
│   └── controller_params.yaml  # 제어기 매개변수
├── launch/
│   ├── navigation_launch.py
│   ├── localization_launch.py
│   └── slam_launch.py
├── maps/
│   ├── warehouse.pgm           # 지도 이미지
│   ├── warehouse.yaml          # 지도 메타데이터
│   ├── floor1.pgm
│   └── floor1.yaml
└── rviz/
    └── nav2_default_view.rviz
```

### ffw_teleop
```
ffw_teleop/
└── ffw_teleop/
    ├── mobile_teleop.py        # 키보드 텔레옵
    ├── joystick_teleop.py      # 조이스틱 텔레옵 (선택적)
    └── __init__.py
```

## Step 7.6: 실전 워크플로우

### 7.6.1 로봇 초기화

```bash
# 1. ROS2 환경 활성화
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# 2. URDF 확인
ros2 param get /robot_state_publisher robot_description

# 3. 노드 실행
# 터미널 A: 로봇 상태 발행자 (실제 로봇 또는 IsaacSim)
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat ~/ros2_ws/src/ai_worker/ffw_description/urdf/ffw_sg2_follower.urdf)"

# 터미널 B: Swerve 드라이브 제어
python3 /home/cho/ms_AIworker/scripts/swerve_controller.py

# 터미널 C: AMCL 위치 추정
ros2 launch nav2_bringup localization_launch.py map:=~/maps/warehouse.yaml use_sim_time:=true

# 터미널 D: 텔레옵 제어
python3 /home/cho/ms_AIworker/scripts/mobile_teleop.py

# 터미널 E: Navigation2
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true

# 터미널 F: Rviz 시각화
rviz2
```

### 7.6.2 자율 네비게이션

Rviz에서:
1. "Set Pose" 버튼으로 초기 위치 설정
2. "Nav2 Goal" 버튼으로 목표 위치 설정
3. 로봇이 자동으로 경로 계획 및 이동

## Troubleshooting

### SLAM이 작동하지 않음
- LiDAR 센서 활성화 확인
- Topic `/scan` 확인: `ros2 topic echo /scan`
- TF 변환 확인: `ros2 run tf2_tools view_frames`

### 위치 추정 오류
- AMCL 초기 위치 설정 확인
- 지도 품질 확인 (충분한 특징점)

### 네비게이션이 느림
- 경로 계획기 매개변수 조정
- 로컬 코스트맵 해상도 증가
- 계산 성능 확인 (CPU 사용률)

### 텔레옵이 반응하지 않음
- ROS2 토픽 확인: `ros2 topic list`
- 권한 확인: `ls -la /dev/tty*`

---
**Status**: PENDING
**Next**: [Complete! Review and Test](../PROGRESS.md)
