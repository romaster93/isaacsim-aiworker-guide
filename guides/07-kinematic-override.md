# Step 6: Kinematic Override Drive

## Overview
물리 기반 구동을 운동학 제어로 대체합니다.
- 정확한 위치/방향 제어
- 네비게이션 및 매핑에 유용
- IsaacSim에서 로봇의 실시간 위치 지정
- 물리 시뮬레이션 우회 (선택적)

## Prerequisites
- [x] IsaacSim 5.1.0 설치 (Step 1)
- [x] URDF 임포트 (Step 2)
- [x] 관절 제어 기본 (Step 4)
- [x] Swerve Drive 이해 (Step 5)

## Step 6.1: 운동학 제어의 개념

### 물리 기반 vs 운동학 제어

| 특성 | 물리 기반 | 운동학 제어 |
|------|---------|----------|
| 시뮬레이션 | 완전 물리 시뮬레이션 | 물리 우회, 위치 직접 지정 |
| 정확성 | 마찰, 관성 고려 (현실적) | 완벽한 제어 (이상적) |
| 용도 | 동역학 검증 | 경로 계획, 네비게이션 |
| 계산 비용 | 높음 | 낮음 |
| 미끄러짐 | 발생 가능 | 발생 없음 |

## Step 6.2: IsaacSim에서 Kinematic 모드 설정

### 6.2.1 로봇 관절을 Kinematic으로 변경

1. **IsaacSim 실행**
   ```bash
   conda activate isaac_sim
   isaacsim
   ```

2. **로봇 선택**
   - Outliner에서 로봇 루트 선택 (e.g., `ffw_sg2_follower`)

3. **Articulation Properties 변경**
   - 우측 Properties 패널 열기
   - `Articulation` 섹션 찾기
   - `Enabled`: ON으로 유지
   - 각 관절 선택 후 Properties에서:
     - `Drive Type`: `Position` 또는 `Velocity`로 설정
     - `Kinematic`: ON (선택적, 물리 계산 우회)

4. **Stage 저장**
   - File → Save

### 6.2.2 RigidBody Kinematics 설정

로봇의 베이스(base_link)에 Kinematics를 적용:

1. Outliner에서 `base_link` 선택
2. Properties 패널에서:
   - `Physics` 섹션
   - `Kinematic`: ON
   - `Gravity`: OFF (선택적)

이렇게 하면 base_link의 위치/방향이 외부에서 직접 제어됩니다.

## Step 6.3: 운동학 제어 Python 노드

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import math
import time
from tf_transformations import euler_from_quaternion, quaternion_from_euler

class KinematicOverrideController(Node):
    def __init__(self):
        super().__init__('kinematic_override_controller')

        # 현재 로봇 상태
        self.x = 0.0  # 위치 X (m)
        self.y = 0.0  # 위치 Y (m)
        self.theta = 0.0  # 방향 θ (rad)

        # 목표 속도
        self.vx = 0.0  # 전진 속도 (m/s)
        self.vy = 0.0  # 좌측 속도 (m/s)
        self.omega = 0.0  # 회전 각속도 (rad/s)

        # 로봇 매개변수
        self.robot_name = "ffw_sg2_follower"
        self.base_frame = "base_link"
        self.world_frame = "world"

        # Subscribers
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.amcl_pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            10
        )

        # Publishers
        self.odom_publisher = self.create_publisher(
            Odometry,
            '/odom',
            10
        )

        # TF Broadcaster (로봇 위치 발행)
        self.tf_broadcaster = None
        try:
            from tf2_ros import TransformBroadcaster
            from geometry_msgs.msg import TransformStamped
            self.tf_broadcaster = TransformBroadcaster(self)
        except ImportError:
            self.get_logger().warn("tf2_ros not available")

        # 타이머: 100Hz
        self.last_time = time.time()
        self.timer = self.create_timer(0.01, self.update_kinematic_state)

        self.get_logger().info("Kinematic Override Controller initialized")

    def cmd_vel_callback(self, msg: Twist):
        """Twist 메시지 수신 콜백"""
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.omega = msg.angular.z

        self.get_logger().debug(
            f"Cmd_vel: vx={self.vx:.2f}, vy={self.vy:.2f}, "
            f"omega={self.omega:.2f}"
        )

    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        """AMCL 포즈 수신 (위치 업데이트)"""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # 쿼터니언에서 θ 추출
        q = msg.pose.pose.orientation
        euler = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.theta = euler[2]  # Z축 회전 (yaw)

        self.get_logger().debug(
            f"AMCL pose: x={self.x:.2f}, y={self.y:.2f}, "
            f"theta={self.theta:.2f}"
        )

    def update_kinematic_state(self):
        """운동학 상태 업데이트"""
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        if dt > 0.1:  # 타이머 지연 보정
            return

        # 속도 명령을 로봇 좌표계에서 월드 좌표계로 변환
        # 회전 행렬 적용
        cos_theta = math.cos(self.theta)
        sin_theta = math.sin(self.theta)

        # 월드 좌표계 속도
        vx_world = self.vx * cos_theta - self.vy * sin_theta
        vy_world = self.vx * sin_theta + self.vy * cos_theta

        # 위치 적분 (오일러 방법)
        self.x += vx_world * dt
        self.y += vy_world * dt
        self.theta += self.omega * dt

        # θ 정규화 [-π, π]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # 로봇 위치 IsaacSim으로 발행 (선택적)
        self.publish_pose_to_isaac_sim()

        # Odometry 발행
        self.publish_odometry(current_time, vx_world, vy_world)

    def publish_pose_to_isaac_sim(self):
        """IsaacSim에 로봇 위치 발행 (TF를 통해)"""
        if self.tf_broadcaster is None:
            return

        try:
            from geometry_msgs.msg import TransformStamped
            from tf2_ros import TransformBroadcaster

            # Transform 생성
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.world_frame
            t.child_frame_id = self.base_frame

            # 위치
            t.transform.translation.x = float(self.x)
            t.transform.translation.y = float(self.y)
            t.transform.translation.z = 0.0

            # 방향 (쿼터니언)
            q = quaternion_from_euler(0, 0, self.theta)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            # 발행
            self.tf_broadcaster.sendTransform(t)
        except Exception as e:
            self.get_logger().debug(f"TF broadcast error: {e}")

    def publish_odometry(self, current_time, vx_world, vy_world):
        """Odometry 메시지 발행"""
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.world_frame
        msg.child_frame_id = self.base_frame

        # 위치
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0

        # 방향
        q = quaternion_from_euler(0, 0, self.theta)
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]

        # 속도 (월드 좌표계)
        msg.twist.twist.linear.x = vx_world
        msg.twist.twist.linear.y = vy_world
        msg.twist.twist.linear.z = 0.0
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = self.omega

        # 공분산 (Odometry 신뢰도)
        msg.pose.covariance[0] = 0.01  # x 분산
        msg.pose.covariance[7] = 0.01  # y 분산
        msg.pose.covariance[35] = 0.01  # theta 분산
        msg.twist.covariance[0] = 0.01
        msg.twist.covariance[7] = 0.01
        msg.twist.covariance[35] = 0.01

        self.odom_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    controller = KinematicOverrideController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Step 6.4: 실행 및 테스트

### 6.4.1 스크립트 저장

```bash
# 스크립트 저장
cat > /home/cho/ms_AIworker/scripts/kinematic_override_controller.py << 'EOF'
# 위의 Python 코드를 여기에 붙여넣기
EOF

chmod +x /home/cho/ms_AIworker/scripts/kinematic_override_controller.py
```

### 6.4.2 실행

```bash
# ROS2 환경 활성화
source /opt/ros/jazzy/setup.bash

# 1단계: Kinematic Override 컨트롤러 실행
python3 /home/cho/ms_AIworker/scripts/kinematic_override_controller.py

# 다른 터미널 - 속도 명령 발행
source /opt/ros/jazzy/setup.bash

# 전진
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.5, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}'

# 회전
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0.5}}'
```

### 6.4.3 Odometry 모니터링

```bash
source /opt/ros/jazzy/setup.bash

# Odometry 메시지 확인
ros2 topic echo /odom

# TF 변환 확인
ros2 run tf2_tools view_frames
```

## Step 6.5: IsaacSim과 연동

### 6.5.1 IsaacSim에서 로봇 위치 읽기

IsaacSim의 Action Graph에서 로봇의 현재 위치를 읽고, ROS2로 발행:

1. **IsaacSim 실행**
   ```bash
   conda activate isaac_sim
   isaacsim
   ```

2. **Window** → **Action Graph** 열기

3. **노드 추가**
   - `Read Prims As Bundle` (로봇 상태 읽기)
   - `ROS2 Publish Odometry` (Odometry 발행)

4. **연결**
   - Prim 경로: `/World/ffw_sg2_follower`
   - Odometry Topic: `/odom`

### 6.5.2 IsaacSim에서 로봇 위치 설정

```python
# IsaacSim Python API를 사용하여 로봇 위치 직접 설정
from isaacsim import core
from pxr import Gf, Usd

# 로봇 Prim 찾기
stage = core.get_context().get_stage()
robot_prim = stage.GetPrimAtPath("/World/ffw_sg2_follower")

# 위치 설정
translate_attr = robot_prim.GetAttribute("xformOp:translate")
rotate_attr = robot_prim.GetAttribute("xformOp:rotateXYZ")

# 새 위치
translate_attr.Set(Gf.Vec3d(1.0, 2.0, 0.5))  # x, y, z
rotate_attr.Set(Gf.Vec3d(0, 0, 0.785))  # roll, pitch, yaw (라디안)
```

## Step 6.6: 고급: Path Following

운동학 제어를 사용하여 사전 정의된 경로를 따르도록:

```python
class PathFollower(KinematicOverrideController):
    def __init__(self, path_points):
        super().__init__()
        self.path_points = path_points  # [(x, y), (x, y), ...]
        self.current_waypoint = 0
        self.reached_tolerance = 0.1  # 0.1m

    def update_path_following(self):
        """경로 추종"""
        if self.current_waypoint >= len(self.path_points):
            self.get_logger().info("Path complete")
            return

        target_x, target_y = self.path_points[self.current_waypoint]

        # 목표까지의 거리
        dx = target_x - self.x
        dy = target_y - self.y
        distance = math.sqrt(dx**2 + dy**2)

        if distance < self.reached_tolerance:
            self.current_waypoint += 1
            self.get_logger().info(f"Waypoint {self.current_waypoint} reached")
            return

        # 목표 방향
        target_theta = math.atan2(dy, dx)

        # Simple P-controller
        # 각도 오차
        angle_error = target_theta - self.theta
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        # 속도 명령 계산
        Kp_linear = 0.5
        Kp_angular = 1.0

        self.vx = Kp_linear * distance
        self.omega = Kp_angular * angle_error

        self.get_logger().debug(
            f"Following waypoint: distance={distance:.2f}, "
            f"angle_error={angle_error:.2f}"
        )

# 사용 예
path = [(0, 0), (1, 0), (1, 1), (0, 1), (0, 0)]  # 사각형
follower = PathFollower(path)
```

## Troubleshooting

### 로봇이 움직이지 않음
- Kinematic 모드가 활성화되어 있는지 확인
- IsaacSim에서 로봇의 `Kinematic` 속성 확인

### Odometry 정보가 정확하지 않음
- 로봇의 실제 좌표를 AMCL이나 외부 위치 추정으로 업데이트
- 공분산 값 조정

### TF 변환 실패
- tf2_ros 설치 확인: `sudo apt-get install ros-jazzy-tf2-ros`
- TF Tree 확인: `ros2 run tf2_tools view_frames`

### 경로 추종이 불안정함
- P 게인 조정 (Kp_linear, Kp_angular)
- 목표점 도달 거리 임계값 증가

---
**Status**: PENDING
**Next**: [Step 7: Navigation System](07-navigation-system.md)
