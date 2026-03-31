# Step 6: Swerve Drive 제어

## Overview

이 가이드에서는 FFW-SG2 로봇의 **swerve drive**(스위브 드라이브)를 ROS2로 제어합니다.

swerve drive란? 각 바퀴가 **독립적으로 방향을 바꿀 수 있는** 구동 방식입니다.
일반 자동차는 앞바퀴만 꺾이지만, swerve drive는 모든 바퀴가 360도 회전 가능하여
전진, 후진은 물론 **옆으로 이동(횡이동)**이나 **제자리 회전**까지 가능합니다.

이 가이드를 끝내면:
- `/cmd_vel` 토픽으로 로봇에게 "앞으로 가", "왼쪽으로 가", "돌아" 명령을 보낼 수 있음
- 키보드로 로봇을 조종할 수 있음
- swerve drive의 역운동학(IK)이 어떻게 동작하는지 이해함

> 유튜브에서 **"swerve drive robot"** 검색하면 실제 동작을 바로 볼 수 있습니다.
> FRC(FIRST Robotics Competition) 로봇들이 swerve drive를 많이 사용합니다.

### 가이드 구조

| 섹션 | 내용 | 할 일 |
|------|------|-------|
| [1] 개념 이해 | swerve drive란 무엇인가 | 읽기 |
| [2] 물리 설정 | IsaacSim에서 바퀴 설정 | **IsaacSim에서 실행** |
| [3] 역운동학 | 수학 원리 | 읽기 (이해 안 돼도 OK) |
| [4] 코드 분석 | 스크립트 동작 원리 | 읽기 |
| [5] 실행 | 실제로 로봇 움직이기 | **터미널에서 실행** |
| [6] 디버깅 | 안 될 때 해결법 | 필요할 때 참고 |
| [7] FAQ | 자주 묻는 질문 | 필요할 때 참고 |

## Prerequisites

- [x] Step 5 (관절 제어) 완료 — [3] Action Graph 설정이 되어 있어야 합니다
- [x] IsaacSim Docker 컨테이너가 실행 중 (`docker-compose up`)
- [x] IsaacSim Play(▶) 상태에서 관절이 움직이는 것을 확인한 상태
- [x] 호스트 터미널에서 `source ros2-bridge-env.sh` 후 `ros2 topic list`에 IsaacSim 토픽이 보이는 상태

> **아직 Step 5를 안 했다면?** 반드시 먼저 완료하세요.
> 이 가이드의 모든 것은 Step 5의 [3] Action Graph(`/JointControlGraph`)를 통해 동작합니다.

### 전체 데이터 흐름 (이 가이드에서 만들 것)

```
키보드 입력 (또는 ros2 topic pub)
    │
    ▼
teleop_twist_keyboard (키 → Twist 변환)
    │
    │  /cmd_vel (geometry_msgs/Twist)
    │  "vx=0.3, vy=0, omega=0.5"
    ▼
swerve_controller.py (역운동학 계산)
    │
    │  /isaac_sim/joint_commands (JointState)
    │  "left_steer=0.3rad, left_drive=8rad/s, ..."
    ▼
IsaacSim Action Graph (Step 5에서 만든 것)
    │
    ▼
로봇이 움직임!
```

> **요약**: 키보드 → `/cmd_vel` 토픽 → swerve_controller가 계산 → `/isaac_sim/joint_commands` 토픽 → IsaacSim이 로봇 구동

---

## [1] Swerve Drive 개념 이해

### 1.1 바퀴 구성

FFW-SG2는 **3륜 swerve** 방식입니다. 3개의 바퀴를 삼각형으로 배치하면
어느 방향으로든 안정적으로 이동할 수 있습니다 (4륜보다 구조가 단순하면서 전방향 이동 가능):

```
        전진 방향 (X+)
            ↑
            |
    L ------+------ R      ← 왼쪽(L), 오른쪽(R) 바퀴가 앞쪽
            |
            |
            B               ← 뒤쪽(B) 바퀴
```

각 바퀴에는 **2개의 모터**가 있습니다:

| 모터 | 역할 | 제어 방식 | 예시 |
|------|------|-----------|------|
| **Steer** (조향) | 바퀴 방향을 돌림 | Position Control (각도) | "바퀴를 45도로 꺾어라" |
| **Drive** (구동) | 바퀴를 굴림 | Velocity Control (속도) | "초당 5rad로 굴려라" (약 48RPM) |

> **왜 2개의 모터?** 자동차를 생각해보세요. 핸들(steer)로 방향을 정하고,
> 엑셀(drive)로 속도를 정합니다. swerve drive도 같은 원리인데,
> 모든 바퀴마다 각각 핸들과 엑셀이 있는 셈입니다.

### 1.2 Joint 이름 (IsaacSim에서 사용하는 이름)

이 이름은 **URDF 파일에 정의된 joint name**입니다. Step 2에서 URDF를 임포트하면
IsaacSim이 자동으로 이 이름을 가져옵니다. Stage 패널에서도 확인할 수 있습니다.

| 바퀴 | Steer Joint (방향) | Drive Joint (구동) |
|------|-------------------|-------------------|
| 왼쪽 (L) | `left_wheel_steer` | `left_wheel_drive` |
| 오른쪽 (R) | `right_wheel_steer` | `right_wheel_drive` |
| 뒤 (B) | `rear_wheel_steer` | `rear_wheel_drive` |

> **참고**: [2]의 물리 설정에서는 전체 USD 경로(예: `/ffw_sg2_follower/world/left_wheel_steer`)를 사용합니다.
> 코드에서 ROS2로 보낼 때는 joint 이름만(예: `left_wheel_steer`) 사용합니다.

### 1.3 좌표계

ROS2 로봇 좌표계 규칙:

```
        X+ (전진)
        ↑
        |
Y+ ←---+--→ Y- (오른쪽)
(왼쪽)  |
        ↓
        X- (후진)

회전: 반시계 방향이 양수 (+ω), 시계 방향이 음수 (-ω)
```

- `vx > 0` → 로봇이 앞으로 감
- `vy > 0` → 로봇이 왼쪽으로 감
- `omega > 0` → 로봇이 반시계 방향으로 회전

---

## [2] IsaacSim 물리 설정 (매우 중요!)

swerve drive가 동작하려면 바퀴 joint의 **물리 속성**을 올바르게 설정해야 합니다.
이 설정이 틀리면 바퀴가 안 돌아가거나, 방향이 안 꺾이거나, 로봇이 제자리에서 떨립니다.

### 2.1 Drive Joint (구동 바퀴) 설정

Drive joint는 **속도로 제어**합니다. "초당 몇 라디안으로 돌아라"라고 명령합니다.

| 속성 | 값 | 왜 이렇게? |
|------|-----|-----------|
| **Stiffness** | `0` | 위치를 유지하려는 힘. 0으로 해야 속도 제어가 됨. 값이 있으면 바퀴가 원래 위치로 돌아가려고 함 |
| **Damping** | `1e10` | 속도를 따라가려는 힘. 크게 해야 명령한 속도대로 잘 돌아감. 1e10 = 100억, "사실상 무한대"로 동작하는 값 (1e5 이상이면 대부분 OK) |
| **Lower/Upper Limit** | 무제한 (코드에서 ±1e10으로 설정) | URDF 기본값은 ±1080도이지만, 바퀴가 한 방향으로 계속 돌아야 하므로 제한을 풀어야 함 |
| **Max Force** | Default 사용 | URDF effort 값이 적용됨. 시뮬레이션에서는 기본값으로 충분 |

> **Stiffness=0이 왜 중요한가?**
> IsaacSim의 Articulation Controller는 position과 velocity 명령을 동시에 받으면
> **position이 우선**됩니다. Stiffness가 0이 아니면 위치 제어가 활성화되어
> 바퀴가 position=0 위치로 돌아가려고 해서 **바퀴가 잠기는 현상**이 발생합니다.

### 2.2 Steer Joint (조향 바퀴) 설정

Steer joint는 **위치(각도)로 제어**합니다. "45도로 꺾어라"라고 명령합니다.

| 속성 | 값 | 왜 이렇게? |
|------|-----|-----------|
| **Stiffness** | `1e10` | 명령한 각도를 정확히 유지하려는 힘. 크게 해야 각도가 정확함 |
| **Damping** | `0` (떨리면 `1e4`~`1e5`) | 속도 감쇠. 0이면 가장 빠르게 목표 각도에 도달하지만, 진동(oscillation)이 발생할 수 있음 |
| **Lower Limit** | `-90°` | 바퀴가 -90도까지 꺾임 |
| **Upper Limit** | `+90°` | 바퀴가 +90도까지 꺾임 |

> **왜 ±90도 제한?**
> 코드에서 180도 플립 최적화를 하기 때문입니다 (아래 [4]에서 설명).
> 예: 150도로 꺾는 대신, -30도로 꺾고 바퀴를 반대로 굴리면 같은 효과입니다.
> 이렇게 하면 바퀴가 최대 90도만 꺾이면 되므로 더 빠르게 반응합니다.

### 2.3 물리 설정 적용 방법

IsaacSim의 `Window` → `Script Editor`를 열고, 기존 내용을 지운 뒤 아래 코드를
**전체 복사-붙여넣기**하고 **Run 버튼** (또는 `Ctrl+Enter`)을 클릭하세요:

> **joint 경로 구조 설명**: URDF에서 바퀴는 `steer joint → wheel link → drive joint`
> 순서의 체인으로 되어 있습니다. 그래서 USD 경로에서 drive가 steer 아래에 위치합니다.
> (예: `.../left_wheel_steer/left_wheel_drive`)
>
> **경고**: 아래 joint 경로는 URDF 임포트 시 생성된 기본 경로입니다.
> 본인의 USD 파일과 다를 수 있습니다. 오류가 나면 Stage 패널에서 경로를 확인하세요.

```python
import omni.usd
from pxr import UsdPhysics, Usd

stage = omni.usd.get_context().get_stage()

# 설정할 joint 목록
drive_joints = [
    '/ffw_sg2_follower/world/left_wheel_steer/left_wheel_drive',
    '/ffw_sg2_follower/world/right_wheel_steer/right_wheel_drive',
    '/ffw_sg2_follower/world/rear_wheel_steer/rear_wheel_drive',
]
steer_joints = [
    '/ffw_sg2_follower/world/left_wheel_steer',
    '/ffw_sg2_follower/world/right_wheel_steer',
    '/ffw_sg2_follower/world/rear_wheel_steer',
]

# Drive joints: 속도 제어용 설정
for path in drive_joints:
    prim = stage.GetPrimAtPath(path)
    drive = UsdPhysics.DriveAPI.Get(prim, "angular")
    drive.GetStiffnessAttr().Set(0.0)         # 위치 제어 비활성화
    drive.GetDampingAttr().Set(1e10)           # 속도 제어 강하게
    # 회전 제한 해제
    joint = UsdPhysics.RevoluteJoint(prim)
    joint.GetLowerLimitAttr().Set(-1e10)
    joint.GetUpperLimitAttr().Set(1e10)
    print(f"  Drive: {path} → Stiffness=0, Damping=1e10, Unlimited rotation")

# Steer joints: 위치(각도) 제어용 설정
for path in steer_joints:
    prim = stage.GetPrimAtPath(path)
    drive = UsdPhysics.DriveAPI.Get(prim, "angular")
    drive.GetStiffnessAttr().Set(1e10)         # 위치 제어 강하게
    drive.GetDampingAttr().Set(0.0)            # 속도 감쇠 없음
    # 회전 범위: -90° ~ +90°
    joint = UsdPhysics.RevoluteJoint(prim)
    joint.GetLowerLimitAttr().Set(-90.0)
    joint.GetUpperLimitAttr().Set(90.0)
    print(f"  Steer: {path} → Stiffness=1e10, Damping=0, Limit=±90°")

print("\nDone! 물리 설정 완료")
```

> **주의**: joint 경로가 다를 수 있습니다. IsaacSim Stage 패널에서
> 바퀴 joint를 클릭하고 하단 Property의 Path를 확인하세요.
> 일반적으로 `/ffw_sg2_follower/world/` 아래에 있습니다.

실행 후 **Ctrl+S로 USD 파일을 저장**하세요. 저장하지 않으면 다음에 열 때 설정이 사라집니다.

### 2.4 설정이 잘 됐는지 확인하는 방법

Script Editor 실행 결과에 오류 없이 아래처럼 나오면 성공입니다:

```
  Drive: /ffw_sg2_follower/world/left_wheel_steer/left_wheel_drive → Stiffness=0, Damping=1e10, Unlimited rotation
  Drive: /ffw_sg2_follower/world/right_wheel_steer/right_wheel_drive → Stiffness=0, Damping=1e10, Unlimited rotation
  Drive: /ffw_sg2_follower/world/rear_wheel_steer/rear_wheel_drive → Stiffness=0, Damping=1e10, Unlimited rotation
  Steer: /ffw_sg2_follower/world/left_wheel_steer → Stiffness=1e10, Damping=0, Limit=±90°
  Steer: /ffw_sg2_follower/world/right_wheel_steer → Stiffness=1e10, Damping=0, Limit=±90°
  Steer: /ffw_sg2_follower/world/rear_wheel_steer → Stiffness=1e10, Damping=0, Limit=±90°

Done! 물리 설정 완료
```

**GUI에서 수동 확인하려면:**
1. IsaacSim Stage 패널에서 `left_wheel_drive` joint를 클릭
2. 우측 Property 패널에서 `Drive` 섹션 찾기
3. `Stiffness = 0`, `Damping = 10000000000` (1e10) 확인

> **오류가 나면?** joint 경로가 다를 수 있습니다.
> Stage 패널에서 바퀴를 직접 찾아 경로를 확인하고, 스크립트의 경로를 수정하세요.
> 경로 확인법: Stage에서 joint 클릭 → 하단에 표시되는 `/ffw_sg2_follower/...` 경로 복사

### 2.5 바퀴 충돌 메쉬 설정 (SDF Mesh)

URDF 임포트 시 바퀴의 collision mesh는 기본적으로 **Convex Decomposition**으로 설정됩니다.
볼록 껍질 근사이므로 바퀴가 울퉁불퉁하게 굴러가는 현상이 발생합니다.

**SDF Mesh**로 변경하면 원본 메쉬 형상 그대로 충돌 판정하여 매끄럽게 굴러갑니다.

**설정 방법 (3개 바퀴 모두):**

1. Instanceable이 체크되어 있으면 **체크 해제** (개별 수정을 위해)
2. Stage에서 drive wheel의 collision mesh prim 선택:
   - `rear_wheel_drive_link/collisions/.../node_STL_BINARY_`
   - `left_wheel_drive_link/collisions/.../node_STL_BINARY_`
   - `right_wheel_drive_link/collisions/.../node_STL_BINARY_`
3. Properties → **Collider** → **Approximation** → **SDF Mesh**
4. **SDF Resolution**: `256` (기본값, 충분히 매끄러움)

> **왜 SDF Mesh인가?**
>
> | Approximation | 동적 바디 | 특징 |
> |---|:---:|---|
> | Convex Hull | O | 단일 볼록 껍질 — 둥근 바퀴가 다각형처럼 굴러감 |
> | Convex Decomposition | O | 여러 볼록 껍질 — 개선되지만 여전히 울퉁불퉁 |
> | Triangle Mesh (`none`) | **X** | 동적 바디에서 Convex Hull로 자동 fallback |
> | **SDF Mesh** | **O** | **원본 메쉬 수준 충돌 — 가장 매끄러움** |
>
> IsaacSim 공식 문서에서는 바퀴에 **실린더 프리미티브**를 권장하지만,
> SDF Mesh가 원본 형상을 그대로 사용하면서도 매끄러운 결과를 줍니다.

> **주의**: Instanceable을 다시 체크하면 수정이 되돌아갑니다. 체크 해제 상태를 유지하세요.

---

## [3] 역운동학 (Inverse Kinematics) 이해

### 3.1 문제 정의

우리가 하고 싶은 것:
> "로봇아, 앞으로 0.5m/s로 가면서 왼쪽으로 0.3m/s, 반시계 방향으로 0.2rad/s로 회전해"

이것을 각 바퀴에 대해:
> "왼쪽 바퀴: 35도 방향으로 꺾고, 초당 8라디안으로 굴려"

이렇게 변환하는 것이 **역운동학 (Inverse Kinematics, IK)**입니다.

```
입력: 로봇 전체의 움직임          출력: 각 바퀴의 개별 명령
┌─────────────────────┐      ┌──────────────────────────┐
│ vx = 0.5 m/s        │      │ 왼쪽: 각도=35°, 속도=8    │
│ vy = 0.3 m/s        │ ──→  │ 오른: 각도=-20°, 속도=6   │
│ omega = 0.2 rad/s   │      │ 뒤쪽: 각도=10°, 속도=7    │
└─────────────────────┘      └──────────────────────────┘
```

### 3.2 계산 원리

각 바퀴의 속도는 2가지 성분의 합입니다:

1. **병진 성분**: 로봇 전체가 이동하는 속도 (vx, vy) — 모든 바퀴에 동일
2. **회전 성분**: 로봇이 회전할 때 각 바퀴 위치에서 생기는 속도 — 바퀴 위치마다 다름

```python
# 바퀴 위치가 (rx, ry)이고, 로봇이 omega로 회전할 때:
# 회전 속도 = omega × r (외적)
# [0, 0, omega] × [rx, ry, 0] = [-omega*ry, omega*rx, 0]
vx_회전 = -omega * ry    # 마이너스는 외적에서 나옴
vy_회전 = omega * rx

# 최종 바퀴 속도 = 병진 + 회전
wheel_vx = vx + vx_회전
wheel_vy = vy + vy_회전
```

> 벡터 외적이 기억 안 나도 괜찮습니다 — 위 공식만 그대로 쓰면 됩니다.

> **직관적 이해**: 놀이공원의 회전목마를 생각하세요.
> 가운데(중심)에 가까울수록 느리게, 바깥쪽일수록 빠르게 돌아갑니다.
> 회전 성분은 바퀴가 로봇 중심에서 얼마나 떨어져 있는지에 따라 달라집니다.

### 3.3 바퀴 속도와 각도 계산

```python
import math

# 바퀴의 총 속도 벡터 (vx, vy)에서:
speed = math.sqrt(wheel_vx**2 + wheel_vy**2)   # 속도 크기 (피타고라스)
angle = math.atan2(wheel_vy, wheel_vx)          # 방향 (라디안)
```

> `math.atan2(y, x)`는 벡터의 방향을 라디안으로 돌려줍니다.
> `math.atan(y/x)`와 비슷하지만, 사분면을 정확히 구분해줍니다.
>
> **주의: `atan2`는 (y, x) 순서입니다!** 수학에서 tan = y/x이기 때문입니다.
> (x, y) 순서로 넣으면 방향이 90도 틀어지니 조심하세요.

### 3.4 선속도 → 각속도 변환

계산된 `speed`는 **m/s** (바퀴가 지면에서 이동하는 속도)입니다.
하지만 IsaacSim에 보내야 하는 것은 **rad/s** (바퀴가 도는 빠르기)입니다.

```python
wheel_radius = 0.033   # FFW-SG2 바퀴 반지름: 33mm = 0.033m
angular_vel = speed / wheel_radius  # m/s → rad/s
```

> **왜?** 바퀴 둘레 = 2π × 반지름. 1회전(2π rad)하면 둘레만큼 이동합니다.
> 따라서: 이동거리 = 각도 × 반지름 → 속도 = 각속도 × 반지름 → 각속도 = 속도 / 반지름

### 3.5 180도 플립 최적화

문제: `atan2`가 계산한 각도가 150도라면?
바퀴가 150도까지 꺾여야 하는데, steer는 ±90도 범위입니다.

해결: **150도로 꺾는 대신, -30도(= 150 - 180)로 꺾고 바퀴를 반대로 굴립니다.**

```python
if angle > math.pi / 2:       # 90도 초과
    angle -= math.pi           # 180도 빼기
    speed = -speed             # 반대로 굴리기
elif angle < -math.pi / 2:    # -90도 미만
    angle += math.pi           # 180도 더하기
    speed = -speed             # 반대로 굴리기
```

> **비유**: "뒤로 가고 싶을 때, 핸들을 180도 돌리는 것보다
> 그냥 후진 기어를 넣는 게 낫다" — 같은 원리입니다.

---

## [4] Swerve Controller 스크립트

### 4.1 전체 코드 (실제 동작하는 버전)

이 프로젝트를 git clone으로 받았다면 `scripts/swerve_controller.py`에 이미 포함되어 있습니다.
**아래 코드를 직접 만들 필요는 없습니다** — [5]에서 기존 스크립트를 바로 실행합니다.

아래는 동작 원리를 이해하기 위해 **주석을 상세히 추가한 설명 버전**입니다.
로직은 실제 스크립트와 동일하지만, 주석이 더 많습니다.

```python
#!/usr/bin/env python3
"""FFW-SG2 Swerve Drive Controller
/cmd_vel (Twist) → 역운동학 → /isaac_sim/joint_commands (JointState)

Steer: position control (목표 각도)
Drive: velocity control (목표 각속도)

사전 설정 필요 (IsaacSim Script Editor에서 실행):
- Drive wheel: Stiffness=0, Damping=MAX, Limit=Not Limited
- Steer wheel: Stiffness=MAX, Damping=0, Limit=-90~+90
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import math


class SwerveController(Node):
    def __init__(self):
        super().__init__('swerve_controller')

        # ── 바퀴 위치 정의 (로봇 좌표계, 미터 단위) ──
        # x: 전진 방향, y: 왼쪽 방향
        # 이 좌표는 로봇 설계 스펙 기준 (URDF joint origin과 약간 다를 수 있음)
        self.wheel_positions = [
            {
                'name': 'left',
                'x': 0.148, 'y': 0.236,              # 앞쪽 왼쪽
                'steer_joint': 'left_wheel_steer',
                'drive_joint': 'left_wheel_drive',
            },
            {
                'name': 'right',
                'x': 0.150, 'y': -0.273,             # 앞쪽 오른쪽
                'steer_joint': 'right_wheel_steer',
                'drive_joint': 'right_wheel_drive',
            },
            {
                'name': 'rear',
                'x': -0.278, 'y': -0.021,            # 뒤쪽 가운데
                'steer_joint': 'rear_wheel_steer',
                'drive_joint': 'rear_wheel_drive',
            }
        ]

        self.wheel_radius = 0.033  # 바퀴 반지름 33mm
        self.max_wheel_vel = 10.0  # 최대 바퀴 각속도 (rad/s), 안전 제한

        # ── ROS2 토픽 설정 ──
        # /isaac_sim/joint_commands: IsaacSim으로 명령을 보내는 토픽
        self.joint_publisher = self.create_publisher(
            JointState, '/isaac_sim/joint_commands', 10
        )

        # /cmd_vel: 로봇에게 "이렇게 움직여"라고 명령하는 표준 토픽
        # geometry_msgs/Twist 타입: linear(x,y,z) + angular(x,y,z)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )

        # ── 현재 속도 명령 저장 ──
        self.vx = 0.0      # 전진 속도 (m/s)
        self.vy = 0.0      # 횡방향 속도 (m/s)
        self.omega = 0.0   # 회전 각속도 (rad/s)
        self.last_cmd_time = self.get_clock().now()
        self.cmd_timeout = 0.5  # 0.5초 동안 명령이 없으면 정지

        # ── 50Hz 타이머: 0.02초마다 명령 발행 ──
        self.create_timer(0.02, self.publish_commands)
        self.get_logger().info('Swerve Drive Controller started.')

    def cmd_vel_callback(self, msg: Twist):
        """
        /cmd_vel 토픽에서 속도 명령을 받는 콜백.
        Twist 메시지의 linear.x, linear.y, angular.z를 사용.
        """
        self.vx = msg.linear.x      # 전진/후진
        self.vy = msg.linear.y      # 좌/우 횡이동
        self.omega = msg.angular.z  # 회전
        self.last_cmd_time = self.get_clock().now()

    def inverse_kinematics(self):
        """
        로봇 속도 (vx, vy, omega) → 각 바퀴의 (각속도, 조향각도) 변환.
        [3]에서 설명한 역운동학 계산을 수행합니다.
        """
        wheel_velocities = []
        wheel_angles = []

        for wheel in self.wheel_positions:
            rx = wheel['x']  # 바퀴의 X 위치 (전진 방향)
            ry = wheel['y']  # 바퀴의 Y 위치 (왼쪽 방향)

            # 회전으로 인한 속도 성분 (ω × r 외적)
            vx_rot = -self.omega * ry
            vy_rot = self.omega * rx

            # 총 속도 = 병진 + 회전
            wheel_vx = self.vx + vx_rot
            wheel_vy = self.vy + vy_rot

            # 속도 크기
            speed = math.sqrt(wheel_vx**2 + wheel_vy**2)

            if speed > 0.01:  # 거의 정지 상태가 아니면
                angle = math.atan2(wheel_vy, wheel_vx)

                # ── 180도 플립 최적화 ──
                # steer 범위를 -90~90도로 유지
                if angle > math.pi / 2:
                    angle -= math.pi
                    speed = -speed
                elif angle < -math.pi / 2:
                    angle += math.pi
                    speed = -speed
            else:
                angle = 0.0
                speed = 0.0

            # m/s → rad/s 변환 (IsaacSim은 각속도를 받음)
            angular_vel = speed / self.wheel_radius

            wheel_velocities.append(angular_vel)
            wheel_angles.append(angle)

        # ── 속도 제한 ──
        # 계산된 속도가 최대값을 넘으면, 비율을 유지하며 줄임
        max_vel = max(abs(v) for v in wheel_velocities) if wheel_velocities else 0
        if max_vel > self.max_wheel_vel:
            scale = self.max_wheel_vel / max_vel
            wheel_velocities = [v * scale for v in wheel_velocities]

        return wheel_velocities, wheel_angles

    def publish_commands(self):
        """
        50Hz로 호출되며, 역운동학 결과를 JointState 메시지로 발행.
        """
        # ── cmd_vel 타임아웃 ──
        # 0.5초 이상 새 명령이 없으면 정지 (안전장치)
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > self.cmd_timeout:
            self.vx = 0.0
            self.vy = 0.0
            self.omega = 0.0

        wheel_velocities, wheel_angles = self.inverse_kinematics()

        # ── JointState 메시지 생성 ──
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = []
        msg.position = []
        msg.velocity = []

        # Steer joints: 위치(각도) 제어
        for i, wheel in enumerate(self.wheel_positions):
            msg.name.append(wheel['steer_joint'])
            msg.position.append(wheel_angles[i])   # 목표 각도 (rad)
            msg.velocity.append(0.0)                # 속도는 사용 안 함

        # Drive joints: 속도 제어
        # ★ 핵심: position에 float('nan')을 넣어야 합니다!
        for i, wheel in enumerate(self.wheel_positions):
            msg.name.append(wheel['drive_joint'])
            msg.position.append(float('nan'))       # NaN = "위치 제어 안 함"
            msg.velocity.append(wheel_velocities[i])  # 목표 각속도 (rad/s)

        self.joint_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SwerveController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 4.2 float('nan') — 왜 필요한가?

이것은 이 가이드에서 **가장 중요한 트릭**입니다.

**문제**: JointState 메시지에서 `position`과 `velocity`를 동시에 보내면,
IsaacSim의 Articulation Controller는 **position을 우선 적용**합니다.

```python
# ❌ 이렇게 하면 안 됨!
msg.position.append(0.0)     # position=0 → "0도 위치로 가라"
msg.velocity.append(5.0)     # velocity=5 → "초당 5rad로 돌아라"
# 결과: position이 우선이라 바퀴가 0도에서 안 움직임 (잠김!)
```

```python
# ✅ 이렇게 해야 함!
msg.position.append(float('nan'))  # NaN = "위치 제어 무시해"
msg.velocity.append(5.0)           # velocity만 적용됨
# 결과: 바퀴가 초당 5rad로 정상 회전!
```

> **float('nan')이란?** Not a Number의 약자입니다.
> 숫자가 아닌 특별한 값으로, IsaacSim은 이 값을 "이 항목은 제어하지 않음"으로 해석합니다.
>
> 이 트릭은 `swerve_controller.py`에 **이미 적용되어 있습니다.**
> 직접 코드를 작성할 때 이 점을 반드시 기억하세요.

### 4.3 cmd_vel 타임아웃 — 왜 필요한가?

```python
elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
if elapsed > self.cmd_timeout:  # 0.5초 이상 명령 없으면
    self.vx = 0.0
    self.vy = 0.0
    self.omega = 0.0
```

**없으면 어떻게 되나?** 키보드에서 손을 떼도 마지막 명령이 계속 유지되어
로봇이 **멈추지 않고 계속 달립니다.** 타임아웃이 있으면 명령이 끊기면 자동 정지합니다.

---

## [5] 실행하기

### 5.1 사전 준비 체크리스트

1. **IsaacSim Docker 컨테이너 실행 중** (`docker-compose up`)
2. **IsaacSim에서 물리 설정 완료** ([2] 참고)
3. **USD 파일 저장** (Ctrl+S)
4. **IsaacSim에서 Play(▶) 클릭**

> **필요한 창**: 이 섹션을 끝까지 따라하려면 **3개의 창**이 필요합니다:
> 1. IsaacSim GUI (이미 열려 있음)
> 2. 터미널 1: swerve_controller.py 실행용
> 3. 터미널 2: 속도 명령 전송용 (ros2 topic pub 또는 teleop)
>
> 3개를 모두 화면에 배치해두세요.

### 5.2 Swerve Controller 실행

호스트 터미널을 열고 (Ubuntu에서 `Ctrl+Alt+T` 또는 터미널 앱 클릭):

```bash
# conda가 활성화되어 있으면 비활성화
# (ROS2 Jazzy = Python 3.12, conda base = Python 3.13 → 충돌)
# conda가 없으면 이 줄은 건너뛰세요 (command not found 나와도 무시 OK)
conda deactivate

# ROS2 환경 설정 (FastDDS UDP 통신 설정 포함)
source ~/ms_AIworker/scripts/ros2-bridge-env.sh

# swerve controller 실행
python3 ~/ms_AIworker/scripts/swerve_controller.py
```

정상이면 아래 메시지가 나옵니다:
```
[INFO] [swerve_controller]: Swerve Drive Controller started.
```

### 5.3 속도 명령 보내기 (테스트)

**새 터미널**을 엽니다 (기존 터미널에서 `Ctrl+Shift+T`로 새 탭, 또는 `Ctrl+Alt+T`로 새 창).
swerve_controller가 돌고 있는 터미널은 **그대로 두세요** — 끄면 제어가 안 됩니다!

```bash
# 새 터미널에서도 ROS2 환경을 다시 설정해야 합니다 (터미널마다 별도)
conda deactivate
source ~/ms_AIworker/scripts/ros2-bridge-env.sh
```

#### 전진 테스트 (가장 먼저 해보세요)

> `ros2 topic pub`은 `Ctrl+C`를 누를 때까지 **1초에 1번씩 계속** 명령을 보냅니다.
> 명령어가 길어서 `\`로 줄바꿈했지만, 한 줄로 이어 써도 됩니다.
> **복사-붙여넣기를 권장합니다** (중괄호/따옴표 오타 주의).

```bash
# 전진: vx=0.3 m/s (천천히)
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

> IsaacSim에서 로봇이 앞으로 움직이면 성공!
> `Ctrl+C`로 명령을 멈추면, 0.5초 후 자동 정지합니다.

#### 횡이동 테스트 (왼쪽으로)

```bash
# 왼쪽 이동: vy=0.3 m/s
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.0, y: 0.3, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

> 로봇이 옆으로 미끄러지듯 이동하면 swerve drive가 정상 동작하는 것입니다!

#### 제자리 회전 테스트

```bash
# 반시계 회전: omega=0.5 rad/s
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}'
```

#### 복합 이동 (전진 + 회전)

```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}'
```

### 5.4 키보드 텔레옵 (Keyboard Teleop)

매번 명령어를 치는 것보다 키보드로 조종하는 게 훨씬 편합니다.

> 다른 터미널에서 `swerve_controller.py`가 실행 중인지 확인하세요.
> teleop은 키보드 입력을 `/cmd_vel`로 변환하고, swerve_controller가 그것을 받아서 바퀴를 움직입니다.

```bash
# teleop_twist_keyboard 설치 (처음 한 번만, sudo 비밀번호 필요)
# 이미 설치되어 있으면 "already the newest version"이라고 나옴 — 정상
sudo apt-get install ros-jazzy-teleop-twist-keyboard

# 실행 (기본 모드: 전진/후진/회전만 가능)
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# ★ 횡이동(swerve의 핵심!)까지 사용하려면 holonomic 모드로 실행:
# ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p holonomic_mode:=true
```

> **holonomic 모드**: swerve drive의 핵심 장점인 **횡이동(linear.y)**을 키보드로 테스트하려면
> holonomic 모드가 필요합니다. 기본 모드는 전진/후진/회전만 보냅니다.
> holonomic 모드에서는 `j`/`l` 키가 좌/우 횡이동이 됩니다.
> 횡이동만 테스트하려면 [5.3]의 `ros2 topic pub` 명령어를 사용해도 됩니다.

화면에 조작법이 나옵니다:

```
Moving around:        (swerve는 횡이동도 가능!)
   u    i    o        i = 전진
   j    k    l        k = 정지 (중요!!)
   m    ,    .        , = 후진

q/z : 최대 속도 증가/감소   (선형+회전 동시)
w/x : 선형 속도만 증가/감소
e/c : 회전 속도만 증가/감소
```

> **팁**: 처음에는 `x`를 몇 번 눌러서 속도를 **아주 낮게** 설정한 뒤 테스트하세요.
> 속도가 너무 빠르면 로봇이 갑자기 돌진해서 시뮬레이션이 불안정해질 수 있습니다.

> **주의**: teleop_twist_keyboard는 기본적으로 `/cmd_vel`에 발행합니다.
> 이미 swerve_controller가 `/cmd_vel`을 구독하고 있으므로 바로 동작합니다.

---

## [6] 디버깅 & Troubleshooting

### 바퀴가 안 돌아감 (가장 흔한 문제)

**원인 1: 물리 설정을 안 했거나 저장하지 않음**
- [2]의 Script Editor 코드를 다시 실행
- `Ctrl+S`로 USD 저장

**원인 2: float('nan') 누락**
- `swerve_controller.py`에서 drive joint의 position이 `float('nan')`인지 확인
- `0.0`으로 되어 있으면 바퀴가 잠깁니다

**원인 3: IsaacSim이 Play 상태가 아님**
- 좌측 상단 ▶ 버튼 확인

### 바퀴 방향은 꺾이는데 구동이 안 됨

- Drive joint의 Stiffness가 0인지 확인 (0이 아니면 position 제어가 우선됨)
- Drive joint의 Damping이 충분히 큰지 확인 (1e10 권장)

### 로봇이 이상한 방향으로 움직임

- 바퀴 위치 좌표 (x, y) 확인. URDF와 일치해야 함
- 좌표계 혼동: X가 전진, Y가 왼쪽 (ROS2 규칙)

### 로봇이 심하게 떨림

- **Steer joint Damping이 0이면 진동 발생 가능** → Damping을 `1e4` ~ `1e5`로 올려보세요
  (Script Editor에서 [2.3] 코드의 Damping 값을 수정 후 다시 실행)
- `cmd_vel` 속도를 낮춰서 테스트 (vx=0.1부터 시작)
- Steer joint의 Stiffness가 너무 낮으면 떨릴 수 있음

### ros2 topic pub 했는데 반응 없음

```bash
# 1. 토픽이 살아있는지 확인
ros2 topic list | grep cmd_vel
# 정상: /cmd_vel 이 나와야 함

# 2. swerve_controller가 구독하고 있는지 확인
ros2 topic info /cmd_vel
# 정상: Publisher count: 1, Subscription count: 1

# 3. 메시지가 IsaacSim까지 가는지 확인
ros2 topic echo /isaac_sim/joint_commands --once
# 정상: name, position, velocity 배열이 출력됨
```

만약 `ros2 topic list`에 아무것도 안 나오면:
- `source ~/ms_AIworker/scripts/ros2-bridge-env.sh` 실행했는지 확인
- Docker 컨테이너가 실행 중인지 확인
- `conda deactivate` 했는지 확인 (Python 3.13 vs 3.12 충돌)

### 바퀴가 돌지만 로봇 자체가 안 움직임 (바닥에 고정됨)

- URDF를 **Fixed Base**로 잘못 임포트한 경우입니다
- Stage에서 `ffw_sg2_follower` 아래를 확인: `world`와 `base_link`가 **같은 레벨**이어야 정상
- `world` 아래에 `base_link`가 있으면 Fixed Base → Step 2에서 URDF 재임포트 필요
  (Fix Base Link: **체크 해제**)

### 토픽은 발행되는데 IsaacSim에서 반응 없음

- Step 5의 Action Graph (`/JointControlGraph`)가 있는지 확인
- Articulation Controller의 `targetPrim`이 `/ffw_sg2_follower/world`인지 확인
- Subscribe Joint State의 `topicName`이 `/isaac_sim/joint_commands`인지 확인

---

## [7] 이해도 확인 — 자주 묻는 질문

### Q: cmd_vel의 linear.z나 angular.x/y는 안 쓰나요?

A: 네, swerve drive는 **평면 이동**만 하므로 3개만 사용합니다:
- `linear.x` (전진/후진), `linear.y` (좌/우), `angular.z` (회전)
- z축 이동(날기)이나 x/y축 회전(기울기)은 바퀴 로봇에서는 의미 없습니다.

### Q: 50Hz는 왜 쓰나요? 더 빠르게/느리게 하면?

A: 50Hz(0.02초)는 **실시간 제어의 일반적인 주기**입니다.
- 너무 느리면 (예: 5Hz) 로봇이 끊기듯 움직임
- 너무 빠르면 (예: 1000Hz) CPU만 낭비하고 의미 없음
- 20~100Hz가 적당하며, IsaacSim의 물리 시뮬레이션 주기와 맞추면 좋음

### Q: wheel_radius가 0.033이면 바퀴 지름이 6.6cm인데, 이 값은 어디서?

A: URDF 파일 또는 로봇 제조사의 params.yaml에서 확인합니다.
이 값이 틀리면 명령한 속도와 실제 속도가 달라집니다.
(예: 반지름을 2배로 잘못 넣으면 로봇이 절반 속도로 움직임)

### Q: max_wheel_vel = 10.0은 어떻게 정한 건가요?

A: 모터의 물리적 한계입니다. 실제 모터가 낼 수 있는 최대 RPM에서 계산합니다.
10 rad/s ≈ 96 RPM. 이 값을 넘으면 모터가 따라가지 못하고 발열이나 오류가 생깁니다.
시뮬레이션에서는 안전 마진으로 보수적으로 잡는 것이 좋습니다.

### Q: 왜 Steer와 Drive를 한 메시지에 같이 보내나요? 따로 보내면 안 되나요?

A: 따로 보내면 **타이밍 문제**가 생깁니다.
"방향이 먼저 바뀌고 구동이 나중에 바뀌면" 잠깐 엉뚱한 방향으로 달릴 수 있습니다.
한 메시지에 같이 보내면 동시에 적용되어 자연스러운 동작이 됩니다.

### Q: `conda deactivate`를 안 하면 어떻게 되나요?

A: conda base 환경의 Python이 **3.13**인데, ROS2 Jazzy는 **Python 3.12**입니다.
Python 버전이 맞지 않으면 `rclpy` 같은 ROS2 패키지를 import할 수 없어서 에러가 납니다:
```
ModuleNotFoundError: No module named 'rclpy'
```
`conda deactivate` 후 시스템 Python 3.12가 사용되어야 정상 동작합니다.

### Q: `source ros2-bridge-env.sh`는 왜 터미널마다 해야 하나요?

A: 환경 변수(ROS2 설정, FastDDS 설정 등)는 **각 터미널 세션에 독립적**입니다.
터미널 A에서 source 해도 터미널 B에는 적용되지 않습니다.
ROS2 명령어를 쓸 모든 터미널에서 각각 source 해주세요.

### Q: 이 가이드의 스크립트를 직접 수정해도 되나요?

A: 네! `scripts/swerve_controller.py`를 수정해서 실험해보세요. 추천 실험:
- `self.max_wheel_vel`을 5.0으로 바꿔서 최대 속도 제한 체감
- `self.cmd_timeout`을 2.0으로 바꿔서 정지까지 걸리는 시간 변화 확인
- `self.wheel_radius`를 0.066으로 바꿔서 속도가 절반이 되는지 확인
- 수정 후 swerve_controller를 `Ctrl+C`로 끄고 다시 실행하면 적용됩니다

---

**Status**: COMPLETED
**이전**: [Step 5: Control Humanoids](05-control-humanoids.md)
**다음**: [Step 7: Navigation System](07-navigation-system.md)
