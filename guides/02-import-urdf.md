# Step 2: Import URDF

## Overview
ROBOTIS FFW-SG2 Mobility AI Worker의 URDF 파일을 IsaacSim으로 임포트합니다.
- 로봇 이름: ROBOTIS FFW-SG2 Mobility AI Worker
- URDF 소스: https://github.com/ROBOTIS-GIT/ai_worker.git
- URDF 파일: `ffw_sg2_follower.urdf`
- 사전 준비된 리소스: `/home/cho/ms_AIworker/isaacsim_ai_worker/`

## Prerequisites
- [x] IsaacSim 5.1.0 설치 완료 (Step 1)
- [x] URDF 및 메시 파일 준비됨

## 방법 A: URDF 직접 임포트

### Step 2.1: URDF 파일 확인

```bash
# URDF 파일 위치 확인
ls -la /home/cho/ms_AIworker/isaacsim_ai_worker/usd_ai_worker/ffw_sg2_follower.urdf
```

> **참고**: URDF는 `package://ffw_description/meshes/...` 경로로 메시를 참조합니다.
> 우리가 사용하는 파일에는 메시가 같은 디렉토리 구조에 포함되어 있습니다.

### Step 2.2: IsaacSim에서 URDF 임포트

1. **IsaacSim이 실행된 상태에서:**
   - 메뉴: `File` → `Import`
   - 파일 선택: `/home/cho/ms_AIworker/isaacsim_ai_worker/usd_ai_worker/ffw_sg2_follower.urdf`

2. **Import Settings 설정 (IsaacSim 5.1.0 기준)**

   | 섹션 | 항목 | 설정값 | 비고 |
   |------|------|--------|------|
   | **Model** | Mode | **Create in Stage** | Referenced Model은 별도 USD 참조용 |
   | **USD Output** | Path | Same as imported model | 기본값 유지 |
   | **Links** | Base Type | **Moveable Base** | Static Base 아님! 로봇이 이동해야 함 |
   | | Default Density | **0.0** kg/m3 | URDF에 정의된 질량 사용 |
   | **Joints and Drives** | Ignore Mimic | **체크하지 않음** | Gripper mimic joint 유지 |
   | **Joint Configuration** | Strength Mode | **Stiffness** | 정밀 위치 제어에 적합 |
   | | Drive Type | **Force** | Acceleration 아님 |
   | **Colliders** | Source | **Collision from Visuals** | |
   | | Collider Type | **Convex Hull** | Decomposition보다 가벼움 |
   | | Replace Cylinders with Capsules | **체크하지 않음** | |
   | | Allow Self Collision | **체크하지 않음** | |

   > **Joint Configuration 참고 - Stiffness vs Natural Frequency:**
   > - **Stiffness**: 조인트 강성을 직접 수치(N·m/rad)로 설정. 직관적이고 물리적으로 명확
   > - **Natural Frequency**: 고유진동수(Hz)와 감쇠비로 간접 설정. 질량에 따라 자동 계산
   > - AI Worker처럼 팔을 정밀 제어하는 로봇은 **Stiffness**가 적합
   > - 나중에 Python 스크립트로 개별 조인트 stiffness/damping을 재설정할 수 있음

3. **Import 클릭**

### Step 2.3: Ground Plane 추가

임포트 직후 Play(▶)를 누르면 로봇이 **바닥 없이 아래로 떨어집니다**. 반드시 바닥을 먼저 추가하세요:

1. 메뉴: `Create` → `Physics` → `Ground Plane`
2. Stage에 GroundPlane이 추가됨
3. 이제 Play(▶) 하면 로봇이 바닥 위에 서있음

### Step 2.4: 임포트 결과 확인

임포트 후 Viewport에 로봇이 나타납니다:

1. **로봇 구조** - Stage 패널(우측)에서 로봇의 전체 계층 구조 확인
   - 주요 링크: base_link, lift_link, arm_base_link
   - 양팔: arm_l_link1~7, arm_r_link1~7
   - 그리퍼: gripper_l_*, gripper_r_*
   - 휠: left_wheel_steer_link, right_wheel_steer_link, rear_wheel_steer_link
   - 카메라: camera_l_link, camera_r_link

2. **메시 및 시각화**
   - 로봇 형태가 보이지만 **색상이 제대로 반영되지 않을 수 있음** (URDF 임포트의 일반적인 특성)
   - 색상/재질은 IsaacSim 환경에서 별도 조정 필요

3. **물리 속성**
   - Collider: 각 링크에 Convex Hull 충돌체 적용됨
   - 질량/관성: URDF에 정의된 값 사용 (Default Density 0.0이므로)

---

## 방법 B: 사전 구성된 USD 파일 사용

URDF 임포트 대신, 이미 구성된 USD 파일을 직접 열 수 있습니다:

열기 방법 (IsaacSim 5.1.0에서는 `File → Open`이 없음):
1. 하단 **Content 브라우저**에서 경로 탐색:
   `/home/cho/ms_AIworker/isaacsim_ai_worker/usd_ai_worker/ffw_sg2_follower/`
2. `ffw_sg2_follower.usd` **더블클릭**
3. "Would you like to save this stage?" → **Don't Save** 클릭

이 USD 파일은 다음을 포함합니다:
- 로봇 베이스 모델 (Base)
- 물리 속성 설정 (Physics)
- 센서 구성 (Sensors)

> **방법 A vs B 비교 결과 및 주의사항**:
> - 겉으로 보기에는 거의 동일함 (둘 다 흰색/무색상).
> - **방법 B의 치명적 한계**: USD 파일을 직접 열면 `ffw_sg2_follower`가 **Stage root**가 됨.
>   이 경우 환경(Warehouse 등)이나 센서를 Stage root 레벨에 추가할 수 없고,
>   모든 것이 ffw_sg2_follower 하위로 들어가버림.
> - **방법 A 권장**: URDF 임포트 시 `World`가 root이므로 환경/센서를 자유롭게 추가 가능.
>   이후 Step 3(센서), Step 4 이후 작업을 위해서는 **반드시 방법 A를 사용**할 것.
> - USD(B)는 로봇 모델만 빠르게 확인할 때만 사용.

## Troubleshooting

### URDF 임포트 시 메시 경로 에러
URDF가 `package://ffw_description/meshes/...` 경로를 참조합니다.
메시를 찾지 못하면 URDF 파일 내 경로를 절대 경로로 수정하거나, 사전 구성된 USD 파일(방법 B)을 사용하세요.

### 색상이 표시되지 않음
URDF 임포트 시 색상이 반영되지 않는 것은 정상입니다.
IsaacSim에서 Material을 별도로 추가하거나, 사전 구성된 USD 파일을 사용하세요.

### 로봇이 Scene에서 떨어지거나 움직이지 않음
- **Moveable Base**로 설정되었는지 확인
- Play(▶) 버튼 누르기 전에 로봇 위치 확인

---
**Status**: COMPLETED (방법 A 완료)
**Next**: [Step 3: Import Sensors](03-import-sensors.md)
