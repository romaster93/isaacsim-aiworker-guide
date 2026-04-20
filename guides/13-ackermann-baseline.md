# Step 13: Ackermann Baseline (논문 비교군)

## Overview

**목적**: ICROS 2026 논문 Table 1의 Ackermann baseline 측정용 가이드.

같은 FFW-SG2 swerve 하드웨어에서 원본 ApexNav 알고리즘(Ackermann kinodynamic)을 실행하여 Step 10의 swerve 홀로노믹 방식과 비교하는 베이스라인입니다.

**핵심 차이**:
- **경로 탐색**: Kinodynamic A* (yaw-rotated footprint) vs Holonomic A* (axis-aligned)
- **궤적 추종**: traj_server (unicycle MPC) vs swerve_path_follower.py (holonomic 직교 제어)
- **제약**: vy=0 (전진/회전만) vs 전방향(vx, vy, ω) 자유도

---

## Branch 전환

### Swerve 모드 (기본, Step 10/11)

```bash
cd ~/ApexNav_ROS2_wrapper
git checkout isaacsim-ffw-swerve
colcon build --packages-select exploration_manager trajectory_manager
```

### Ackermann 모드 (이 가이드)

```bash
cd ~/ApexNav_ackermann_worktree
git checkout isaacsim-ffw-ackermann
colcon build --packages-select exploration_manager trajectory_manager
```

**중요**: 두 디렉토리는 독립적인 ROS2 install 공간을 사용합니다. 같은 환경에서 동시에 띄우면 안 됩니다.

---

## 차이점 표

| 항목 | Swerve (Step 10/11) | Ackermann (이 가이드) |
|------|--------|-----------|
| 브랜치 | `isaacsim-ffw-swerve` | `isaacsim-ffw-ackermann` |
| 디렉토리 | `~/ApexNav_ROS2_wrapper` | `~/ApexNav_ackermann_worktree` |
| 경로 탐색기 | 2D 홀로노믹 A* (축 정렬 풋프린트) | Kinodynamic A* (yaw 회전 풋프린트) |
| 궤적 추종 | `swerve_path_follower.py` (vx, vy, ω) | `traj_server` (단일 트랙 MPC, vx + ω) |
| 헤딩 제어 | 독립 제어 | 경로 접선 추종 |
| 풋프린트 | 0.56m × 0.51m (축 정렬) | 0.56m × 0.51m (yaw 회전) |
| 하드웨어 | FFW-SG2swerve (vy 활용) | FFW-SG2 swerve (vy=0 제약) |
| max_vel | 0.35 m/s | 0.35 m/s (공정 비교) |

---

## 실행 (Ackermann 모드)

Step 10/11의 동일한 환경 설정 하에서 다음 터미널별 명령을 사용합니다.

### 터미널 1: IsaacSim

Step 10과 동일:

```bash
cd ~/ms_AIworker
docker-compose up --build
```

### 터미널 2: Swerve 제어기

Step 10과 동일:

```bash
conda deactivate
source ~/ms_AIworker/scripts/ros2-bridge-env.sh
python3 ~/ms_AIworker/scripts/swerve_controller.py
```

### 터미널 3: Nav2 Bridge

Step 10과 동일:

```bash
conda deactivate
source ~/ms_AIworker/scripts/ros2-bridge-env.sh
python3 ~/ms_AIworker/scripts/nav2_bridge.py
```

### 터미널 4: ApexNAV C++ Bridge

Step 9와 동일:

```bash
conda deactivate
source ~/ms_AIworker/scripts/ros2-bridge-env.sh
source ~/ApexNav_ROS2_wrapper/install/setup.bash
python3 ~/ms_AIworker/scripts/isaacsim_apexnav_bridge.py
```

### 터미널 5-8: VLM 서버 (4개)

Step 11과 동일. 각 터미널에서:

```bash
conda activate apexnav_ros2
python3 /home/cho/ApexNav/VLM_server.py --port 5005  # 터미널 5: 포트 5005
python3 /home/cho/ApexNav/VLM_server.py --port 5006  # 터미널 6: 포트 5006
python3 /home/cho/ApexNav/VLM_server.py --port 5007  # 터미널 7: 포트 5007
python3 /home/cho/ApexNav/VLM_server.py --port 5008  # 터미널 8: 포트 5008
```

### 터미널 9: ApexNAV 플래너 — **Ackermann 모드 변경**

```bash
conda deactivate
source ~/ms_AIworker/scripts/ros2-bridge-env.sh
source ~/ApexNav_ackermann_worktree/install/setup.bash
ros2 launch exploration_manager exploration_traj_ackermann.launch.py
```

**이전 Step 10**: `~/ApexNav_ROS2_wrapper/install/setup.bash`와 `exploration_traj.launch.py`

**이번**: `~/ApexNav_ackermann_worktree/install/setup.bash`와 `exploration_traj_ackermann.launch.py`

### 터미널 10: VLM 노드

Step 11과 동일:

```bash
conda deactivate
source ~/ms_AIworker/scripts/ros2-bridge-env.sh
source ~/ApexNav_ROS2_wrapper/install/setup.bash
python3 ~/ms_AIworker/scripts/isaacsim_realworld_node.py
```

### 터미널 11: (선택) 진단

```bash
conda deactivate
source ~/ms_AIworker/scripts/ros2-bridge-env.sh
python3 ~/ms_AIworker/scripts/vlm_diagnostic.py
```

### 터미널 12: 목표 물체 지정

Step 11과 동일:

```bash
conda deactivate
source ~/ms_AIworker/scripts/ros2-bridge-env.sh
python3 ~/ms_AIworker/scripts/target_label_publisher.py --target chair
```

### 터미널 13: **경로 추종기 비활성화**

**Ackermann 모드에서는 swerve_path_follower.py를 실행하지 않습니다.**

traj_server가 `/planning/trajectory` 토픽을 구독하여 직접 `/cmd_vel` (geometry_msgs/Twist)를 발행하기 때문입니다.

### 터미널 14: 메트릭 수집 — **신규**

```bash
conda deactivate
source ~/ms_AIworker/scripts/ros2-bridge-env.sh
source ~/ApexNav_ackermann_worktree/install/setup.bash
python3 ~/ms_AIworker/scripts/exploration_metrics.py --mode ackermann --target chair --timeout 300
```

---

## 메트릭 노드 사용법

### CSV 출력 경로

메트릭 실행 완료 시 다음 경로에 결과를 저장합니다:

```
~/ms_AIworker/results/metrics_ackermann_YYYYMMDD_HHMMSS.csv
```

(예: `metrics_ackermann_20260420_143022.csv`)

### 컬럼 설명

| 컬럼 | 설명 |
|------|------|
| `success` | 성공 여부 (0 또는 1). 클라우드 중심 거리 < 1.0m 도달 시 1 |
| `path_length` | 누적 이동 거리 (미터) |
| `optimal_length` | 시작점 → 목표 클라우드 중심 직선거리 (미터) |
| `spl` | Success weighted by Path Length = `success × optimal / max(path, optimal)` |
| `duration_s` | 전체 wall-clock 실행 시간 (초) |
| `motion_time_s` | **실제 이동 시간만** 누적 (선형 속도 \|linear_vel\| > 0.05 m/s 구간) |
| `idle_time_s` | 비활동 시간 = `duration_s - motion_time_s` (계획/대기) |
| `mean_speed` | 평균 이동 속도 = `path_length / motion_time_s` (m/s) |

### 왜 motion_time을 따로 측정하나?

알고리즘 비교의 **공정성** 때문입니다. Ackermann과 Swerve는 계산 복잡도가 다르므로:
- **duration_s**: 컴퓨터 성능(계획 시간, 시뮬 속도)의 영향 포함
- **motion_time_s**: 순수 알고리즘 효율(경로 품질, 추종 정확도)만 측정
- **mean_speed**: 실제 이동 효율 = `(총 거리) / (실제 움직인 시간)` — 대기/계획 시간 제외

### 사용 예시

```bash
# 1회 실행
python3 ~/ms_AIworker/scripts/exploration_metrics.py --mode ackermann --target chair --timeout 300

# 여러 회 반복 실행 → 같은 CSV에 append
# (결과 분석은 pandas로 수행)
```

### 종료 조건

- **Success** (클라우드 중심 < 1.0m): 자동 종료, `success=1` 기록
- **Timeout** (300초 초과): 자동 종료, `success=0` 기록
- **Ctrl+C**: 수동 중단, 현재 실행 상태 CSV에 기록

### 결과 비교 예시

다음은 **가짜 숫자**입니다. 실제 실행 후 업데이트하세요:

| 모드 | SR (Success Rate) | SPL | duration_s | motion_time_s | mean_speed |
|------|-------|------|-----------|---------------|------------|
| Ackermann | 0.70 | 0.52 | 180 | 95 | 0.22 m/s |
| Swerve | 0.90 | 0.71 | 120 | 78 | 0.28 m/s |

**해석**:
- Swerve가 더 높은 성공률(SR), 더 짧은 경로(SPL)를 달성
- Ackermann이 더 느린 이동(mean_speed 0.22 vs 0.28)
- duration 차이는 계획 복잡도, motion_time 차이는 경로 품질 반영

---

## Troubleshooting

### traj_server crash: "can't subtract times with different time sources"

**원인**: ROS2 시뮬레이션 시간 설정 불일치.

**해결**:

```bash
# use_sim_time: true 확인
# planning_param_ffw_ackermann.yaml에서 use_sim_time이 true인지 검증
```

### /cmd_vel Hz가 0 (속도 명령 안 옴)

**확인 사항**:

```bash
# 터미널에서:
ros2 topic hz /cmd_vel

# traj_server 로그 확인:
# 터미널 9에서 LaunchDescription 에러나 ExitCode != 0 있는지 확인

# /planning/trajectory 발행 여부:
ros2 topic echo /planning/trajectory
```

**일반적 원인**:
- 탐색 노드가 경로를 계획하지 못함 (시작점이 맵 범위 밖)
- traj_server 활성화 못 함 (colcon build 실패)

### 좁은 통로를 통과하지 못함

**Ackermann의 알려진 한계**:

Kinodynamic A*는 yaw 회전 풋프린트를 사용하므로, 축 정렬(axis-aligned) 풋프린트보다 통로가 좁을 때 경로를 찾기 어렵습니다.

예: 폭 0.6m 통로 → Swerve는 0.56m 축 정렬으로 통과, Ackermann은 0.56m yaw 회전으로 불가능.

이는 논문의 설계 트레이드오프입니다 — **kinodynamic 제약(vy=0)의 대가로 좁은 공간 통과 어려움**.

### exploration_metrics.py import 에러 (plan_env.msg, etc.)

**해결**:

```bash
# Ackermann worktree setup.bash source 확인:
source ~/ApexNav_ackermann_worktree/install/setup.bash

# colcon build 재실행:
cd ~/ApexNav_ackermann_worktree
colcon build --packages-select exploration_manager
```

---

## Status

### 빌드 검증
**완료** (Phase 4): `colcon build` 성공, 모든 C++ 노드 컴파일 OK.

### 드라이런 검증
**완료** (Phase 5): LaunchDescription 10 args 파싱 OK, traj_server ExecuteProcess 활성화 OK.

### 동작 검증
**미완료**: 사용자가 IsaacSim 환경에서 직접 실행하여 다음을 확인하세요:
- 터미널 9: LaunchDescription 에러 없음
- 터미널 14: CSV 생성, success 기록 가능
- RViz: `/planning/trajectory` 시각화 가능

### 메트릭 수집
**메서드**: `exploration_metrics.py` 사용 (motion_time 포함).

---

## 참고

**이전**: [Step 12: InteriorAgent 데이터셋](12-interioragent-dataset.md)

**다음**: 메트릭 비교 분석 (SR, SPL, motion_time 기반)
