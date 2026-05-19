# Step 14: 실하드웨어 네트워크 토폴로지 + 무선 전송 설계

## Overview

Step 00~13은 IsaacSim 위에서 검증한 시뮬레이션 가이드였습니다.
Step 14부터는 **실제 하드웨어(AMR + Jetson Orin + Workstation)** 위에서 같은 파이프라인을 동작시키기 위한 가이드입니다.

이 단계의 첫 가이드는 **코드를 짜기 전에 결정해야 하는 것**들을 다룹니다:

- 세 장비를 어떻게 연결할 것인지 (토폴로지)
- 무선 구간에서 카메라 데이터를 어떻게 흘릴 것인지 (전송 스택)
- VLM/ApexNAV 같은 무거운 노드를 어디에 배치할 것인지 (연산 분담)

여기서 정한 결정이 Step 15 이후(`15-realsense-bringup.md`, `16-wireless-transport.md`, …)의 구현 방향을 정합니다.

> **시뮬 가이드와의 관계**: Step 00~13은 그대로 둡니다.
> 시뮬에서 검증된 ApexNAV C++ wrapper, VLM 노드, swerve_path_follower 같은 자산은
> "Workstation 내부에서는 ROS 토픽으로 그대로 사용"이라는 원칙으로 재활용합니다.

---

## 대상 플랫폼

| 항목 | 값 |
|------|------|
| 로봇 | 자체 제작 원통형 2-wheel (Differential drive) |
| 모터 제어 | 별도 MCU가 처리 — 상위는 `/cmd_vel` 또는 goal만 송신 |
| MCU 출력 | odom, localization pose (Ethernet) |
| 카메라 | RealSense D435i (RGB-D + 6축 IMU) |
| LiDAR | 없음 |
| 컴퓨팅 | Jetson Orin (로봇 탑재) + Workstation (원격) |

> **시뮬(FFW-SG2 swerve)과의 가장 큰 차이**
> - swerve IK는 MCU가 처리 → `scripts/swerve_controller.py` 사용 안 함
> - odom 적분도 MCU가 처리 → `scripts/nav2_bridge.py` odom 재발행 사용 안 함
> - LiDAR 없음 → `scripts/laser_merger.py` 사용 안 함
> - depth 한 종류로 ApexNAV가 동작해야 함

---

## 네트워크 토폴로지

### 물리 연결

```
[AMR MCU]                       (로봇 본체)
   │
   │ Ethernet (유선, 동일 본체 내부)
   │
[Jetson Orin] ── RealSense D435i (USB 3)
   │
   │ Wi-Fi (이동성 필수, 5GHz 권장)
   │
[Wi-Fi 라우터/AP]
   │
   │ Ethernet 또는 Wi-Fi (정적, 책상 위)
   │
[Workstation]
```

### 각 구간의 역할

| 구간 | 매체 | 주요 트래픽 | 미들웨어 |
|------|------|------|------|
| AMR ↔ Jetson | Ethernet (유선) | `/cmd_vel`, `/odom`, pose | TBD (MCU 인터페이스 스펙 확인 필요) |
| Jetson ↔ Workstation | Wi-Fi (무선) | RGB, Depth, VLM 결과 | **non-DDS** (이 가이드의 핵심) |
| Workstation 내부 | 로컬 | ApexNAV, VLM, RViz | DDS (시뮬과 동일) |

> **왜 Wi-Fi 구간만 DDS를 피하는가?**
> Workstation 내부의 ApexNAV/VLM 노드끼리는 시뮬에서 이미 DDS로 검증되었습니다.
> 이걸 굳이 바꾸면 시뮬 자산을 재작성해야 합니다. 문제는 오직 무선 구간이므로 거기만 갈아끼웁니다.

---

## DDS 회피 범위 결정

### 회피 범위: **무선 구간만**

| 구간 | DDS 사용 | 이유 |
|------|----------|------|
| AMR ↔ Jetson | 사용 가능 (유선) | 유선 Ethernet은 DDS 패킷 손실이 거의 없음. MCU 측 인터페이스에 따라 결정. |
| Jetson ↔ Workstation | **사용 안 함** | Wi-Fi에서 raw 이미지를 DDS로 보내면 대역폭 초과 + fragment 손실로 메시지 통째 드랍 |
| Workstation 내부 | 사용 | 시뮬 자산(ApexNAV C++ wrapper, VLM 노드) 그대로 재사용 |

### 핵심 원칙

- Jetson 측: 센서를 ROS 토픽으로 받은 뒤 → non-DDS 전송으로 송출
- Workstation 측: non-DDS 수신 → ROS 토픽으로 republish → 기존 노드들이 그대로 구독

양 끝점에서 "ROS ↔ non-DDS"로 변환하는 어댑터 노드 두 개만 만들면 됩니다.

---

## 무선 구간 대역폭 산정

### Raw로 보낼 때 (DDS, 압축 없음)

| 토픽 | 해상도 | 인코딩 | 30Hz 대역폭 |
|------|--------|--------|--------------|
| RGB | 1280×720 | rgb8 (3 B/px) | ~80 MB/s ≈ 640 Mbps |
| Depth | 1280×720 | 16UC1 (2 B/px) | ~55 MB/s ≈ 440 Mbps |
| **합계** | | | **~1 Gbps** |

5GHz Wi-Fi 6 실효 대역폭은 보통 300–600 Mbps입니다. **raw로는 불가능**합니다.
게다가 DDS는 fragment 하나만 손실되어도 메시지 전체가 드랍되므로 무선에서는 더 취약합니다.

### 목표 대역폭 (이 가이드의 설계)

| 토픽 | 해상도 | 압축 | Hz | 대역폭 |
|------|--------|------|-----|--------|
| RGB | 1280×720 | H.265 (NVENC 하드웨어 인코딩) | 15 | ~5 Mbps |
| Depth | 640×480 | zstd 또는 RVL (무손실) | 15 | ~20–30 Mbps |
| **합계** | | | | **~30 Mbps** |

Wi-Fi 실효 대역폭의 10% 미만으로 안정 구간에서 동작합니다.

### Depth에 H.264/265를 쓰지 않는 이유

depth 16-bit 값을 H.264/265로 압축하면:
- 인접 픽셀 상관성이 RGB와 달라서 압축률이 나쁨
- 양자화 노이즈가 거리 오차로 직결됨 (1 LSB가 1 mm)
- ApexNAV의 SDF 맵 품질이 망가짐

→ depth는 **무손실**(zstd/PNG/RVL) 압축만 사용합니다.

### Hz를 30 → 15로 낮추는 이유

- VLM 추론 실측 한계가 보통 5–10Hz
- ApexNAV C++ planner도 시뮬에서 ~10Hz로 동작 검증됨
- 15Hz면 두 곳 모두 충분하고, 대역폭이 절반으로 줄어듭니다

---

## 전송 스택: GStreamer + 커스텀 UDP

### 선택 이유

| 후보 | 채택 | 이유 |
|------|------|------|
| **GStreamer (RGB) + 커스텀 UDP (Depth)** | **채택** | Jetson NVENC 하드웨어 인코더 최대 활용, latency 최소, 영상/depth 각각 최적 압축 |
| Zenoh | 불채택 | 단일 스택은 깔끔하나 NVENC 미사용(CPU 압축), 무선 latency 우위 없음 |
| ROS image_transport (`compressed`/`compressedDepth`) | 불채택 | 결국 DDS 위에서 동작 → 무선 fragment 손실 문제 그대로 |

### 데이터 흐름

```
[Jetson]
  RealSense D435i (USB3)
    │
    ├─ realsense2_camera_node
    │     ↓ ROS topic /camera/color/image_raw
    │     ↓ ROS topic /camera/depth/image_rect_raw
    │
    ├─ RGB 송신 노드:
    │     ROS subscribe → nvv4l2h265enc (NVENC) → RTP/UDP →
    │                                                    │
    └─ Depth 송신 노드:                                  │
          ROS subscribe → zstd compress → UDP socket →   │
                                                         │
                                              ┌──────────┘ Wi-Fi
                                              ↓
[Workstation]
  ├─ RGB 수신 노드:
  │     UDP/RTP → nvv4l2decoder → ROS publish /camera/color/image_raw
  │
  ├─ Depth 수신 노드:
  │     UDP → zstd decompress → ROS publish /camera/depth/image_rect_raw
  │
  └─ (시뮬과 동일한 ApexNAV/VLM 파이프라인이 위 토픽을 구독)
```

### 토픽 이름 매핑

| 시뮬 | 실기 |
|------|------|
| `/zed_mini/rgb` | `/camera/color/image_raw` |
| `/zed_mini/depth` | `/camera/depth/image_rect_raw` |
| `/zed_mini/camera_info` | `/camera/color/camera_info` |
| `/scan` (병합 LiDAR) | (없음 — depth만 사용) |
| `/cmd_vel` | `/cmd_vel` (그대로) |
| `/odom` (nav2_bridge가 재발행) | `/odom` (MCU가 직접 발행) |

> **camera_info 전송 방법**
> camera_info는 메시지 크기가 작고 거의 정적인 값(K, D, frame_id 등)이라 매 프레임 송신할 필요 없습니다.
> Workstation 측에 yaml로 미리 두거나 시작 시 1회만 UDP로 전송합니다.

---

## VLM / ApexNAV 배치 결정

### 분담

| 노드 | 위치 | 이유 |
|------|------|------|
| RealSense 드라이버 | Jetson | 카메라가 Jetson에 USB로 연결됨 (물리 제약) |
| MCU 통신 | Jetson | AMR이 Jetson과 유선 Ethernet으로 연결됨 (물리 제약) |
| RGB/Depth 송신 어댑터 | Jetson | 무선 송출 시작점 |
| RGB/Depth 수신 어댑터 | Workstation | 무선 수신 시작점 |
| **ApexNAV C++ planner** | **Workstation** | 시뮬과 동일 환경 재현, Workstation GPU/CPU 활용 |
| **VLM 추론** | **Workstation** | GroundingDINO/SAM 등 무거운 모델, Workstation GPU 필요 |
| RViz 시각화 | Workstation | 사용자가 Workstation 앞에 앉아 있음 |

### Jetson에서 ApexNAV/VLM을 돌리지 않는 이유

- Jetson Orin에서도 둘 다 동작은 가능
- 다만 시뮬에서 검증된 환경이 Workstation 한 머신이고, 같은 환경을 그대로 재현하는 게 디버깅이 쉬움
- depth/RGB 대역폭은 위 설계로 Wi-Fi에서 견딜 수 있다는 게 확인됨 → Workstation으로 옮길 이유가 충분

---

## ApexNAV 실하드웨어 적용 시 주의

시뮬에서 발견된 위험 요소 두 가지는 실기에서도 다시 점검이 필요합니다.

### 1. depth 분포 차이 문제 재현 가능성

시뮬 IsaacSim World2에서는 max-range 픽셀 비율이 66%여서 ApexNAV의 outlier filter를 통과해 가짜 벽이 형성됐습니다 (참고: `guides/09-apexnav-bridge.md`, `project_apexnav_isaacsim_debug.md`).

실하드웨어에서 RealSense D435i의 depth 분포는:
- 야외/개방 공간 → max-range(8–10m) 픽셀 비율 높음 (시뮬 World2와 유사)
- 실내 → max-range 비율 낮음 (Habitat HM3D와 유사)

→ Step 15에서 **depth 통계를 먼저 측정**하고, Step 18(ApexNAV 통합)에서 ApexNAV C++ 측 파라미터(`filter_min_height`, `filter_max_height`, `max_dist` 등)를 D435i 기준으로 재조정합니다.

### 2. 카메라 내부 파라미터 (fx, fy)

시뮬 ZED Mini의 fx/fy=245.33은 D435i에 적용하면 안 됩니다.
RealSense의 `camera_info` 토픽에서 K 행렬을 실측해야 합니다 (시뮬 가이드에서 학습한 교훈: hfov 계산값 신뢰 금지).

### 3. 카메라 마운팅 위치

base_link 기준 카메라 오프셋(x, z, q_cam)은 로봇 본체 실측이 필요합니다.
시뮬 값(x=0.066, z=1.08m)은 FFW-SG2 기준이므로 새 플랫폼에서 그대로 쓰면 안 됩니다.

---

## 후속 가이드 로드맵

| 가이드 | 내용 | 실기 필요 |
|--------|------|----------|
| 14 (이 문서) | 토폴로지/전송 설계 | 불필요 |
| 15 `realsense-bringup.md` | RealSense D435i 단독 검증, fx/fy/depth 통계 측정 | D435i 1대 |
| 16 `wireless-transport.md` | GStreamer NVENC + UDP zstd 어댑터 노드 구현 | Jetson + Workstation |
| 17 `mcu-interface.md` | MCU ↔ Jetson Ethernet 프로토콜, `/cmd_vel`/odom/pose 연동 | AMR + Jetson |
| 18 `apexnav-realhw.md` | 실기 depth-only ApexNAV 통합, 파라미터 재조정 | 전체 |

> Step 17은 MCU 인터페이스 스펙(프로토콜, 토픽 이름, Hz, frame_id)이 확인된 시점에 작성합니다.
> 그 전에 Step 15/16은 D435i + Jetson + Workstation만 있으면 진행 가능합니다.

---

## 결정 요약 (한 화면)

```
토폴로지        : AMR ─Eth─ Jetson ─Wi-Fi─ Workstation
DDS 회피 범위   : Wi-Fi 구간만 (양 끝은 ROS 토픽 그대로)
RGB 전송        : GStreamer H.265 NVENC over UDP/RTP
Depth 전송      : zstd 무손실 + UDP
목표 해상도/Hz  : RGB 1280×720@15Hz, Depth 640×480@15Hz
목표 대역폭     : ~30 Mbps (Wi-Fi 5GHz Wi-Fi 6 실효의 10% 미만)
ApexNAV 위치    : Workstation
VLM 위치        : Workstation
RealSense 위치  : Jetson (USB3)
MCU 위치        : AMR (Jetson과 Ethernet)
```

다음 가이드: [Step 15: RealSense D435i Bringup](./15-realsense-bringup.md)
