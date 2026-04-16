---
description: Run VLM pipeline diagnostic and summarize health (Hz, sync, cloud frame/centroid)
---

## Task

VLM 파이프라인 전체 상태를 `scripts/vlm_diagnostic.py` 로 측정하고 결과를 요약해. 사용자가 지정한 시간(초) 동안 수집, 기본 30초.

## Execution

인자 `$ARGUMENTS` 가 숫자면 그 초 동안 수집, 아니면 30초.

```bash
DURATION=${ARGUMENTS:-30}
cd /home/cho/ms_AIworker
source scripts/ros2-bridge-env.sh
source ~/ApexNav_ROS2_wrapper/install/setup.bash
timeout "$DURATION" python3 -u scripts/vlm_diagnostic.py 2>&1
```

## Output analysis

수집 결과에서 다음을 판정하고 보고:

| 체크 | Pass 기준 | 실패 시 원인 힌트 |
|------|-----------|------------------|
| rgb/depth/sensor_pose Hz | 25~30Hz | IsaacSim Play OFF 또는 bridge crash |
| detect_img/itm/cloud Hz | 6~8Hz | VLM 서버 freeze, sync 실패, 또는 label 미입력 |
| sync ms | <50ms slop 유지 | 타임스탬프 정렬 실패 — use_sim_time 불일치 가능성 |
| cloud frame_id | `"World"` (대문자) | 과거 "world" 소문자 버그 재발 — `basic_utils/object_point_cloud_utils/object_point_cloud.py` 확인 |
| cloud centroid | 로봇 이동에도 고정 | 좌표 변환 버그 — `inverse_habitat_publisher_transform` 확인 |
| cloud n | ≤2000 | height filter 또는 downsample 미동작 |

## Report format

한국어로 3-5줄 요약:
1. 전체 Pass/Fail 판정
2. 실패한 체크 항목 + 가장 유력한 원인 1개
3. 다음 조치 제안 (예: "터미널 10 VLM 노드 로그 확인" / "scripts/start-vlm-servers.sh --restart 실행")

사용자가 `/vlm-diag 60` 처럼 시간 지정 시 그 값 사용.
