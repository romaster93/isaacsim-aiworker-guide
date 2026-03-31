# Step 12: InteriorAgent 데이터셋

## Overview

[InteriorAgent](https://huggingface.co/datasets/spatialverse/InteriorAgent)는 NVIDIA Isaac Sim용 고품질 3D USD 인테리어 씬 데이터셋입니다.
다양한 가구와 오브젝트가 배치된 실내 환경으로, ApexNAV Phase 3 VLM object-goal navigation 테스트에 활용합니다.

| 항목 | 내용 |
|------|------|
| **위치** | `/home/cho/InteriorAgent/` |
| **출처** | [HuggingFace: spatialverse/InteriorAgent](https://huggingface.co/datasets/spatialverse/InteriorAgent) |
| **씬 수** | 25개 (`kujiale_0003` ~ `kujiale_0067`) |
| **총 용량** | ~14GB |
| **총 메시** | 6,149개 (79개 카테고리) |
| **총 방** | 151개 |
| **포맷** | USD (.usda), MDL 머티리얼, HDR 조명, rooms.json |
| **호환** | Isaac Sim v4.2 / v4.5 (5.1.0 테스트 예정) |

---

## 씬 구조

```
kujiale_xxxx/
├── Materials/
│   ├── Textures/          # PBR 텍스처 (color, normal, metallic 등)
│   └── *.mdl              # NVIDIA MDL 머티리얼 (1.3/1.6)
├── Meshes/                # 오브젝트별 USD 메시 (bed, sofa, vase 등)
├── kujiale_xxxx.usda      # 메인 씬 파일 (Isaac Sim에서 열기)
├── *.hdr                  # 환경 HDR 조명
└── rooms.json             # 방 타입 + 폴리곤 좌표 (ground truth)
```

### USD 내부 계층

```
Root (Xform)
├── Meshes (Scope)
│   ├── <room_name>_<id> (Scope)       # bedroom_767840, kitchen_474 등
│   │   └── <object_type>_XXXX (Xform) # prepend references = @./Meshes/<name>.usd@
│   └── other (Scope)                  # 문, 창문
└── Rendering (Scope)
    └── Lights (Scope)                 # DistantLight, DomeLight, RectLight
```

---

## 씬 인벤토리

### 전체 요약

| Scene | Rooms | Meshes | Categories | Room Types |
|-------|------:|-------:|-----------:|------------|
| kujiale_0003 | 12 | 321 | 48 | living room, Bedroom, bedroom(x2), study room, Kitchen, bathroom(x2), balcony, storage, unknown(x2) |
| kujiale_0004 | 14 | 367 | 54 | living room, bedroom(x2), dining room, study room, kitchen, bathroom(x2), balcony(x2), unknown(x4) |
| kujiale_0008 | 6 | 258 | 44 | living room, Bedroom, bedroom, kitchen, unknown(x2) |
| kujiale_0009 | 5 | 246 | 39 | living room, bedroom(x2), bathroom, kitchen |
| kujiale_0020 | 5 | 204 | 42 | living room, bedroom, balcony, kitchen, bathroom |
| kujiale_0021 | 5 | 208 | 46 | living room, bedroom, balcony, kitchen, bathroom |
| kujiale_0022 | 5 | 185 | 46 | living room, bedroom(x2), bathroom, kitchen |
| kujiale_0024 | 5 | 246 | 45 | living room, bedroom, kitchen, bathroom, balcony |
| kujiale_0025 | 5 | 203 | 43 | living room, bedroom(x2), kitchen, Bathroom |
| kujiale_0026 | 8 | 328 | 50 | living room, bedroom, study room, dining room, bathroom, balcony(x2), kitchen |
| kujiale_0030 | 5 | 191 | 43 | living room, bedroom(x2), kitchen, bathroom |
| kujiale_0031 | 5 | 179 | 38 | living room, bedroom(x2), kitchen, bathroom |
| kujiale_0032 | 5 | 255 | 45 | living room, bedroom(x2), kitchen, bathroom |
| kujiale_0033 | 11 | 342 | 51 | living room, bedroom(x2), study room(x2), kitchen, balcony(x3), bathroom(x2) |
| kujiale_0034 | 5 | 248 | 45 | living room, bedroom, balcony, kitchen, bathroom |
| kujiale_0035 | 5 | 173 | 36 | living room, bedroom, kitchen, balcony, bathroom |
| kujiale_0036 | 5 | 257 | 46 | living room, bedroom(x2), kitchen, bathroom |
| kujiale_0037 | 5 | 265 | 45 | living room, bedroom(x2), bathroom, kitchen |
| kujiale_0038 | 5 | 245 | 45 | living room, bedroom, kitchen, balcony, bathroom |
| kujiale_0040 | 5 | 301 | 48 | living room, bedroom(x2), bathroom, kitchen |
| kujiale_0042 | 5 | 247 | 39 | living room, bedroom, study room, kitchen, bathroom |
| kujiale_0043 | 5 | 229 | 49 | living room, bedroom(x2), kitchen, bathroom |
| kujiale_0065 | 5 | 203 | 37 | living room, bedroom, kitchen, bathroom, balcony |
| kujiale_0066 | 5 | 227 | 45 | living room, bedroom, balcony, kitchen, bathroom |
| kujiale_0067 | 5 | 221 | 40 | living room, bedroom, kitchen, balcony, bathroom |
| **TOTAL** | **151** | **6,149** | **79** | |

### 오브젝트 카테고리 (상위 30개)

| Category | Count | Category | Count |
|----------|------:|----------|------:|
| book | 576 | picture_frame | 75 |
| ornament | 512 | tea_set | 70 |
| wall | 477 | table_lamp | 69 |
| cabinet | 303 | throw_pillow | 68 |
| ceiling | 301 | kitchenware | 66 |
| pillow | 289 | tray | 60 |
| floor | 179 | cup | 58 |
| doorsill | 153 | bedding | 54 |
| door | 152 | shelf | 53 |
| window | 148 | television | 53 |
| wine_set | 146 | ceiling_light | 52 |
| daily_equipment | 133 | sofa | 50 |
| plate | 133 | basin | 49 |
| chair | 129 | chopstick | 48 |
| bathroom_product | 127 | other_cooker | 47 |

### 희귀 오브젝트 (1~5개 씬에만 존재)

| Object | Scenes | Rarity |
|--------|--------|--------|
| computer | 0043 | 1개 씬 |
| fruit | 0040 | 1개 씬 |
| blanket | 0033 | 1개 씬 |
| water_tap | 0026 | 1개 씬 |
| screen | 0008 | 1개 씬 |
| pillar | 0008 | 1개 씬 |
| microwave | 0037, 0040 | 2개 씬 |
| toy | 0009, 0033 | 2개 씬 |
| decorative_box | 0004, 0067 | 2개 씬 |
| clock | 0004, 0026, 0034 | 3개 씬 |
| floor_lamp | 0037, 0040, 0043, 0066 | 4개 씬 |
| cushion | 0025, 0038, 0040, 0066 | 4개 씬 |
| menorah | 0008, 0026, 0034, 0043, 0067 | 5개 씬 |

> 상세 씬별 인벤토리: `/home/cho/InteriorAgent/INVENTORY.md`
> 구조화 데이터: `/home/cho/InteriorAgent/scene_inventory.json`

---

## USD 호환성 (Isaac Sim 5.1.0)

### 기본 속성

| Property | 씬 값 | Isaac Sim 5.1.0 기본값 | 호환? |
|----------|--------|----------------------|-------|
| `upAxis` | `"Z"` | `"Y"` | O (메타데이터 자동 인식) |
| `metersPerUnit` | `1` | `1` | O |
| USD 포맷 | USDA 1.0 + USDC | USDA/USDC/USDZ | O |
| MDL 버전 | 1.3 / 1.6 | 1.6+ 지원 | O |
| Light API | UsdLux inputs | UsdLux inputs | O |
| 메시 참조 | 상대 경로만 사용 | 상대 경로 | O |
| 텍스처 참조 | 상대 경로만 사용 | 상대 경로 | O |

### 결론: 기본적으로 잘 열림

- 모든 경로가 **상대 경로** — Nucleus 의존성 없음
- USD 스키마 표준 준수, deprecated API 없음
- `light:shaderId`는 legacy지만 무해

### 주의사항

| 심각도 | 이슈 | 설명 |
|--------|------|------|
| **중간** | Negative-scale 미러링 | 일부 가구에 `xformOp:scale = (-1, 1, 1)` 사용. 렌더링 OK, **PhysX 충돌 활성화 시 깨질 수 있음** |
| **중간** | 문/창문 Physics joints | PhysX 4.x 기준 작성됨. 5.1.0(PhysX 5.x)에서 joint stiffness/damping 동작 확인 필요 |
| **낮음** | RTX 렌더 설정 | kujiale_0009는 `PathTracing` 모드가 하드코딩됨 — 렌더 속도 느릴 수 있음 |
| **낮음** | SemanticsAPI 없음 | semantic segmentation 라벨이 USD에 없음 (rooms.json에만 존재) |

**Negative-scale 오브젝트 찾기:**
```bash
grep -r "xformOp:scale = (-" /home/cho/InteriorAgent/*/kujiale_*.usda
```

> 상세 리포트: `/home/cho/InteriorAgent/USD_COMPATIBILITY.md`

---

## VLM 테스트 시나리오

### 난이도 기준

| Tier | 기준 | VLM 난이도 |
|------|------|-----------|
| **Easy** | 크고 흔한 물체, 정석적 위치 (소파→거실) | 높은 recall, 명확한 탐지 |
| **Medium** | 특정 오브젝트, 중간 크기, 반정석 배치 | fine-grained grounding 필요 |
| **Hard** | 작고 희귀하고 시각적으로 복잡한 물체 | 낮은 confidence, 가려짐 |
| **Challenge** | 모호한 자연어 쿼리 (기능/의미 기반) | BLIP2-ITM 추론 필요 |

**총 339개 테스트 케이스**: Easy 96 / Medium 97 / Hard 75 / Challenge 71

### 씬별 테스트 용도 매핑

| 테스트 목적 | 추천 씬 | 이유 |
|------------|---------|------|
| **기본 Easy 테스트** | 0009, 0031, 0035 | 적은 오브젝트, 표준 방 구성 |
| **멀티룸 탐색** | 0003, 0026, 0033 | 8~12개 방, 넓은 플로어플랜 |
| **주방 도구 테스트** | 0032, 0038, 0033 | 포크/나이프/스푼/젓가락 풀세트 |
| **희귀 오브젝트** | 0008, 0026, 0043 | menorah, screen, computer, water_tap |
| **침실 구분** | 0022, 0040, 0037 | 침실 2개 이상 — 방 판별 필요 |
| **서재 테스트** | 0003, 0004, 0026, 0042 | desk, office_supply 포함 |
| **발코니 탐색** | 0020, 0033, 0065, 0066 | 발코니 공간 포함 |
| **가전 인식** | 0037, 0040, 0043 | microwave, floor_lamp, computer |
| **Challenge 쿼리** | 0025, 0033, 0040 | 의미적 다양성 (fruit, blanket, toy) |
| **세탁기 위치** | 0003, 0024, 0034, 0038, 0067 | washing_machine 포함 |
| **속도 벤치마크** | 0009, 0020, 0021, 0067 | 작은 5방 씬, 빠른 반복 |
| **스트레스 테스트** | 0033 | 11개 방 (발코니 3개, 서재 2개) |

### 예시 쿼리 (씬 kujiale_0003)

| # | 난이도 | 쿼리 | 타겟 | 예상 위치 |
|---|--------|------|------|----------|
| 1 | Easy | "find the sofa" | sofa | living room |
| 2 | Easy | "where is the bed" | bed | bedroom |
| 3 | Easy | "find the television" | television | living room |
| 4 | Medium | "find the chandelier" | chandelier | living room |
| 5 | Medium | "where is the wine set" | wine_set | living room |
| 6 | Medium | "find the vase" | vase | living room |
| 7 | Hard | "find the ornament" | ornament | bedroom / shelf |
| 8 | Hard | "find the cosmetics" | cosmetic | bathroom |
| 9 | Challenge | "find something to read" | book | study room |
| 10 | Challenge | "where can I relax" | sofa / bed | living room |
| 11 | Challenge | "find something decorative" | vase / ornament | living room |

### 테스트 프로토콜 (6단계)

1. **Phase 1 — 캘리브레이션** (씬: 0009, 0020, 0031): Easy 쿼리로 sofa, bed, TV, dining_table 기본 detection rate 확인
2. **Phase 2 — Medium** (씬: 0021, 0032, 0036): wine_set, tea_set, tablecloth, kettle 등 중간 난이도
3. **Phase 3 — Hard** (씬: 0003, 0026, 0034): ornament, menorah, clock 등 false-positive rate 측정
4. **Phase 4 — 희귀 오브젝트** (씬: 0008, 0040, 0043): screen, fruit, computer — zero-shot 일반화 평가
5. **Phase 5 — Challenge 쿼리** (씬: 0025, 0033, 0040): BLIP2-ITM ranking 정확도 측정
6. **Phase 6 — 스트레스 테스트** (씬: 0033): 11개 방 전체 탐색, 순차 쿼리, 누적 VLM 메모리 테스트

### VLM 파이프라인 팁

**GroundingDINO / YOLOv7:**
- sofa, bed, television, dining_table → 높은 confidence 기대, 캘리브레이션용
- chandelier → 위를 봐야 탐지됨, 벽 보고 있으면 놓칠 수 있음
- vase, ornament, tea_set → 작음, 선반 위에 있음 — 가까이에서 테스트

**BLIP2-ITM Challenge 쿼리 매핑:**
- "something to read" → book
- "somewhere comfortable to sit" → sofa, chair, stool, cushion
- "something decorative" → vase, ornament, painting, flower
- "something that tells time" → clock
- "something edible" → fruit
- "something to boil water" → kettle
- "something that burns/glows" → menorah, chandelier, lamp

**MobileSAM:**
- 큰 물체 (sofa, bed) → 양호한 세그멘테이션
- 투명/얇은 물체 (wine_set, cup, vase) → 경계 아티팩트 가능
- 밀집된 작은 물체 (bathroom_product, cosmetic) → 인스턴스 분리 실패 가능
- 씬 0031 (sparse) vs 0033 (dense)로 성능 범위 측정

> 전체 25개 씬 상세 시나리오: `/home/cho/InteriorAgent/TEST_SCENARIOS.md`

---

## 로봇 배치 스크립트

씬을 Isaac Sim에 로드하고 FFW-SG2 로봇을 거실 중심에 자동 배치하는 스크립트:

```
/home/cho/ms_AIworker/scripts/load_interioragent_scene.py
```

### 사용법

**Script Editor에서:**
1. 스크립트 상단의 `SCENE_NAME` 변수를 원하는 씬으로 변경 (기본: `kujiale_0065`)
2. 전체 코드를 Script Editor에 붙여넣기 후 실행

**Standalone 모드:**
```bash
python load_interioragent_scene.py
```

### 설정 변수

| 변수 | 기본값 | 설명 |
|------|--------|------|
| `SCENE_NAME` | `"kujiale_0065"` | 로드할 씬 이름 |
| `INTERIOR_AGENT_ROOT` | `/home/cho/InteriorAgent` | 데이터셋 루트 |
| `ROBOT_USD_PATH` | `/home/cho/AIworker/usd_ai_worker/ffw_sg2_follower/ffw_sg2_follower.usd` | 로봇 USD |
| `ROBOT_PRIM_PATH` | `/ffw_sg2_follower` | 로봇 prim 경로 (루트 레벨) |
| `SCENE_PRIM_PATH` | `/World/InteriorScene` | 씬이 마운트될 경로 |
| `ROBOT_Z_OFFSET` | `0.05` | 바닥에서 띄우는 높이 (m) |

### 동작 순서

1. `.usda` 및 로봇 USD 파일 존재 확인
2. `rooms.json`에서 living room 폴리곤 찾아 중심점 계산 (없으면 첫 번째 방 사용)
3. `/World/InteriorScene`에 씬 USD를 Reference로 추가
4. `/World/DistantLight` 기본 조명 생성 (없을 때만)
5. `/ffw_sg2_follower`에 로봇 USD를 Reference로 추가, 거실 중심에 배치
6. 씬 정보 출력 (방 수, 메시 수, 스폰 위치)

> **주의**: 로봇은 `/ffw_sg2_follower`에 루트 레벨로 배치됩니다 (`/World` 하위 아님). 기존 Stage 구조 규칙을 따릅니다.

---

## rooms.json 활용

각 씬의 `rooms.json`에 방 타입과 2D 폴리곤 좌표가 포함됩니다.

```json
{
    "room_type": "living room",
    "polygon": [
        [-4.29, -0.26],
        [-4.29, -3.84],
        [0.55, -4.77],
        ...
    ]
}
```

### Ground Truth 맵 / 커버리지 평가

```python
from shapely.geometry import Polygon
import json

with open("rooms.json", "r") as f:
    rooms = json.load(f)

for room in rooms:
    poly = Polygon(room["polygon"])
    print(f"Room: {room['room_type']}, Area: {poly.area:.1f}m2")
```

방 타입: living room, bedroom, kitchen, bathroom, study room, dining room, balcony, storage.

좌표계: X=전방, Y=오른쪽, Z=위 (Isaac Sim world frame 호환).

---

## 테스트 권장 순서

1. **가벼운 씬 먼저**: `kujiale_0065` (202MB, 5방, 203메시) — Isaac Sim 5.1.0 호환성 확인
2. **중간 씬**: `kujiale_0003` (505MB, 12방, 321메시) — 멀티룸 탐색 테스트
3. **복잡한 씬**: `kujiale_0033` (896MB, 11방, 342메시) — 스트레스 테스트
4. **최대 씬**: `kujiale_0040` (1.3GB, 5방, 301메시) — 대용량 환경 테스트

---

## 다운로드 방법 (참고)

```bash
pip install huggingface_hub
python3 -c "
from huggingface_hub import snapshot_download
snapshot_download('spatialverse/InteriorAgent', repo_type='dataset',
                  local_dir='/home/cho/InteriorAgent')
"
```

---

**이전**: [Step 11: ApexNAV VLM 통합](11-apexnav-vlm.md)
