"""
load_interioragent_scene.py
============================
InteriorAgent 씬을 로드하고 FFW-SG2 로봇을 거실 중심에 배치하는 스크립트.

Isaac Sim 5.1.0 Script Editor에서 실행하거나
standalone 모드로 실행 가능:
    python load_interioragent_scene.py

사용법:
    Script Editor에서 직접 붙여넣기 후 실행
    standalone 실행 시 SCENE_NAME 변수를 수정
"""

import json
import os

# ─────────────────────────────────────────────────────────────────────────────
# 설정 변수 (필요에 따라 수정)
# ─────────────────────────────────────────────────────────────────────────────

# 로드할 씬 이름 (InteriorAgent 데이터셋 폴더명)
SCENE_NAME = "kujiale_0065"

# InteriorAgent 씬 루트 디렉토리
INTERIOR_AGENT_ROOT = "/home/cho/InteriorAgent"

# FFW-SG2 로봇 USD 경로
ROBOT_USD_PATH = "/home/cho/AIworker/usd_ai_worker/ffw_sg2_follower/ffw_sg2_follower.usd"

# 로봇이 배치될 씬 내 prim 경로 (루트 레벨, /World 밖)
ROBOT_PRIM_PATH = "/ffw_sg2_follower"

# InteriorAgent 씬이 마운트될 prim 경로
SCENE_PRIM_PATH = "/World/InteriorScene"

# 로봇 Z 오프셋 (바닥에서 살짝 띄움, 단위: 미터)
ROBOT_Z_OFFSET = 0.05

# ─────────────────────────────────────────────────────────────────────────────
# 유틸리티 함수
# ─────────────────────────────────────────────────────────────────────────────

def find_living_room_centroid(rooms_json_path: str):
    """
    rooms.json에서 거실(living room) 폴리곤을 찾아 중심점(centroid)을 반환.

    좌표계: InteriorAgent는 2D (X, Y) 평면 좌표를 사용.
    Isaac Sim 좌표계: X=전방, Y=오른쪽, Z=위쪽.
    rooms.json의 X,Y → Isaac Sim의 X,Y로 그대로 매핑.

    반환: (x, y) 튜플 또는 (0.0, 0.0) (거실 없을 때)
    """
    if not os.path.exists(rooms_json_path):
        print(f"[WARN] rooms.json 없음: {rooms_json_path}")
        return (0.0, 0.0)

    with open(rooms_json_path, "r", encoding="utf-8") as f:
        rooms = json.load(f)

    # 거실(living room) 찾기
    living_room = None
    for room in rooms:
        if room.get("room_type", "").lower() == "living room":
            living_room = room
            break

    if living_room is None:
        # 거실이 없으면 첫 번째 방 사용
        print("[WARN] 거실(living room)을 찾지 못함. 첫 번째 방의 중심 사용.")
        living_room = rooms[0] if rooms else None

    if living_room is None:
        print("[WARN] 방 정보 없음. 원점에 배치.")
        return (0.0, 0.0)

    # 폴리곤 중심점 계산 (단순 평균)
    polygon = living_room["polygon"]
    cx = sum(p[0] for p in polygon) / len(polygon)
    cy = sum(p[1] for p in polygon) / len(polygon)

    room_type = living_room["room_type"]
    print(f"[INFO] 방 타입: {room_type}, 폴리곤 꼭짓점 수: {len(polygon)}")
    print(f"[INFO] 거실 중심 (X={cx:.4f}, Y={cy:.4f})")
    return (cx, cy)


def print_scene_info(rooms_json_path: str, scene_prim_path: str, spawn_pos):
    """씬 정보를 출력."""
    import omni.usd
    stage = omni.usd.get_context().get_stage()

    # 방 개수
    room_count = 0
    if os.path.exists(rooms_json_path):
        with open(rooms_json_path, "r", encoding="utf-8") as f:
            rooms = json.load(f)
        room_count = len(rooms)
        room_types = [r.get("room_type", "unknown") for r in rooms]
        print(f"[INFO] 방 개수: {room_count} ({', '.join(room_types)})")

    # 씬 prim 하위의 메시 개수 (간단 카운트)
    mesh_count = 0
    if stage:
        scene_prim = stage.GetPrimAtPath(scene_prim_path)
        if scene_prim.IsValid():
            for prim in stage.TraverseAll():
                if str(prim.GetPath()).startswith(scene_prim_path):
                    if prim.GetTypeName() == "Mesh":
                        mesh_count += 1

    x, y, z = spawn_pos
    print(f"[INFO] 메시 개수 (씬 내): {mesh_count}")
    print(f"[INFO] 로봇 스폰 위치: X={x:.4f}, Y={y:.4f}, Z={z:.4f}")


# ─────────────────────────────────────────────────────────────────────────────
# 메인 로직
# ─────────────────────────────────────────────────────────────────────────────

def main():
    print("=" * 60)
    print(f"  InteriorAgent 씬 로더  [씬: {SCENE_NAME}]")
    print("=" * 60)

    # ── 1. 경로 설정 ──────────────────────────────────────────────────────────
    scene_dir = os.path.join(INTERIOR_AGENT_ROOT, SCENE_NAME)
    scene_usda = os.path.join(scene_dir, f"{SCENE_NAME}.usda")
    rooms_json = os.path.join(scene_dir, "rooms.json")

    # 파일 존재 확인
    if not os.path.exists(scene_usda):
        print(f"[ERROR] 씬 파일 없음: {scene_usda}")
        return
    if not os.path.exists(ROBOT_USD_PATH):
        print(f"[ERROR] 로봇 USD 없음: {ROBOT_USD_PATH}")
        return

    print(f"[INFO] 씬 파일: {scene_usda}")
    print(f"[INFO] 로봇 USD: {ROBOT_USD_PATH}")

    # ── 2. 거실 중심점 계산 ───────────────────────────────────────────────────
    cx, cy = find_living_room_centroid(rooms_json)
    spawn_x = cx
    spawn_y = cy
    spawn_z = ROBOT_Z_OFFSET

    # ── 3. Isaac Sim API 임포트 ───────────────────────────────────────────────
    import omni.usd
    import omni.kit.commands
    from pxr import Gf, Sdf, UsdGeom

    # 현재 스테이지 가져오기
    context = omni.usd.get_context()
    stage = context.get_stage()

    if stage is None:
        print("[ERROR] 스테이지를 가져올 수 없음. Isaac Sim이 실행 중인지 확인하세요.")
        return

    # ── 4. 새 스테이지 열기 또는 현재 스테이지 초기화 ──────────────────────
    # Script Editor에서 실행 시 현재 스테이지를 재사용
    # 필요 시 아래 주석을 해제하면 새 스테이지로 시작
    # context.new_stage()
    # stage = context.get_stage()

    # ── 5. /World 기본 prim 확인 및 생성 ─────────────────────────────────────
    world_prim = stage.GetPrimAtPath("/World")
    if not world_prim.IsValid():
        # /World가 없으면 Xform으로 생성
        omni.kit.commands.execute(
            "CreatePrimWithDefaultXform",
            prim_type="Xform",
            prim_path="/World",
        )
        print("[INFO] /World prim 생성")

    # ── 6. InteriorAgent 씬 로드 (Reference로 추가) ───────────────────────────
    print(f"\n[STEP] InteriorAgent 씬 로드 중...")

    # SCENE_PRIM_PATH에 씬을 Reference로 추가
    scene_prim = stage.GetPrimAtPath(SCENE_PRIM_PATH)
    if not scene_prim.IsValid():
        # Xform prim 생성 후 reference 추가
        omni.kit.commands.execute(
            "CreatePrimWithDefaultXform",
            prim_type="Xform",
            prim_path=SCENE_PRIM_PATH,
        )
        scene_prim = stage.GetPrimAtPath(SCENE_PRIM_PATH)

    # USD Reference로 씬 파일 연결
    references = scene_prim.GetReferences()
    references.ClearReferences()
    references.AddReference(scene_usda)
    print(f"[OK] 씬 로드 완료: {SCENE_PRIM_PATH}")

    # ── 7. 기본 조명 설정 ─────────────────────────────────────────────────────
    # DistantLight (태양광)가 없으면 추가
    light_path = "/World/DistantLight"
    if not stage.GetPrimAtPath(light_path).IsValid():
        omni.kit.commands.execute(
            "CreatePrimWithDefaultXform",
            prim_type="DistantLight",
            prim_path=light_path,
        )
        light_prim = stage.GetPrimAtPath(light_path)
        # 밝기 설정
        light_prim.GetAttribute("inputs:intensity").Set(3000.0)
        # 45도 각도로 기울이기
        xform = UsdGeom.Xformable(light_prim)
        xform.ClearXformOpOrder()
        xform.AddRotateXYZOp().Set(Gf.Vec3f(45.0, 0.0, 0.0))
        print(f"[OK] 기본 조명 추가: {light_path}")
    else:
        print(f"[INFO] 기존 조명 사용: {light_path}")

    # ── 8. FFW-SG2 로봇 배치 ─────────────────────────────────────────────────
    print(f"\n[STEP] FFW-SG2 로봇 배치 중...")
    print(f"[INFO] 로봇 prim 경로: {ROBOT_PRIM_PATH} (루트 레벨)")

    # 로봇 prim이 이미 있으면 제거 후 재생성 (재실행 시 중복 방지)
    existing_robot = stage.GetPrimAtPath(ROBOT_PRIM_PATH)
    if existing_robot.IsValid():
        omni.kit.commands.execute("DeletePrimsCommand", paths=[ROBOT_PRIM_PATH])
        print(f"[INFO] 기존 로봇 prim 제거: {ROBOT_PRIM_PATH}")

    # 루트 레벨에 Xform prim 생성 (/World 밖, 아티큘레이션 루트용)
    omni.kit.commands.execute(
        "CreatePrimWithDefaultXform",
        prim_type="Xform",
        prim_path=ROBOT_PRIM_PATH,
    )

    robot_prim = stage.GetPrimAtPath(ROBOT_PRIM_PATH)

    # 로봇 USD를 Reference로 추가
    robot_refs = robot_prim.GetReferences()
    robot_refs.AddReference(ROBOT_USD_PATH)

    # 로봇 위치 설정 (거실 중심 + Z 오프셋)
    xform = UsdGeom.Xformable(robot_prim)
    xform.ClearXformOpOrder()

    # Translate: 거실 중심에 배치
    translate_op = xform.AddTranslateOp()
    translate_op.Set(Gf.Vec3d(spawn_x, spawn_y, spawn_z))

    # 로봇 초기 방향: X=전방 방향으로 세팅 (회전 없음 = 기본 자세)
    # 필요 시 RotateZ로 방향 조정 가능
    # xform.AddRotateZOp().Set(0.0)

    print(f"[OK] 로봇 배치 완료: {ROBOT_PRIM_PATH}")
    print(f"     위치: X={spawn_x:.4f}, Y={spawn_y:.4f}, Z={spawn_z:.4f}")

    # ── 9. 씬 정보 출력 ───────────────────────────────────────────────────────
    print("\n" + "=" * 60)
    print("  씬 정보 요약")
    print("=" * 60)
    print_scene_info(rooms_json, SCENE_PRIM_PATH, (spawn_x, spawn_y, spawn_z))
    print(f"[INFO] 씬 이름: {SCENE_NAME}")
    print(f"[INFO] 씬 prim: {SCENE_PRIM_PATH}")
    print(f"[INFO] 로봇 prim: {ROBOT_PRIM_PATH}")
    print("=" * 60)
    print("[완료] 씬 및 로봇 로드 완료!")
    print("       Isaac Sim 뷰포트에서 확인하세요.")
    print("=" * 60)


# ─────────────────────────────────────────────────────────────────────────────
# 실행 진입점
# ─────────────────────────────────────────────────────────────────────────────

# Script Editor에서 직접 실행 시 바로 main() 호출
# standalone 모드에서도 동일하게 동작

if __name__ == "__main__":
    # Standalone 모드: SimulationApp을 먼저 초기화
    try:
        from isaacsim import SimulationApp
        simulation_app = SimulationApp({"headless": False, "width": 1280, "height": 720})
        main()
        # 뷰포트 유지 (창 닫을 때까지 대기)
        while simulation_app.is_running():
            simulation_app.update()
        simulation_app.close()
    except ImportError:
        # Script Editor에서 실행 시 SimulationApp 없이 직접 실행
        main()
else:
    # Script Editor에서 import 없이 exec()로 실행될 때
    main()
