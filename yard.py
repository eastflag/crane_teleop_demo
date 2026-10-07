"""야드: 씬 애셋 로드 + 개별 철근/분철 강체 컬렉션 + 등급 구역 판정.

씬 구성물(yard.usd·truck_load.usd·rebar_set.usd·crane.usd)은 make_scene_assets.py
로 미리 만들어 두고 데모에서 reference로 로드한다(스크립트가 아니라 애셋으로 구성).
- 트럭 위 분철 더미는 시각+정적 충돌체만 있고, 자석으로 집는 개별 강체는
  더미 위 ~1cm에서 떨어뜨려 안착시킨다(30개 이하).
- 항아리 통은 폐기: 웹 페이지처럼 등급별 지정 구역(중량A/B·경량A/B)에 적치한다.
"""
import math
from pathlib import Path

import numpy as np
from pxr import UsdPhysics

import config as C

ASSET_DIR = Path(__file__).resolve().parent / "assets"
YARD_ASSET = ASSET_DIR / "yard.usd"
TRUCK_ASSET = ASSET_DIR / "truck_load.usd"
REBAR_ASSET = ASSET_DIR / "rebar_set.usd"
CRANE_ASSET = ASSET_DIR / "crane.usd"


def load_scene_assets(stage):
    """야드·트럭·철근·크레인 애셋을 /World 아래 reference로 로드.

    반환: {"yard": 마운트 경로|None, "truck": ..., "rebar": ..., "crane": ...}
    (없는 애셋은 None - 데모가 절차적 폴백으로 처리)
    """
    out = {}
    for name, path, mount in (("yard", YARD_ASSET, "/World/Yard"),
                              ("truck", TRUCK_ASSET, "/World/Truck"),
                              ("rebar", REBAR_ASSET, "/World/RebarSet"),
                              ("crane", CRANE_ASSET, "/World/Crane")):
        if path.is_file():
            prim = stage.DefinePrim(mount, "Xform")
            prim.GetReferences().AddReference(str(path))
            out[name] = mount
            print(f"[yard] 애셋 로드: {path.name} → {mount}")
        else:
            out[name] = None
            print(f"[yard] 애셋 없음({path.name}) - 절차적 폴백")
    return out


# ═══════════════════════ 개별 철근/분철 컬렉션 ═══════════════════════
def rebar_count_available():
    """rebar_set.usd 에 실려 있는 강체 수(애셋 생성 시 결정)."""
    if not REBAR_ASSET.is_file():
        return 0
    from pxr import Usd
    stage = Usd.Stage.Open(str(REBAR_ASSET))
    n = sum(1 for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI))
    return n


def make_rebar_collection_cfg(num=C.REBAR_NUM, seed=7):
    """자석으로 집는 철근/분철 컬렉션 설정.

    rebar_set.usd(개별 강체 ≤30, 더미 위 1cm)를 reference로 깔아둔 프림을
    그대로 래핑한다(spawn=None). 애셋이 없으면 구버전처럼 캡슐을 절차 스폰.
    """
    import isaaclab.sim as sim_utils
    from isaaclab.assets import RigidObjectCfg, RigidObjectCollectionCfg

    if REBAR_ASSET.is_file():
        entries = {}
        stage_rebar = rebar_count_available()
        for i in range(min(num, stage_rebar)):
            entries[f"rebar_{i}"] = RigidObjectCfg(
                prim_path=f"/World/RebarSet/rebar_{i}",
                spawn=None,  # 이미 애셋에 리지드바디+충돌+질량이 구성됨
                init_state=RigidObjectCfg.InitialStateCfg(),
            )
        print(f"[yard] 개별 강체 {len(entries)}개 - 애셋 프림 래핑(spawn=None)")
        return RigidObjectCollectionCfg(rigid_objects=entries)

    # 폴백: 캡슐 절차 스폰(구버전 동작)
    rng = np.random.default_rng(seed)
    entries = {}
    _BUNDLE_DX = (-1.15, 0.0, 1.15)
    for i in range(num):
        b = _BUNDLE_DX[(i // 5) % len(_BUNDLE_DX)]
        k = i % 5
        layer, col = (0, k) if k < 3 else (1, k - 3)
        x = C.TRUCK_XY[0] + b + col * 0.036 + (0.018 if layer else 0.0)
        y = C.TRUCK_XY[1] + rng.uniform(-0.12, 0.12)
        z = C.TRUCK_BED_Z + 0.24 + 0.018 + layer * 0.032
        yaw = math.pi / 2.0 + rng.uniform(-0.05, 0.05)
        cy, sy_ = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
        entries[f"rebar_{i}"] = RigidObjectCfg(
            prim_path=f"{{ENV_REGEX_NS}}/rebar_{i}",
            spawn=sim_utils.CapsuleCfg(
                radius=C.REBAR_RADIUS,
                height=C.REBAR_LENGTH - 2.0 * C.REBAR_RADIUS,
                collision_props=sim_utils.CollisionPropertiesCfg(),
                rigid_props=sim_utils.RigidBodyPropertiesCfg(),
                visual_material=sim_utils.PreviewSurfaceCfg(
                    diffuse_color=(0.62, 0.30, 0.12), metallic=0.55, roughness=0.4
                ),
            ),
            init_state=RigidObjectCfg.InitialStateCfg(pos=(x, y, z),
                                                      rot=(cy, 0.0, 0.0, sy_)),
        )
    print(f"[yard] 개별 강체 {num}개 - 절차 스폰 폴백")
    return RigidObjectCollectionCfg(rigid_objects=entries)


# ═══════════════════════ 등급 구역 판정 ═══════════════════════
def zone_at(x, y):
    """(x, y)가 속한 등급 구역 인덱스(0~3). 구역 밖이면 -1."""
    zw, zd = C.ZONE_SIZE
    for i, z in enumerate(C.ZONES):
        if abs(x - z["cx"]) <= zw / 2.0 and abs(y - z["cy"]) <= zd / 2.0:
            return i
    return -1


def zone_name(i):
    return C.ZONES[i]["name"] if 0 <= i < len(C.ZONES) else "구역 밖"


def judge_drop(x, y, target):
    """낙하 지점 판정(웹 페이지의 적치 판정 메시지 재현).

    반환: (정답 여부, 메시지)
    """
    drop = zone_at(x, y)
    if drop == target:
        return True, f"{zone_name(target)} 구역에 정위치 적치했습니다"
    if drop < 0:
        return False, f"적치 구역 밖에 내려놓았습니다 (판정 {zone_name(target)})"
    return False, f"판정은 {zone_name(target)}인데 {zone_name(drop)} 구역에 놓였습니다"


# ═══════════════════════ 절차적 폴백(애셋 없을 때) ═══════════════════════
def add_truck(stage):
    """(폴백) 철근을 실은 트럭 - 간이 형상."""
    from usd_utils import box, cylinder, make_material

    looks = "/World/Yard/Looks"
    body = make_material(stage, f"{looks}/TruckBody", (0.72, 0.70, 0.66), roughness=0.5, metallic=0.2)
    dark = make_material(stage, f"{looks}/TruckDark", (0.13, 0.14, 0.16), roughness=0.6, metallic=0.3)
    x, y = C.TRUCK_XY
    root = "/World/Yard/Truck"
    bed_top = C.TRUCK_BED_Z
    wheel_q = quat_axis_angle((1.0, 0.0, 0.0), math.pi / 2.0)
    for i, ax in enumerate((-1.3, 0.9, 2.35)):
        for j, wy in enumerate((-0.85, 0.85)):
            cylinder(stage, f"{root}/Wheel{i}{j}", radius=0.35, height=0.20,
                     mat=dark, pos=(x + ax, y + wy, 0.35), quat=wheel_q)
    box(stage, f"{root}/Bed", 1.0, dark, pos=(x, y, bed_top - 0.06), scale=(3.4, 1.9, 0.12))
    for side, ry in (("L", 0.95), ("R", -0.95)):
        box(stage, f"{root}/Rail{side}", 1.0, body, pos=(x, y + ry, bed_top + 0.15), scale=(3.4, 0.05, 0.30))
    box(stage, f"{root}/RailBack", 1.0, body, pos=(x - 1.70, y, bed_top + 0.15), scale=(0.05, 1.9, 0.30))
    box(stage, f"{root}/Cab", 1.0, body, pos=(x + 2.35, y, bed_top + 0.45), scale=(1.25, 1.85, 1.00))
    for name in ("Bed", "RailL", "RailR", "RailBack"):
        UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(f"{root}/{name}"))
