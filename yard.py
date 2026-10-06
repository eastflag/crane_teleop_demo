"""야드: 철근을 실은 트럭(분철 픽업) + 검은 항아리 통(반입) + 철근 강체 컬렉션.

철근이 이 씬에서 유일한 동적 물리 객체다(중력·충돌). 트럭 적재대와 항아리 통은
충돌체를 가진 정적 구조물이고, 나머지(크레인·조종실·로봇)는 물리에 참여하지 않는다.
"""
import math

import numpy as np
from pxr import UsdPhysics

import config as C
from usd_utils import box, cylinder, make_material, quat_axis_angle

# 캡슐 높이 = 전장 - 반구 2개분
_CAPSULE_H = C.REBAR_LENGTH - 2.0 * C.REBAR_RADIUS

# 분철 배치: 3개 분철 × 5본(아래 3 + 위 2), 트럭 X방향 간격
_BUNDLE_DX = (-1.15, 0.0, 1.15)
_PER_BUNDLE = 5


def make_rebar_collection_cfg(num=C.REBAR_NUM, seed=7):
    """트럭 적재대 위 분철 3개(각 5본)로 철근을 스폰한다."""
    import isaaclab.sim as sim_utils
    from isaaclab.assets import RigidObjectCfg, RigidObjectCollectionCfg

    rng = np.random.default_rng(seed)
    entries = {}

    for i in range(num):
        b = _BUNDLE_DX[(i // _PER_BUNDLE) % len(_BUNDLE_DX)]
        k = i % _PER_BUNDLE
        layer, col = (0, k) if k < 3 else (1, k - 3)
        x = C.TRUCK_XY[0] + b + col * 0.036 + (0.018 if layer else 0.0)
        y = C.TRUCK_XY[1] + rng.uniform(-0.12, 0.12)
        z = C.TRUCK_BED_Z + 0.018 + layer * 0.032
        # 철근은 트럭 가로(Y)방향으로 눕힘 + 미세 요동
        yaw = math.pi / 2.0 + rng.uniform(-0.05, 0.05)
        cy, sy_ = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
        q_yaw = np.array([cy, 0.0, 0.0, sy_])
        q_tip = np.array([math.cos(math.pi / 4.0), 0.0, math.sin(math.pi / 4.0), 0.0])
        from usd_utils import quat_mul
        quat = quat_mul(q_yaw, q_tip)

        entries[f"rebar_{i}"] = RigidObjectCfg(
            prim_path=f"{{ENV_REGEX_NS}}/rebar_{i}",
            spawn=sim_utils.CapsuleCfg(
                radius=C.REBAR_RADIUS,
                height=_CAPSULE_H,
                collision_props=sim_utils.CollisionPropertiesCfg(),
                rigid_props=sim_utils.RigidBodyPropertiesCfg(),
                visual_material=sim_utils.PreviewSurfaceCfg(
                    diffuse_color=(0.62, 0.30, 0.12), metallic=0.55, roughness=0.4
                ),
            ),
            init_state=RigidObjectCfg.InitialStateCfg(pos=(x, y, z), rot=tuple(quat)),
        )
    return RigidObjectCollectionCfg(rigid_objects=entries)


def add_truck(stage):
    """철근을 실은 트럭(시각 + 적재대·레일 충돌체). 바퀴 6개, X방향 주차."""
    looks = "/World/Yard/Looks"
    body = make_material(stage, f"{looks}/TruckBody", (0.72, 0.70, 0.66), roughness=0.5, metallic=0.2)
    dark = make_material(stage, f"{looks}/TruckDark", (0.13, 0.14, 0.16), roughness=0.6, metallic=0.3)
    x, y = C.TRUCK_XY
    root = "/World/Yard/Truck"
    bed_top = C.TRUCK_BED_Z

    # 바퀴(3축 × 2)
    wheel_q = quat_axis_angle((1.0, 0.0, 0.0), math.pi / 2.0)  # 축을 Y로
    for i, ax in enumerate((-1.3, 0.9, 2.35)):
        for j, wy in enumerate((-0.85, 0.85)):
            cylinder(stage, f"{root}/Wheel{i}{j}", radius=0.35, height=0.20,
                     mat=dark, pos=(x + ax, y + wy, 0.35), quat=wheel_q)
    # 적재대(충돌) + 측면/뒷면 레일
    box(stage, f"{root}/Bed", 1.0, dark, pos=(x, y, bed_top - 0.06), scale=(3.4, 1.9, 0.12))
    for side, ry in (("L", 0.95), ("R", -0.95)):
        box(stage, f"{root}/Rail{side}", 1.0, body, pos=(x, y + ry, bed_top + 0.15), scale=(3.4, 0.05, 0.30))
    box(stage, f"{root}/RailBack", 1.0, body, pos=(x - 1.70, y, bed_top + 0.15), scale=(0.05, 1.9, 0.30))
    # 운전석(앞쪽)
    box(stage, f"{root}/Cab", 1.0, body, pos=(x + 2.35, y, bed_top + 0.45), scale=(1.25, 1.85, 1.00))

    # 분철이 얹히는 표면 충돌체
    for name in ("Bed", "RailL", "RailR", "RailBack"):
        UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(f"{root}/{name}"))


def add_jar(stage):
    """검은 항아리 통(위가 뚫린 8각 벽 + 바닥, 전부 충돌체). 철근 반입 목적지."""
    looks = "/World/Yard/Looks"
    black = make_material(stage, f"{looks}/JarBlack", (0.07, 0.07, 0.09), roughness=0.35, metallic=0.1)
    x, y = C.JAR_XY
    root = "/World/Yard/Jar"
    radius, height, thick = C.JAR_R, C.JAR_H, 0.08

    # 벽 8Segments(겉보기 지름 1.6m) - 긴 축이 접선 방향이 되도록 ang+90° 회전
    n = 8
    for i in range(n):
        ang = 2.0 * math.pi * i / n
        r = radius - thick / 2.0
        pos = (x + r * math.cos(ang), y + r * math.sin(ang), height / 2.0)
        box(stage, f"{root}/Wall{i}", 1.0, black, pos=pos,
            quat=quat_axis_angle((0.0, 0.0, 1.0), ang + math.pi / 2.0),
            scale=(2.0 * radius * math.sin(math.pi / n) + thick, thick, height))
    # 바닥
    cylinder(stage, f"{root}/Bottom", radius=radius, height=0.06, mat=black, pos=(x, y, 0.03))

    for i in range(n):
        UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(f"{root}/Wall{i}"))
    UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(f"{root}/Bottom"))
