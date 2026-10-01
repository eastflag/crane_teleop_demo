"""철근 야드: 물리 강체(RigidObjectCollection) 설정 생성 + 드롭존 표시.

철근이 이 씬에서 유일한 동적 물리 객체다(중력·충돌). 나머지(크레인·조종실·로봇)는
운동학적 시각 요소로서 물리 시뮬레이션에 참여하지 않는다.
"""
import numpy as np

import config as C
from usd_utils import cylinder, make_material

# 캡슐 높이 = 전장 - 반구 2개분
_CAPSULE_H = C.REBAR_LENGTH - 2.0 * C.REBAR_RADIUS


def make_rebar_collection_cfg(num=C.REBAR_NUM, seed=7):
    import isaaclab.sim as sim_utils
    from isaaclab.assets import RigidObjectCfg, RigidObjectCollectionCfg

    rng = np.random.default_rng(seed)
    piles = [np.array(C.PILE_A_XY), np.array(C.PILE_B_XY)]
    entries = {}

    for i in range(num):
        pile = piles[i % 2]
        row, layer = (i // 2) % 4, (i // 2) // 4
        # 더미 A는 X방향, B는 Y방향으로 눕힘(다양성) + 미세 요동
        yaw = (0.0 if i % 2 == 0 else np.pi / 2.0) + rng.uniform(-0.06, 0.06)
        along = rng.uniform(-0.25, 0.25)
        rod_dir = np.array([np.cos(yaw), np.sin(yaw)])
        perp_dir = np.array([-rod_dir[1], rod_dir[0]])
        pos = np.array([
            pile[0] + along * rod_dir[0] + (row - 1.5) * 0.05 * perp_dir[0],
            pile[1] + along * rod_dir[1] + (row - 1.5) * 0.05 * perp_dir[1],
            0.03 + layer * 0.037,
        ])
        # 캡슐 기본축 Z를 수평으로: Z축 회전(yaw) ∘ Y축 90°(Z→X)
        cy, sy_ = np.cos(yaw / 2.0), np.sin(yaw / 2.0)
        q_yaw = np.array([cy, 0.0, 0.0, sy_])
        q_tip = np.array([np.cos(np.pi / 4.0), 0.0, np.sin(np.pi / 4.0), 0.0])
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
            init_state=RigidObjectCfg.InitialStateCfg(pos=tuple(pos), rot=tuple(quat)),
        )
    return RigidObjectCollectionCfg(rigid_objects=entries)


def add_drop_zone(stage):
    """드롭존 바닥 표시(시각 전용)."""
    looks = "/World/Yard/Looks"
    yellow = make_material(stage, f"{looks}/Zone", (0.90, 0.80, 0.10), roughness=0.6)
    cylinder(stage, "/World/Yard/DropZone", radius=1.5, height=0.02, mat=yellow,
             pos=(*C.DROP_XY, 0.012))
    cylinder(stage, "/World/Yard/DropZoneRing", radius=1.62, height=0.015, mat=yellow,
             pos=(*C.DROP_XY, 0.010))
