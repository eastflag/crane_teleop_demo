"""갠트리 크레인: 프리즘 3축 아큘레이션 구동(주행 Y + 횡행 X + 권상 Z).

구조(assets/crane.usd, make_scene_assets.py 로 생성):
  - 정적: 고가 레일 + 기둥(야드 양쪽 x=1/17)
  - 아큘레이션: Portal(Y 조인트, 월드 고정) → Trolley(X 조인트) → Hook(Z 조인트)
    각 축은 위치 제어 드라이브를 가진다(게인은 config.CRANE_DRIVE).
  - 후크 링크에 자석 디스크가 붙어 있고, 와이어는 시각용 캡슐(런타임 갱신).

런타임 동작:
  - 스틱 속도 명령을 적분해 조인트 위치 목표로 쓴다(작업 범위 클램프).
  - 실제 관절 위치/속도에서 자석 위치를 계산해 magnet_logic 로 전달.
  - 아큘레이션 핸들이 없으면(애셋 없음) 기존 절차적 생성+운동학 구동으로 폴백.
"""
import math
from pathlib import Path

import numpy as np

import config as C
from input_source import Command
from usd_utils import box, capsule, cylinder, group, make_material, place_segment, set_xform

G = 9.81

CRANE_ASSET = Path(__file__).resolve().parent / "assets" / "crane.usd"
"""크레인 USD 애셋(프리즘 3축 아큘레이션 포함). 없으면 절차적 폴백."""

HOOK_REST_Z = 3.5          # 후크 링크 작성 높이(crane.usd 와 일치)
JOINT_NAMES = {"x": "TrolleyTravelJoint", "y": "PortalTravelJoint", "z": "HoistJoint"}


class GantryCrane:
    def __init__(self, stage, articulation=None, usd_asset="auto"):
        """articulation: Isaac Lab Articulation 핸들(assets/crane.usd 래핑).
        None이면 절차적 생성 + 운동학 구동(레거시)으로 폴백한다."""
        self.root = "/World/Crane"
        self.has_articulation = articulation is not None
        self._art = articulation

        if self.has_articulation:
            self._init_from_articulation(stage)
        else:
            self._build_procedural(stage)

        # ── 상태 ──
        self.x, self.y, self.z = C.CRANE_START          # 명령 적분값(목표)
        self.th = np.zeros(2)                            # 진자 각(폴백 모드만)
        self.thd = np.zeros(2)
        self.magnet_pos = np.zeros(3)
        self.magnet_vel = np.zeros(3)
        self._prev_anchor = None
        self._prev_anchor_vel = np.zeros(2)
        self._prev_magnet = None
        if not self.has_articulation:
            self.update(1.0 / 120.0, Command())  # 절차적 모드 초기 배치
            # 아큘레이션 모드는 리셋 후 첫 update에서 관절 상태를 읽는다

    # ───────────────────── 아큘레이션 모드(crane.usd) ─────────────────────
    def _init_from_articulation(self, stage):
        from usd_utils import attach_capsule

        self._cable = attach_capsule(stage, f"{self.root}/Cable")
        # 조인트 인덱스: [주행(y), 횡행(x), 권상(z)] 순서로 고정
        ids, names = self._art.find_joints(
            [JOINT_NAMES["y"], JOINT_NAMES["x"], JOINT_NAMES["z"]])
        assert len(ids) == 3, f"크레인 조인트 발견 실패: {names}"
        self._joint_ids = ids
        print(f"[crane] 프리즘 3축 아큘레이션 구동: {names}")

    # ───────────────────── 절차적 폴백(레거시 운동학) ─────────────────────
    def _build_procedural(self, stage):
        self.portal_op = group(stage, f"{self.root}/Portal")
        looks = f"{self.root}/Looks"
        self._mats = {
            "yellow": make_material(stage, f"{looks}/Yellow", (0.85, 0.66, 0.10), roughness=0.5, metallic=0.3),
            "gray":   make_material(stage, f"{looks}/Gray",   (0.30, 0.32, 0.36), roughness=0.6, metallic=0.4),
            "dark":   make_material(stage, f"{looks}/Dark",   (0.12, 0.12, 0.14), roughness=0.7),
            "red":    make_material(stage, f"{looks}/Red",    (0.80, 0.15, 0.12), roughness=0.4, metallic=0.5),
        }
        for i, x in enumerate(C.RAIL_X):
            box(stage, f"{self.root}/Rail{i}", size=1.0, mat=self._mats["dark"],
                pos=(x, 0.0, 0.07), scale=(0.40, C.RAIL_LEN_Y + 1.0, 0.14))
        for i, x in enumerate(C.RAIL_X):
            cylinder(stage, f"{self.root}/Portal/Leg{i}", radius=0.16, height=C.BRIDGE_HEIGHT,
                     mat=self._mats["yellow"], pos=(x, 0.0, C.BRIDGE_HEIGHT / 2.0))
        box(stage, f"{self.root}/Portal/Bridge", size=1.0, mat=self._mats["yellow"],
            pos=((C.RAIL_X[0] + C.RAIL_X[1]) / 2.0, 0.0, C.BRIDGE_HEIGHT),
            scale=(C.RAIL_X[1] - C.RAIL_X[0] + 1.2, 0.55, 0.60))
        self.trolley_op = group(stage, f"{self.root}/Portal/Trolley")
        box(stage, f"{self.root}/Portal/Trolley/Body", size=1.0, mat=self._mats["gray"],
            pos=(0.0, 0.0, 0.0), scale=(0.95, 0.75, 0.50))
        self._cable = capsule(stage, f"{self.root}/Cable", radius=0.02, height=1.0, mat=self._mats["dark"])
        self.hook_op = group(stage, f"{self.root}/Hook")
        box(stage, f"{self.root}/Hook/Plate", size=1.0, mat=self._mats["gray"],
            pos=(0.0, 0.0, 0.0), scale=(0.28, 0.28, 0.08))
        cylinder(stage, f"{self.root}/Hook/Magnet", radius=0.28, height=0.12,
                 mat=self._mats["red"], pos=(0.0, 0.0, -C.MAGNET_DROP))
        print("[crane] 절차적 생성(애셋/아큘레이션 없음 - 운동학 구동)")

    # ───────────────────────── 메인 갱신 ─────────────────────────
    def update(self, dt, cmd):
        # 1) 스틱 속도 명령 적분 + 작업 범위 클램프 → 조인트 목표
        self.x = float(np.clip(self.x + cmd.lx * C.CRANE_SPEED_XY * dt, *C.CRANE_X_LIMITS))
        self.y = float(np.clip(self.y + cmd.ly * C.CRANE_SPEED_XY * dt, *C.CRANE_Y_LIMITS))
        self.z = float(np.clip(self.z + cmd.rz * C.CRANE_SPEED_Z * dt, *C.HOOK_Z_LIMITS))

        if self.has_articulation:
            self._update_articulation(dt)
        else:
            self._update_kinematic(dt, cmd)

    def _update_articulation(self, dt):
        import torch

        # 2) 위치 목표 기록+커밋(주행 y, 횡행 x, 권상 z=후크 높이-작성높이)
        target = torch.tensor(
            [[self.y, self.x, self.z - HOOK_REST_Z]], dtype=torch.float, device=self._art.device)
        self._art.set_joint_position_target(target, joint_ids=self._joint_ids)
        self._art.write_data_to_sim()

        # 3) 실제 관절 상태에서 자석 위치/속도(월드) 계산
        jp = self._art.data.joint_pos[0, self._joint_ids].detach().cpu().numpy()
        jv = self._art.data.joint_vel[0, self._joint_ids].detach().cpu().numpy()
        ax, ay, az = float(jp[1]), float(jp[0]), float(jp[2])
        hook = np.array([ax, ay, HOOK_REST_Z + az])
        self.magnet_pos = hook - np.array([0.0, 0.0, C.MAGNET_DROP])
        self.magnet_vel = np.array([float(jv[1]), float(jv[0]), float(jv[2])])

        # 4) 와이어(시각): 앵커(트롤리 하부) → 후크 판 상단
        anchor = np.array([ax, ay, C.ANCHOR_Z])
        place_segment(self._cable, anchor, hook + np.array([0.0, 0.0, 0.05]))

    def _update_kinematic(self, dt, cmd):
        # 진자(폴백 모드): 트롤리 가속도에 반응하는 감쇠 진자
        anchor = np.array([self.x, self.y, C.ANCHOR_Z])
        cable_len = max(C.ANCHOR_Z - self.z, 0.5)
        if self._prev_anchor is not None:
            vel = (anchor[:2] - self._prev_anchor[:2]) / dt
            acc = (vel - self._prev_anchor_vel) / dt
            self._prev_anchor_vel = vel
            self.thd += (-(G / cable_len) * self.th - acc / cable_len - C.SWAY_DAMPING * self.thd) * dt
            self.th += self.thd * dt
            self.th = np.clip(self.th, -C.SWAY_MAX, C.SWAY_MAX)
        self._prev_anchor = anchor

        u = np.array([self.th[0], self.th[1], -1.0])
        u /= np.linalg.norm(u)
        hook = anchor + cable_len * u
        magnet = hook + np.array([0.0, 0.0, -C.MAGNET_DROP])
        if self._prev_magnet is not None and dt > 0.0:
            self.magnet_vel = (magnet - self._prev_magnet) / dt
        self._prev_magnet = magnet.copy()
        self.magnet_pos = magnet

        set_xform(self.portal_op, (0.0, self.y, 0.0))
        set_xform(self.trolley_op, (self.x, 0.0, C.TROLLEY_Z))
        place_segment(self._cable, anchor, hook)
        set_xform(self.hook_op, hook)

    def state(self):
        """스크립트 파일럿 등에 전달할 현재 상태."""
        if self.has_articulation:
            jp = self._art.data.joint_pos[0, self._joint_ids].detach().cpu().numpy()
            return {"x": float(jp[1]), "y": float(jp[0]), "z": float(jp[2]) + HOOK_REST_Z}
        return {"x": self.x, "y": self.y, "z": self.z}
