"""갠트리 크레인: 포탈(Y 이동) + 트롤리(X 이동) + 권상(Z) + 해석적 진자 후크.

v1 설계: 크레인 자체는 물리 바디가 아닌 운동학(USD 프림 갱신)으로 구동한다.
  - 안정성: 조이스틱 속도 명령이 그대로 기구에 적용되므로 시뮬레이션 튜닝이 필요 없다.
  - 하중 흔들림(진자)은 감쇠 진자 방정식으로 해석적 모사 → 시각적으로 사실적.
  - 실제 물리(중력·충돌)는 철근에만 적용(yard.py) → 자석 부착/낙하가 물리로 일어난다.
v2 업그레이드 경로: 프리즘/회전 조인트 + 액추에이터 아티큘레이션으로 교체(README 참조).
"""
import numpy as np

import config as C
from input_source import Command
from usd_utils import box, capsule, cylinder, group, make_material, place_segment, set_xform

G = 9.81


class GantryCrane:
    def __init__(self, stage):
        self.root = "/World/Crane"
        looks = f"{self.root}/Looks"
        self._mats = {
            "yellow": make_material(stage, f"{looks}/Yellow", (0.85, 0.66, 0.10), roughness=0.5, metallic=0.3),
            "gray":   make_material(stage, f"{looks}/Gray",   (0.30, 0.32, 0.36), roughness=0.6, metallic=0.4),
            "dark":   make_material(stage, f"{looks}/Dark",   (0.12, 0.12, 0.14), roughness=0.7),
            "red":    make_material(stage, f"{looks}/Red",    (0.80, 0.15, 0.12), roughness=0.4, metallic=0.5),
        }

        # ── 지상 레일 (월드 고정, Y 방향) ──
        for i, x in enumerate(C.RAIL_X):
            box(stage, f"{self.root}/Rail{i}", size=1.0, mat=self._mats["dark"],
                pos=(x, 0.0, 0.07), scale=(0.40, C.RAIL_LEN_Y + 1.0, 0.14))

        # ── 포탈(다리 2 + 브리지) — 루트가 Y로 이동 ──
        self.portal_op = group(stage, f"{self.root}/Portal")
        for i, x in enumerate(C.RAIL_X):
            cylinder(stage, f"{self.root}/Portal/Leg{i}", radius=0.16, height=C.BRIDGE_HEIGHT,
                     mat=self._mats["yellow"], pos=(x, 0.0, C.BRIDGE_HEIGHT / 2.0))
        box(stage, f"{self.root}/Portal/Bridge", size=1.0, mat=self._mats["yellow"],
            pos=((C.RAIL_X[0] + C.RAIL_X[1]) / 2.0, 0.0, C.BRIDGE_HEIGHT),
            scale=(C.RAIL_X[1] - C.RAIL_X[0] + 1.2, 0.55, 0.60))
        # 사선 브레이스(외관)
        box(stage, f"{self.root}/Portal/Brace1", size=1.0, mat=self._mats["yellow"],
            pos=(C.RAIL_X[0], 0.0, 7.0), quat=(0.9239, 0.0, 0.3827, 0.0), scale=(8.6, 0.10, 0.10))
        box(stage, f"{self.root}/Portal/Brace2", size=1.0, mat=self._mats["yellow"],
            pos=(C.RAIL_X[0], 0.0, 7.0), quat=(0.9239, 0.0, -0.3827, 0.0), scale=(8.6, 0.10, 0.10))
        # 조종실(cockpit.py)은 Portal 아래에 추가된다.

        # ── 트롤리 (포탈 기준 X 이동) ──
        self.trolley_op = group(stage, f"{self.root}/Portal/Trolley")
        box(stage, f"{self.root}/Portal/Trolley/Body", size=1.0, mat=self._mats["gray"],
            pos=(0.0, 0.0, 0.0), scale=(0.95, 0.75, 0.50))

        # ── 와이어(앵커→후크, 가변 길이)와 후크+자석 (월드 좌표) ──
        self._cable = capsule(stage, f"{self.root}/Cable", radius=0.02, height=1.0, mat=self._mats["dark"])
        self.hook_op = group(stage, f"{self.root}/Hook")
        box(stage, f"{self.root}/Hook/Plate", size=1.0, mat=self._mats["gray"],
            pos=(0.0, 0.0, 0.0), scale=(0.28, 0.28, 0.08))
        cylinder(stage, f"{self.root}/Hook/Magnet", radius=0.28, height=0.12,
                 mat=self._mats["red"], pos=(0.0, 0.0, -C.MAGNET_DROP))

        # ── 상태 ──
        self.x, self.y, self.z = C.CRANE_START          # z = 후크 목표 높이
        self.th = np.zeros(2)                            # 진자 각 (θx, θy)
        self.thd = np.zeros(2)
        self.magnet_pos = np.zeros(3)
        self.magnet_vel = np.zeros(3)
        self._prev_anchor = None
        self._prev_anchor_vel = np.zeros(2)
        self._prev_magnet = None
        self.update(1.0 / 120.0, Command())  # 초기 배치

    # ───────────────────────── 메인 갱신 ─────────────────────────
    def update(self, dt, cmd):
        # 1) 스틱 속도 명령 적분 + 작업 범위 클램프
        self.x = float(np.clip(self.x + cmd.lx * C.CRANE_SPEED_XY * dt, *C.CRANE_X_LIMITS))
        self.y = float(np.clip(self.y + cmd.ly * C.CRANE_SPEED_XY * dt, *C.CRANE_Y_LIMITS))
        self.z = float(np.clip(self.z + cmd.rz * C.CRANE_SPEED_Z * dt, *C.HOOK_Z_LIMITS))

        # 2) 진자: 트롤리(앵커) 가속도에 반응하는 감쇠 진자
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

        # 3) 후크/자석 위치 = 앵커 + L * 기울기 방향
        u = np.array([self.th[0], self.th[1], -1.0])
        u /= np.linalg.norm(u)
        hook = anchor + cable_len * u
        magnet = hook + np.array([0.0, 0.0, -C.MAGNET_DROP])
        if self._prev_magnet is not None and dt > 0.0:
            self.magnet_vel = (magnet - self._prev_magnet) / dt
        self._prev_magnet = magnet.copy()
        self.magnet_pos = magnet

        # 4) USD 프림 갱신
        set_xform(self.portal_op, (0.0, self.y, 0.0))
        set_xform(self.trolley_op, (self.x, 0.0, C.TROLLEY_Z))  # Portal 루트는 원점(0, y, 0)이라 z는 월드와 동일
        place_segment(self._cable, anchor, hook)
        set_xform(self.hook_op, hook)

    def state(self):
        """스크립트 파일럿 등에 전달할 현재 상태."""
        return {"x": self.x, "y": self.y, "z": self.z}
