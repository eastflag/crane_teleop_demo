"""조종실: 캐빈 + 콘솔(스틱 2개) + 패들 + 시트 + 조종 로봇(매네킹).

설계 의도(v1):
  - 캐빈은 크레인 포탈에 매달린 자식 계층 → 크레인이 Y로 이동하면 조종실도 함께 이동.
  - "로봇이 스틱을 조종한다"는 표현을, 스틱 손잡이 위치를 손이 IK로 추적하는 방식으로 구현.
    즉 신호 흐름은 [게임패드] → [스틱 각도] → [로봇 손이 스틱을 따라감 + 크레인 명령].
    물리 접촉(그립 마찰)에 의존하지 않아 씬이 흔들리지 않고 결정론적이다.
  - v2 업그레이드: 매네킹 대신 실제 로봇(Franka 팔 2대/G1 착석) + DifferentialIK,
    스틱을 스프링 복원식 아티큘레이션으로 만들어 물리적으로 밀게 하는 경로(README 참조).

좌표: 캐빈 중심이 원점. 로봇은 +Y(북쪽, 야드 쪽)를 바라본다. 바닥 z = -1.3.
"""
import numpy as np

from input_source import Command
from usd_utils import box, capsule, cylinder, group, make_material, place_segment, quat_axis_angle, set_xform, sphere

# ── 내부 치수(캐빈 중심 기준 국부 좌표) ──
# 주의: 어깨→스틱 피벗 거리가 팔 길이(0.58m)의 60~90%가 되도록 맞춰야
#       로봇 손이 실제로 스틱 손잡이에 닿는다.
CABIN_SIZE = (2.4, 2.0, 2.6)
FLOOR_Z = -CABIN_SIZE[2] / 2.0            # -1.3
PIVOT_L = np.array([-0.25, 0.50, -0.54])  # 왼쪽 스틱 피벗
PIVOT_R = np.array([0.25, 0.50, -0.54])   # 오른쪽 스틱 피벗
STICK_H = 0.10                            # 스틱 손잡이 높이
PEDAL_POS = np.array([0.15, 0.30, -1.25])  # 패들 판 중심
SHOULDER_L = np.array([-0.20, 0.08, -0.32])
SHOULDER_R = np.array([0.20, 0.08, -0.32])
ARM_UPPER, ARM_FORE = 0.30, 0.28
HIP_L = np.array([-0.11, -0.28, -0.72])
HIP_R = np.array([0.11, -0.28, -0.72])
THIGH, SHIN = 0.42, 0.42
FOOT_L_REST = np.array([-0.15, 0.15, -1.24])


def _two_bone(root, target, l1, l2, pole):
    """2본 IK: 어깨(또는 고관절) → 대상. 반환: (중간 관절 위치, 손목 위치)."""
    d = np.asarray(target, dtype=float) - np.asarray(root, dtype=float)
    dist = float(np.linalg.norm(d))
    dist = float(np.clip(dist, abs(l1 - l2) + 1e-3, (l1 + l2) * 0.99))
    dirv = d / max(float(np.linalg.norm(d)), 1e-8)
    cos_a = float(np.clip((l1 * l1 + dist * dist - l2 * l2) / (2.0 * l1 * dist), -1.0, 1.0))
    sin_a = np.sqrt(max(0.0, 1.0 - cos_a * cos_a))
    perp = pole - np.dot(pole, dirv) * dirv
    n = float(np.linalg.norm(perp))
    perp = perp / n if n > 1e-6 else np.array([0.0, 0.0, -1.0])
    mid = root + l1 * (cos_a * dirv + sin_a * perp)
    wrist = root + dist * dirv
    return mid, wrist


class CockpitRig:
    def __init__(self, stage, parent_path):
        self.root = f"{parent_path}/Cabin"
        looks = f"{self.root}/Looks"
        mats = {
            "steel":  make_material(stage, f"{looks}/Steel", (0.55, 0.58, 0.62), roughness=0.5, metallic=0.5),
            "glass":  make_material(stage, f"{looks}/Glass", (0.65, 0.75, 0.85), roughness=0.1, opacity=0.22),
            "console": make_material(stage, f"{looks}/Console", (0.18, 0.19, 0.22), roughness=0.6),
            "stick":  make_material(stage, f"{looks}/Stick", (0.10, 0.10, 0.10), roughness=0.5),
            "pedal":  make_material(stage, f"{looks}/Pedal", (0.75, 0.15, 0.10), roughness=0.5),
            "seat":   make_material(stage, f"{looks}/Seat", (0.12, 0.12, 0.14), roughness=0.8),
            "robot":  make_material(stage, f"{looks}/Robot", (0.95, 0.55, 0.10), roughness=0.45, metallic=0.3),
            "robot2": make_material(stage, f"{looks}/Robot2", (0.25, 0.26, 0.30), roughness=0.5, metallic=0.5),
        }
        self._mats = mats

        # ── 캐빈 뼈대 (포탈 기준 위치: 다리(x=1) 옆에 매달림) ──
        group(stage, self.root, pos=(0.6, -7.7, 4.35))
        sx, sy, sz = CABIN_SIZE
        box(stage, f"{self.root}/Floor", 1.0, mats["steel"], pos=(0, 0, FLOOR_Z + 0.04), scale=(sx, sy, 0.08))
        box(stage, f"{self.root}/Roof", 1.0, mats["steel"], pos=(0, 0, -FLOOR_Z - 0.04), scale=(sx, sy, 0.08))
        box(stage, f"{self.root}/BackWall", 1.0, mats["steel"], pos=(0, -sy / 2 + 0.04, 0), scale=(sx, 0.08, sz))
        box(stage, f"{self.root}/GlassFront", 1.0, mats["glass"], pos=(0, sy / 2 - 0.02, 0), scale=(sx, 0.04, sz))
        box(stage, f"{self.root}/GlassL", 1.0, mats["glass"], pos=(-sx / 2 + 0.02, 0, 0), scale=(0.04, sy, sz))
        box(stage, f"{self.root}/GlassR", 1.0, mats["glass"], pos=(sx / 2 - 0.02, 0, 0), scale=(0.04, sy, sz))
        for i, (cx, cy) in enumerate([(-1, -1), (1, -1), (-1, 1), (1, 1)]):
            box(stage, f"{self.root}/Col{i}", 1.0, mats["steel"],
                pos=(cx * (sx / 2 - 0.05), cy * (sy / 2 - 0.05), 0), scale=(0.10, 0.10, sz))

        # ── 콘솔 + 스틱 2 + 패들 ──
        box(stage, f"{self.root}/Console", 1.0, mats["console"], pos=(0, 0.50, -0.92), scale=(1.5, 0.40, 0.72))
        for name, pivot in (("L", PIVOT_L), ("R", PIVOT_R)):
            cylinder(stage, f"{self.root}/StickBase{name}", radius=0.05, height=0.05,
                     mat=mats["stick"], pos=(pivot[0], pivot[1], -0.565))
        self._shaft_l = capsule(stage, f"{self.root}/StickShaftL", radius=0.012, height=STICK_H, mat=mats["stick"])
        self._shaft_r = capsule(stage, f"{self.root}/StickShaftR", radius=0.012, height=STICK_H, mat=mats["stick"])
        self._knob_l = sphere(stage, f"{self.root}/KnobL", radius=0.028, mat=mats["robot2"], pos=PIVOT_L)
        self._knob_r = sphere(stage, f"{self.root}/KnobR", radius=0.028, mat=mats["robot2"], pos=PIVOT_R)
        box(stage, f"{self.root}/PedalBase", 1.0, mats["console"], pos=(0.15, 0.30, -1.28), scale=(0.18, 0.10, 0.03))
        self._pedal_op = box(stage, f"{self.root}/PedalPlate", 1.0, mats["pedal"],
                             pos=tuple(PEDAL_POS), scale=(0.14, 0.22, 0.02))

        # ── 시트 ──
        box(stage, f"{self.root}/SeatPan", 1.0, mats["seat"], pos=(0, -0.35, -0.78), scale=(0.45, 0.45, 0.08))
        box(stage, f"{self.root}/SeatBack", 1.0, mats["seat"], pos=(0, -0.56, -0.50), scale=(0.45, 0.08, 0.62))

        # ── 조종 로봇(매네킹): 몸통은 고정, 팔/다리만 IK ──
        self._torso = capsule(stage, f"{self.root}/Robot/Torso", radius=0.10, height=0.42, mat=mats["robot"])
        place_segment(self._torso, (0, -0.30, -0.70), (0, 0.06, -0.30))
        sphere(stage, f"{self.root}/Robot/Head", radius=0.085, mat=mats["robot2"], pos=(0, 0.10, -0.16))
        self._upper_l = capsule(stage, f"{self.root}/Robot/UpperArmL", radius=0.035, height=0.2, mat=mats["robot2"])
        self._fore_l = capsule(stage, f"{self.root}/Robot/ForeArmL", radius=0.030, height=0.2, mat=mats["robot"])
        self._upper_r = capsule(stage, f"{self.root}/Robot/UpperArmR", radius=0.035, height=0.2, mat=mats["robot2"])
        self._fore_r = capsule(stage, f"{self.root}/Robot/ForeArmR", radius=0.030, height=0.2, mat=mats["robot"])
        self._hand_l = sphere(stage, f"{self.root}/Robot/HandL", radius=0.035, mat=mats["robot"], pos=PIVOT_L)
        self._hand_r = sphere(stage, f"{self.root}/Robot/HandR", radius=0.035, mat=mats["robot"], pos=PIVOT_R)
        self._thigh_l = capsule(stage, f"{self.root}/Robot/ThighL", radius=0.05, height=0.3, mat=mats["robot"])
        self._shin_l = capsule(stage, f"{self.root}/Robot/ShinL", radius=0.04, height=0.3, mat=mats["robot2"])
        self._thigh_r = capsule(stage, f"{self.root}/Robot/ThighR", radius=0.05, height=0.3, mat=mats["robot"])
        self._shin_r = capsule(stage, f"{self.root}/Robot/ShinR", radius=0.04, height=0.3, mat=mats["robot2"])

        # 초기 자세 1회 배치
        self.update(1.0 / 120.0, Command())

    # ───────────────────────── 메인 갱신 ─────────────────────────
    def update(self, dt, cmd: Command):
        # 1) 스틱: 명령 → 손잡이 위치 (기울기 비율 0.55)
        knob_l = PIVOT_L + STICK_H * self._tilt_dir(cmd.lx, cmd.ly)
        knob_r = PIVOT_R + STICK_H * self._tilt_dir(0.0, cmd.rz)
        place_segment(self._shaft_l, PIVOT_L, knob_l)
        place_segment(self._shaft_r, PIVOT_R, knob_r)
        set_xform(self._knob_l, knob_l)
        set_xform(self._knob_r, knob_r)

        # 2) 패들: 밟으면 앞쪽으로 기울어짐
        ang = float(np.clip(cmd.pedal, 0.0, 1.0)) * 0.5
        set_xform(self._pedal_op, tuple(PEDAL_POS), quat_axis_angle((1.0, 0.0, 0.0), -ang))
        pedal_top = PEDAL_POS + np.array([0.0, 0.02, 0.04 - 0.06 * ang])

        # 3) 로봇 팔: 손이 각 스틱 손잡이를 추적 (팔꿈치는 아래+바깥)
        for side, shoulder, knob, upper, fore, hand in (
            ("L", SHOULDER_L, knob_l, self._upper_l, self._fore_l, self._hand_l),
            ("R", SHOULDER_R, knob_r, self._upper_r, self._fore_r, self._hand_r),
        ):
            pole = np.array([-0.4 if side == "L" else 0.4, 0.1, -1.0])
            elbow, wrist = _two_bone(shoulder, knob, ARM_UPPER, ARM_FORE, pole)
            place_segment(upper, shoulder, elbow)
            place_segment(fore, elbow, wrist)
            set_xform(hand, wrist)

        # 4) 로봇 다리: 왼발 정지 자세, 오른발은 패들 위
        knee_l, ankle_l = _two_bone(HIP_L, FOOT_L_REST, THIGH, SHIN, np.array([0.0, 0.5, 0.3]))
        place_segment(self._thigh_l, HIP_L, knee_l)
        place_segment(self._shin_l, knee_l, ankle_l)
        knee_r, ankle_r = _two_bone(HIP_R, pedal_top, THIGH, SHIN, np.array([0.0, 0.5, 0.3]))
        place_segment(self._thigh_r, HIP_R, knee_r)
        place_segment(self._shin_r, knee_r, ankle_r)

    @staticmethod
    def _tilt_dir(x, y):
        v = np.array([x * 0.55, y * 0.55, 1.0])
        return v / np.linalg.norm(v)
