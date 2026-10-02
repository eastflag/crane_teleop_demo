"""조종실: 회색 부스 외관 + 사무용 의자(등받이·팔걸이) + 조이스틱 2 + 페달 2 + 착석 로봇.

설계 의도(PoC):
  - 외관은 drawing/cockpit1_flat.png 참고: 회색 밀폐 부스, 앞면이 뒤로 기운 큰 창,
    왼쪽 벽 모니터. 그림의 로봇/사람은 무시.
  - 의자는 등받이 + 좌우 손받침대(팔걸이)가 있는 사무용 의자. 팔걸이 위에 주황색
    조이스틱 2개, 의자 바로 앞 바닥에 주황색 페달 2개(그림에서 주황색 = 조작 장치).
  - 매네킹 대신 raise_a3_ultra_t3d0(AgiBot A3 Ultra) 로봇이 의자에 앉는다.
    인터페이스 USD(Physics=physx 변형)를 reference로 가져와 리지드바디+관절
    드라이브가 구성되고, 물리 레이어의 관절 프레임으로 순운동학(FK)을 계산한
    착석 포즈를 초기 자세+드라이브 목표로 굽는다. 로봇은 허벅지가 시트에
    얹히는 높이로 스폰되어 중력에 의해 의자에 안착한다(시트·바닥은 정적 충돌체).
  - 캐빈은 월드에 고정된 독립 계층 → 크레인이 움직여도 조종실은 정지한다.
  - 캐빈 구조물은 USD 애셋(assets/cockpit.usd)으로 분리하고, 런타임에는
    reference로 로드만 한다. 애셋이 없으면 절차적 생성으로 폴백.
    재생성: ./isaaclab.sh -p crane_teleop_demo/make_cockpit_asset.py

좌표: 캐빈 중심이 원점. 의자는 +Y(창 쪽)를 바라본다. 바닥 z = -1.3.
"""
import math
import re
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

from input_source import Command
from usd_utils import (
    attach_capsule, attach_op, box, capsule, cylinder, get_op, group,
    make_material, place_segment, quat_axis_angle, set_xform, sphere,
)

DEFAULT_ASSET = Path(__file__).resolve().parent / "assets" / "cockpit.usd"
"""조종실 USD 애셋 기본 경로(없으면 절차적 생성으로 폴백)."""

ROBOT_IFACE = Path(__file__).resolve().parent / "raise_a3_ultra_t3d0" / "raise_a3_ultra_t3d0.usda"
"""A3 로봇 인터페이스(구조 3.0). Physics 변형 기본값이 physx라 강체·관절 드라이브가
함께 구성된다 - 로봇은 리지드바디로 시뮬레이션되어 중력으로 의자에 안착한다."""

ROBOT_BASE = ROBOT_IFACE.parent / "payloads" / "base.usda"
"""로봇 시각 계층(물리 없음) - FK 계측 읽기와 폴백 참조용."""

ROBOT_PHYSICS = ROBOT_BASE.parent / "Physics" / "physics.usda"
"""관절 프레임(축·본체 연결)을 읽는 물리 레이어 - FK 계산에만 사용."""

SIT_RAISE = 0.5
"""착석 상승량: 발이 아닌 허벅지 밑면이 시트에 얹히도록 골반을 올리는 높이."""

CABIN_POS = (8.2, -7.7, 4.35)
"""데모에서 캐빈을 놓는 월드 위치(크레인 가운데 x=8.2, 남쪽 레일 바깥)."""

# ── 내부 치수(캐빈 중심 기준 국부 좌표) ──
CABIN_SIZE = (2.4, 2.0, 2.6)
FLOOR_Z = -CABIN_SIZE[2] / 2.0            # -1.3 (바닥 슬래브 아랫면 기준 캐빈 바닥)
FLOOR_T = 0.08                            # 바닥 슬래브 두께
FLOOR_TOP = FLOOR_Z + FLOOR_T             # 바닥 윗면(-1.22): 발·페달은 이면 위에 놓인다
STICK_H = 0.10                            # 스틱 손잡이 높이
SOLE_T = 0.06                             # 발목 → 발바닥 두께

# 페달: 발 하나 크기(가로 15cm × 세로 30cm × 두께 1cm).
# 안식 상태 = 뒤엣지(A, 로봇 쪽)는 바닥에 붙고 앞엣지(B, 먼 쪽)가 PEDAL_LIFT 만큼
# 부상(30cm 판에서 약 9.6°). 밟으면 B가 내려와 A와 같은 높이(판이 수평)가 된다.
PEDAL_SIZE = (0.15, 0.30, 0.01)
PEDAL_LIFT = 0.05
PEDAL_TILT_REST = math.asin(PEDAL_LIFT / PEDAL_SIZE[1])

# 로봇이 없을 때 쓰는 기본 착석 수치(로봇이 있으면 FK로 계산해 덮어쓴다)
DEFAULT_FIT = {"pelvis_y": -0.05, "pelvis_z": FLOOR_TOP + 0.65, "knee_dy": 0.41}

# 착석 포즈(라디안). 관절 축/방향은 물리 레이어 기준으로 검증됨.
SIT_ANGLES = {
    "left_hip_pitch_joint": -math.pi / 2,
    "right_hip_pitch_joint": -math.pi / 2,
    # 무릎 60° → 정강이가 수직에서 앞쪽으로 30° 기울어 발이 전방으로 뻗는다.
    # 낙하 시 발끝이 아닌 편평한 발바닥으로 착지하게 발목 +30° 보상.
    "left_knee_joint": math.pi / 3,
    "right_knee_joint": math.pi / 3,
    "left_ankle_pitch_joint": math.pi / 6,
    "right_ankle_pitch_joint": math.pi / 6,
    "left_shoulder_pitch_joint": -0.8,    # 팔을 앞으로 들어 팔걸이 스틱 쪽으로
    "right_shoulder_pitch_joint": -0.8,
}


# ═══════════════════════ 로봇 착석 포즈(FK) ═══════════════════════
# 관례: 이 모듈의 numpy 행렬은 열벡터 관례(x' = R@x + t, 이동은 마지막 열).
# pxr Gf 행렬은 행벡터 관례(이동이 마지막 행)이므로 경계마다 전치한다.
def _np_mat(m):
    """Gf.Matrix4d(행벡터 관례) → numpy 4x4(열벡터 관례)."""
    return np.array([[m[i][j] for j in range(4)] for i in range(4)]).T


def _gf_mat(m):
    """numpy 4x4(열벡터 관례) → Gf.Matrix4d(행벡터 관례)."""
    return Gf.Matrix4d(*[float(v) for row in m.T for v in row])


def _prim_local_mat(prim):
    """링크 프림의 국부 행렬. 로봇 링크는 translate/orient/scale TRS 옵 구성
    (스케일 1)이므로 속성에서 직접 T·R 를 만든다."""
    t = prim.GetAttribute("xformOp:translate").Get() or Gf.Vec3d(0.0, 0.0, 0.0)
    q = prim.GetAttribute("xformOp:orient").Get() or Gf.Quatd(1.0, 0.0, 0.0, 0.0)
    g = Gf.Matrix4d().SetRotate(Gf.Rotation().SetQuat(Gf.Quatd(q)))
    g.SetTranslateOnly(Gf.Vec3d(t))
    return _np_mat(g)


def _parse_joints():
    """physics.usda에서 관절 목록. 회전 관절과 고정 관절 모두 읽는다.

    고정 관절(셸 커버·손바닥·손가락 팁 등)은 각도가 없지만 트리 연결이 필요하다 -
    빠뜨리면 부모 링크가 회전해도 자식이 제자리에 남아 착석 포즈가 분리돼 보인다.
    """
    text = ROBOT_PHYSICS.read_text()
    joints = []
    blocks = []
    for kind in ("RevoluteJoint", "FixedJoint"):
        blocks.extend(text.split(f'def Physics{kind} "')[1:])
    for block in blocks:
        name = block.split('"', 1)[0]
        def grab(pat, cast=float):
            mm = re.search(pat, block)
            return cast(mm.group(1)) if mm else None

        axis = grab(r'physics:axis = "(\w+)"', str)
        b0 = grab(r'physics:body0 = <([^>]+)>', str)
        b1 = grab(r'physics:body1 = <([^>]+)>', str)
        pos = re.search(r'physics:localPos1 = \(([^)]*)\)', block)
        rot = re.search(r'physics:localRot1 = \(([^)]*)\)', block)
        p = tuple(float(v) for v in pos.group(1).split(",")) if pos else (0.0, 0.0, 0.0)
        q = tuple(float(v) for v in rot.group(1).split(",")) if rot else (1.0, 0.0, 0.0, 0.0)
        joints.append({"name": name, "axis": axis, "b0": b0, "b1": b1, "pos1": p, "quat1": q})
    return joints


def _rot_about(axis, angle):
    """축 단위벡터 축 회전 행렬(4x4)."""
    ax = {"X": (1.0, 0.0, 0.0), "Y": (0.0, 1.0, 0.0), "Z": (0.0, 0.0, 1.0)}[axis]
    c, s = math.cos(angle), math.sin(angle)
    x, y, z = ax
    return np.array([
        [c + x * x * (1 - c), x * y * (1 - c) - z * s, x * z * (1 - c) + y * s, 0.0],
        [y * x * (1 - c) + z * s, c + y * y * (1 - c), y * z * (1 - c) - x * s, 0.0],
        [z * x * (1 - c) - y * s, z * y * (1 - c) + x * s, c + z * z * (1 - c), 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])


def _quat_mat(q):
    """쿼터니언(w,x,y,z) → 회전 행렬(4x4)."""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y), 0.0],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x), 0.0],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y), 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])


def _tf_mat(quat, pos):
    """회전 + 이동 → 4x4."""
    m = _quat_mat(quat)
    m[:3, 3] = pos
    return m


def add_sitting_robot(stage, parent_path, pelvis_y=-0.05):
    """parent 아래에 A3 로봇(physx 인터페이스)을 reference로 추가하고 착석 포즈를 구운다.

    - 링크 transform 오버라이드: FK 착석 포즈 = 시뮬레이션 초기 자세
    - 관절 드라이브 목표 오버라이드: 초기 자세를 홀딩(단위: 도)
    - 배치: 허벅지 밑면이 시트에 닿도록 골반을 SIT_RAISE 만큼 올림 →
      리지드바디가 중력에 의해 시트 위에 안착한다(시트/바닥은 정적 충돌체).
    반환: fit dict(의자 치수용 계측값) 또는 로봇 없으면 None.
    주의: reference는 참조 대상(루트 Xform)을 참조하는 프림 자체에 겹쳐 맵핑되므로
    링크들은 {parent}/Robot 의 직속 자식으로 나타난다.
    """
    if not ROBOT_IFACE.is_file():
        print(f"[cockpit] 로봇 애셋 없음({ROBOT_IFACE}) - 의자만 생성")
        return None

    robot_path = f"{parent_path}/Robot"
    prim = stage.DefinePrim(robot_path, "Xform")
    prim.GetReferences().AddReference(str(ROBOT_IFACE))

    # 1) 링크별 국부 변환(기본 자세) 읽기 - 링크는 참조 프림 아래 플랫 구성
    mats = {}
    for child in prim.GetChildren():
        if child.GetTypeName() == "Xform":
            mats[child.GetName()] = _prim_local_mat(child)

    # 2) 관절 트리(자식 링크 → 관절) 구성
    joints = _parse_joints()
    joint_of_child = {j["b1"].split("/")[-1]: j for j in joints}

    # 3) 관절 회전(루트 프레임 고정축): R = P_body1 · T(localPos1, localRot1) · Rot(axis, θ) · ...
    rot = {}
    for j in joints:
        child_name = j["b1"].split("/")[-1]
        if j["name"] in SIT_ANGLES and child_name in mats:
            fj = mats[child_name] @ _tf_mat(j["quat1"], j["pos1"])
            rot[child_name] = fj @ _rot_about(j["axis"], SIT_ANGLES[j["name"]]) @ np.linalg.inv(fj)

    # 4) 각 링크: 경로 위 관절(자기를 직접 달고 있는 관절 포함)의 회전을
    #    얕은 관절이 가장 왼쪽(포인트에 마지막 적용)에 오도록 곱해 오버라이드로 기록
    new_mats = {}
    for name, m in mats.items():
        chain = []  # 깊→얕
        cur = name
        while cur in joint_of_child:
            if cur in rot:  # cur 링크를 직접 달고 있는 관절의 각도
                chain.append(rot[cur])
            cur = joint_of_child[cur]["b0"].split("/")[-1]
        out = m
        for r in chain:  # 깊은 관절부터 왼쪽에 곱해 얕은 관절이 가장 왼쪽
            out = r @ out
        new_mats[name] = out

        g = _gf_mat(out)
        t = g.ExtractTranslation()
        quat = g.ExtractRotation().GetQuat()
        lp = stage.GetPrimAtPath(f"{robot_path}/{name}")
        lp.GetAttribute("xformOp:translate").Set(Gf.Vec3d(*t))
        lp.GetAttribute("xformOp:orient").Set(Gf.Quatd(quat.real, *quat.imaginary))

    # 5) 의자 치수용 계측(로봇 국부 좌표: +X=정면, 골반=원점)
    knee = new_mats["left_knee_Link"][:3, 3]
    ankle = new_mats["left_ankle_roll_Link"][:3, 3]
    wrist = new_mats["left_wrist_yaw_Link"][:3, 3]
    # 허벅지 밑면(고관절 아래 + 허벅지 반지름)이 시트에 얹히는 높이 + 여유 상승.
    # 발은 바닥에 닿지 않고 매달리며, 리지드바디가 중력으로 시트에 안착한다.
    pelvis_z = FLOOR_TOP + (-ankle[2] + SOLE_T) + SIT_RAISE

    # 6) 배치: 국부 +X(정면) → 캐빈 +Y(창 쪽). 참조된 루트 op를 오버라이드한다.
    yaw = math.sqrt(0.5)
    prim.GetAttribute("xformOp:translate").Set(Gf.Vec3d(0.0, pelvis_y + 0.35, pelvis_z))
    prim.GetAttribute("xformOp:orient").Set(Gf.Quatd(yaw, 0.0, 0.0, yaw))

    # 7) 관절 드라이브 목표를 착석 각도로 오버라이드(도 단위).
    #    구운 초기 자세와 일치시켜 스텝 시작 시 스냅 없이 포즈를 홀딩한다.
    #    관절 프림은 body0 링크 아래에 있다(예: pelvis_link/left_hip_pitch_joint).
    n_drives = 0
    for j in joints:
        if j["name"] in SIT_ANGLES:
            jp = stage.GetPrimAtPath(f"{robot_path}/{j['b0'].split('/')[-1]}/{j['name']}")
            attr = jp.GetAttribute("drive:angular:physics:targetPosition") if jp.IsValid() else None
            if attr is not None and attr:
                attr.Set(math.degrees(SIT_ANGLES[j["name"]]))
                n_drives += 1
    print(f"[cockpit] 로봇 물리 구성: 강체(physx) + 드라이브 홀딩 {n_drives}개 관절")

    fit = {
        "pelvis_y": pelvis_y,
        "pelvis_z": pelvis_z,
        "knee_dy": float(knee[0]),
        "diagnostics": {
            "knee": knee.round(3).tolist(), "ankle": ankle.round(3).tolist(),
            "wrist": wrist.round(3).tolist(),
        },
    }
    return fit


# ═══════════════════════ 조종실 릭 ═══════════════════════
class CockpitRig:
    def __init__(self, stage, parent_path, usd_asset="auto", root_pos=CABIN_POS):
        """usd_asset: "auto"(애셋 있으면 로드, 없으면 생성) | 경로(강제 로드) | None(강제 생성)."""
        self.root = f"{parent_path}/Cabin"
        if usd_asset == "auto":
            usd_asset = str(DEFAULT_ASSET) if DEFAULT_ASSET.is_file() else None
        if usd_asset:
            self._attach(stage, usd_asset, root_pos)
        else:
            self._build(stage, root_pos)

    # ─────────────── 애셋 로드 모드: reference + 갱신 핸들만 다시 묶기 ───────────────
    def _attach(self, stage, usd_asset, root_pos):
        root_prim = UsdGeom.Xform.Define(stage, self.root).GetPrim()
        root_prim.GetReferences().AddReference(usd_asset)
        set_xform(get_op(root_prim, create=True), root_pos)  # 위치는 참조하는 쪽에서 부여

        def p(name):  # 애셋 내부 프림 경로(_build와 이름 동일)
            return f"{self.root}/{name}"

        self._shaft_l = attach_capsule(stage, p("StickShaftL"))
        self._shaft_r = attach_capsule(stage, p("StickShaftR"))
        self._knob_l = attach_op(stage, p("KnobL"))
        self._knob_r = attach_op(stage, p("KnobR"))
        self._pedal_l = attach_op(stage, p("PedalPlateL"))
        self._pedal_r = attach_op(stage, p("PedalPlateR"))

        # 중립일 때 손잡이는 피벗 위 STICK_H 지점에 저장돼 있고, 페달 판 위치도
        # 그대로 저장돼 있으므로 애셋에서 읽어 갱신 수학에 필요한 기준점을 복구한다.
        def local_pos(name):
            m = get_op(stage.GetPrimAtPath(p(name))).Get()  # 매트릭스 op(로컬)
            return np.array([m[3][0], m[3][1], m[3][2]])

        knob_l0, knob_r0 = local_pos("KnobL"), local_pos("KnobR")
        self._pivot = {"L": knob_l0 - np.array([0.0, 0.0, STICK_H]),
                       "R": knob_r0 - np.array([0.0, 0.0, STICK_H])}
        self._pedal_pos = {"L": local_pos("PedalPlateL"), "R": local_pos("PedalPlateR")}
        self.update(1.0 / 120.0, Command())  # 초기 자세 배치
        print(f"[cockpit] USD 애셋 로드: {usd_asset}")

    # ─────────────── 절차적 생성 모드(애셋 생성 스크립트가 사용) ───────────────
    def _build(self, stage, root_pos):
        looks = f"{self.root}/Looks"
        mats = {
            "shell":  make_material(stage, f"{looks}/Shell",  (0.62, 0.64, 0.68), roughness=0.7, metallic=0.2),
            "glass":  make_material(stage, f"{looks}/Glass",  (0.65, 0.75, 0.85), roughness=0.1, opacity=0.25),
            "seat":   make_material(stage, f"{looks}/Seat",   (0.14, 0.14, 0.16), roughness=0.8),
            "orange": make_material(stage, f"{looks}/Orange", (0.90, 0.42, 0.06), roughness=0.45, metallic=0.2),
        }
        self._mats = mats

        # ── 착석 로봇(reference + FK 착석 포즈 + 배치) ──
        fit = add_sitting_robot(stage, self.root) or dict(DEFAULT_FIT)
        pz, py, knee_dy = fit["pelvis_z"], fit["pelvis_y"], fit["knee_dy"]

        # ── 외관: 회색 부스 + 뒤로 기운 전면 창(도면) ──
        group(stage, self.root, pos=root_pos)
        sx, sy, sz = CABIN_SIZE
        box(stage, f"{self.root}/Floor", 1.0, mats["shell"], pos=(0, 0, FLOOR_Z + 0.04), scale=(sx, sy, 0.08))
        # 지붕·좌우는 투명 유리 - 안쪽(의자·로봇·조작장치)이 보인다
        box(stage, f"{self.root}/Roof", 1.0, mats["glass"], pos=(0, 0, -FLOOR_Z - 0.04), scale=(sx, sy, 0.08))
        box(stage, f"{self.root}/BackWall", 1.0, mats["shell"], pos=(0, -sy / 2 + 0.04, 0), scale=(sx, 0.08, sz))
        box(stage, f"{self.root}/SideL", 1.0, mats["glass"], pos=(-sx / 2 + 0.04, 0, 0), scale=(0.08, sy, sz))
        box(stage, f"{self.root}/SideR", 1.0, mats["glass"], pos=(sx / 2 - 0.04, 0, 0), scale=(0.08, sy, sz))
        # 전면: 아래 회색 스커트 + 위로 갈수록 뒤로 물리는 경사 유리(도면의 앞면)
        box(stage, f"{self.root}/FrontSkirt", 1.0, mats["shell"], pos=(0, sy / 2 - 0.04, FLOOR_Z + 0.25), scale=(sx, 0.08, 0.5))
        tilt = math.atan2(0.55, sz)                       # 약 12° 후경사
        glass_len = math.hypot(0.55, sz - 0.6)
        box(stage, f"{self.root}/FrontGlass", 1.0, mats["glass"],
            pos=(0, sy / 2 - 0.30, 0.35), quat=quat_axis_angle((1.0, 0.0, 0.0), -tilt),
            scale=(sx - 0.1, 0.04, glass_len))

        # ── 사무용 의자: 시트 + 등받이 + 팔걸이(스틱 베이스 포함) ──
        # 시트 높이는 상승 전(SIT_RAISE 제외) 골반 기준 - 로봇을 20cm 올린 결과
        # 허벅지 밑면이 시트 위면에 살짝 얹혀 중력으로 안착한다.
        seat_top = pz - SIT_RAISE - 0.2   # 의자 누적 15cm 낮춤
        knee_y = py + knee_dy
        # 시트: 뒷단(py-0.18) 고정, 앞쪽은 무릎보다 14cm 남김(길이 0.44m)
        box(stage, f"{self.root}/SeatPan", 1.0, mats["seat"],
            pos=(0, (py - 0.18 + knee_y - 0.14) / 2, seat_top - 0.03), scale=(0.46, knee_y - 0.14 - (py - 0.18), 0.06))
        box(stage, f"{self.root}/SeatPost", 1.0, mats["seat"], pos=(0, py, (FLOOR_TOP + seat_top) / 2), scale=(0.08, 0.08, seat_top - FLOOR_TOP))
        box(stage, f"{self.root}/BackRest", 1.0, mats["seat"],
            pos=(0, py - 0.26, seat_top + 0.36), quat=quat_axis_angle((1.0, 0.0, 0.0), 0.12), scale=(0.44, 0.07, 0.74))
        box(stage, f"{self.root}/BasePlate", 1.0, mats["seat"], pos=(0, py, FLOOR_TOP + 0.015), scale=(0.5, 0.5, 0.03))
        # 로봇 리지드바디가 앉는 정적 충돌체(시트·등받이·바닥)
        for name in ("SeatPan", "BackRest", "Floor"):
            UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(f"{self.root}/{name}"))

        arm_z = seat_top + 0.25
        # 거치대: 길이 앞으로 1.5배(0.69m, 뒷단 고정), 폭 좌우 2배(0.20m)
        for side, x_arm in (("L", -0.36), ("R", 0.36)):  # 좌우 5cm씩 벌림
            box(stage, f"{self.root}/ArmRest{side}", 1.0, mats["seat"],
                pos=(x_arm, py + 0.145, arm_z - 0.025), scale=(0.20, 0.69, 0.05))
            box(stage, f"{self.root}/ArmPost{side}", 1.0, mats["seat"],
                pos=(x_arm, py - 0.14, (seat_top + arm_z) / 2), scale=(0.05, 0.05, arm_z - seat_top))

        # ── 조이스틱 2(거치대 위, 주황) - 거치대 앞쪽(피벗 y = py+0.35) ──
        self._pivot = {"L": np.array([-0.36, py + 0.35, arm_z]), "R": np.array([0.36, py + 0.35, arm_z])}  # 거치대와 같은 간격(±0.36)
        for side in ("L", "R"):
            pivot = self._pivot[side]
            cylinder(stage, f"{self.root}/StickBase{side}", radius=0.045, height=0.035,
                     mat=mats["orange"], pos=tuple(pivot + np.array([0, 0, 0.018])))
        self._shaft_l = capsule(stage, f"{self.root}/StickShaftL", radius=0.012, height=STICK_H, mat=mats["orange"])
        self._shaft_r = capsule(stage, f"{self.root}/StickShaftR", radius=0.012, height=STICK_H, mat=mats["orange"])
        self._knob_l = sphere(stage, f"{self.root}/KnobL", radius=0.030, mat=mats["orange"], pos=tuple(self._pivot["L"]))
        self._knob_r = sphere(stage, f"{self.root}/KnobR", radius=0.030, mat=mats["orange"], pos=tuple(self._pivot["R"]))

        # ── 페달 2(의자 바로 앞, 주황) - 뒤엣지(A)는 바닥, 앞엣지(B) 부상 안식각.
        #    판 중심 높이도 기울기에 맞춰 잡아 뒤엣지가 항상 바닥에 붙게 한다. ──
        _theta0 = PEDAL_TILT_REST
        _pedal_z = FLOOR_TOP + 0.5 * PEDAL_SIZE[2] + 0.5 * PEDAL_SIZE[1] * math.sin(_theta0)
        self._pedal_pos = {"L": np.array([-0.13, knee_y + 0.26, _pedal_z]),
                           "R": np.array([0.13, knee_y + 0.26, _pedal_z])}
        for side in ("L", "R"):
            pos = self._pedal_pos[side]
            box(stage, f"{self.root}/PedalBase{side}", 1.0, mats["seat"],
                pos=(pos[0], pos[1] - 0.16, FLOOR_TOP + 0.010), scale=(0.14, 0.05, 0.02))
        rest_quat = quat_axis_angle((1.0, 0.0, 0.0), _theta0)
        self._pedal_l = box(stage, f"{self.root}/PedalPlateL", 1.0, mats["orange"],
                            pos=tuple(self._pedal_pos["L"]), quat=rest_quat, scale=PEDAL_SIZE)
        self._pedal_r = box(stage, f"{self.root}/PedalPlateR", 1.0, mats["orange"],
                            pos=tuple(self._pedal_pos["R"]), quat=rest_quat, scale=PEDAL_SIZE)

        # 초기 자세 1회 배치
        self.update(1.0 / 120.0, Command())
        print("[cockpit] 절차적 생성(USD 애셋 없음)")
        if "diagnostics" in fit:
            print(f"[cockpit] 로봇 착석 계측: {fit['diagnostics']}")

    # ───────────────────────── 메인 갱신 ─────────────────────────
    def update(self, dt, cmd: Command):
        # 1) 스틱: 명령 → 손잡이 위치 (기울기 비율 0.55). 로봇은 정적(PoC).
        knob_l = self._pivot["L"] + STICK_H * self._tilt_dir(cmd.lx, cmd.ly)
        knob_r = self._pivot["R"] + STICK_H * self._tilt_dir(0.0, cmd.rz)
        place_segment(self._shaft_l, self._pivot["L"], knob_l)
        place_segment(self._shaft_r, self._pivot["R"], knob_r)
        set_xform(self._knob_l, knob_l)
        set_xform(self._knob_r, knob_r)

        # 2) 페달 2: 안식 = 앞엣지(B) 부상, 답압 = B가 내려와 뒤엣지(A)와 같은
        #    높이(수평)로. 판은 뒤엣지를 축으로 회전하므로 중심 높이도 함께 조정.
        #    주의: set_xform이 스케일을 덮어쓰므로 PEDAL_SIZE를 항상 함께 넘긴다.
        press = float(np.clip(cmd.pedal, 0.0, 1.0))
        theta = PEDAL_TILT_REST * (1.0 - press)
        quat = quat_axis_angle((1.0, 0.0, 0.0), theta)
        z = FLOOR_TOP + 0.5 * PEDAL_SIZE[2] + 0.5 * PEDAL_SIZE[1] * math.sin(theta)
        set_xform(self._pedal_l, (self._pedal_pos["L"][0], self._pedal_pos["L"][1], z), quat, PEDAL_SIZE)
        set_xform(self._pedal_r, (self._pedal_pos["R"][0], self._pedal_pos["R"][1], z), quat, PEDAL_SIZE)

    @staticmethod
    def _tilt_dir(x, y):
        v = np.array([x * 0.55, y * 0.55, 1.0])
        return v / np.linalg.norm(v)
