"""조종실(USD 애셋) 로드 + 조이스틱 아큘레이션 구동 + 페달 갱신.

애셋(assets/cockpit.usd) 내용 - 로드만 하고 스크립트로 생성하지 않는다:
  - 회색 밀폐 부스 + 사무용 의자(등받이·팔걸이) + 착석 A3 로봇
    (raise_a3_ultra_t3d0/ 를 reference로 포함)
  - 조이스틱 2자유도 회전 조인트 아큘레이션 2개(짐벌 X+Y, 스프링 복원 드라이브)
  - 페달 판 2개(1-DOF 리볼루트 조인트, 뒷모서리 힌지, 스프링 복원 드라이브)

런타임:
  - 스틱: 드라이브 목표각을 쓰고 관절각을 "측정값"으로 읽는다.
    구동은 set_stick_articulations 로 주입된 Isaac Lab 핸들이 담당.
  - 페달: 답압량에 따라 관절 드라이브 목표각을 기록(안식각→수평).
    물리 발로 밟아도(접촉력>스프링) 수평까지 눌린다.
"""

import math
from pathlib import Path

import numpy as np
from pxr import UsdGeom, UsdPhysics

import config as C
from input_source import Command
from usd_utils import get_op, set_xform

DEFAULT_ASSET = Path(__file__).resolve().parent / "assets" / "cockpit.usd"
"""조종실 USD 애셋 경로(필수 - 없으면 오류로 종료)."""

CABIN_POS = (8.2, -7.7, 4.35)
"""데모에서 캐빈을 놓는 월드 위치(크레인 가운데 x=8.2, 남쪽 레일 바깥)."""

# ── 내부 치수(캐빈 중심 기준 국부 좌표) ──
CABIN_SIZE = (2.4, 2.0, 2.6)
FLOOR_Z = -CABIN_SIZE[2] / 2.0            # -1.3 (바닥 슬래브 아랫면 기준 캐빈 바닥)
FLOOR_T = 0.08                            # 바닥 슬래브 두께
FLOOR_TOP = FLOOR_Z + FLOOR_T             # 바닥 윗면(-1.22): 페달은 이면 위에 놓인다

# 페달: 발 하나 크기(가로 15cm × 세로 30cm × 두께 1cm).
# 안식 상태 = 뒤엣지(A, 로봇 쪽)는 바닥에 붙고 앞엣지(B, 먼 쪽)가 PEDAL_LIFT 만큼
# 부상(30cm 판에서 약 9.6°). 밟으면 B가 내려와 A와 같은 높이(판이 수평)가 된다.
PEDAL_SIZE = (0.15, 0.30, 0.01)
PEDAL_LIFT = 0.05
PEDAL_TILT_REST = math.asin(PEDAL_LIFT / PEDAL_SIZE[1])


# ═══════════════════════ 조종실 릭 ═══════════════════════
class CockpitRig:
    def __init__(self, stage, parent_path, usd_asset="auto", root_pos=CABIN_POS):
        """usd_asset: "auto"(기본 애셋 로드) | 경로(강제 로드)."""
        self.root = f"{parent_path}/Cabin"
        self._stick_arts = None            # (좌 아큘레이션, 우 아큘레이션) - set_stick_articulations
        self._stick_joint_ids = None       # 각 스틱의 [TiltXJoint, TiltYJoint] 인덱스
        if usd_asset == "auto":
            usd_asset = str(DEFAULT_ASSET)
        if not Path(usd_asset).is_file():
            raise FileNotFoundError(f"조종실 애셋이 없습니다: {usd_asset} - USD를 준비해 주세요")
        self._attach(stage, usd_asset, root_pos)

    def _attach(self, stage, usd_asset, root_pos):
        """애셋 reference 로드 + 갱신 핸들 다시 묶기."""
        root_prim = UsdGeom.Xform.Define(stage, self.root).GetPrim()
        root_prim.GetReferences().AddReference(usd_asset)
        set_xform(get_op(root_prim, create=True), root_pos)  # 위치는 참조하는 쪽에서 부여

        def p(name):  # 애셋 내부 프림 경로
            return f"{self.root}/{name}"


        # 조이스틱: 애셋이 아큘레이션이어야 한다(구동은 set_stick_articulations 로
        # 주입된 Isaac Lab 핸들이 담당).
        if not stage.GetPrimAtPath(p("StickL")).HasAPI(UsdPhysics.ArticulationRootAPI):
            raise RuntimeError(f"조종실 애셋에 조이스틱 아큘레이션이 없습니다: {usd_asset}")

        # 페달: 애셋의 1-DOF 리볼루트 조인트 프림(드라이브 목표각을 기록)
        self._pedal_joints = [stage.GetPrimAtPath(p(f"Pedal{n}/PedalPlate{n}Joint")) for n in ["L", "R"]]
        for jp in self._pedal_joints:
            if not jp.IsValid() or not jp.HasAPI(UsdPhysics.DriveAPI):
                raise RuntimeError(f"페달 조인트가 없거나 드라이브가 없습니다: {jp.GetPath()}")
        self.update(1.0 / 120.0, Command())  # 초기 자세 배치
        print(f"[cockpit] USD 애셋 로드: {usd_asset} | 조이스틱 아큘레이션")

    # ───────────────────────── 메인 갱신 ─────────────────────────
    def set_stick_articulations(self, stick_l, stick_r):
        """Isaac Lab 아큘레이션 핸들 주입(데모에서 InteractiveScene 생성 후 호출)."""
        import torch  # 아큘레이션 핸들 경로 - 앱 실행 이후에만 import 가능
        self._torch = torch
        self._stick_arts = (stick_l, stick_r)
        self._stick_joint_ids = []
        for art in self._stick_arts:
            ids, names = art.find_joints(["TiltXJoint", "TiltYJoint"])
            assert len(ids) == 2, f"스틱 조인트 발견 실패: {names}"
            self._stick_joint_ids.append(ids)

    def _drive_stick(self, art, ids, tilt_x, tilt_y):
        """스틱 1개의 짐벌 목표각 기록+커밋. tilt_x=전후(ly/rz), tilt_y=좌우(lx)."""
        target = self._torch.tensor([[tilt_x, tilt_y]], dtype=self._torch.float, device=art.device)
        art.set_joint_position_target(target, joint_ids=ids)
        art.write_data_to_sim()

    def update(self, dt, cmd: Command):
        # 1) 스틱: 명령 → 짐벌 목표각. 관절각 부호는 손잡이가 명령 방향으로 기울도록
        #    (+ly=북 → X축 -회전, +lx=동 → Y축 +회전).
        #    핸들이 주입되기 전(리셋 전)에는 물리가 중립을 유지한다.
        if self._stick_arts is not None:
            tilt = C.STICK_MAX_TILT
            self._drive_stick(self._stick_arts[0], self._stick_joint_ids[0],
                              -float(np.clip(cmd.ly, -1.0, 1.0)) * tilt,
                              float(np.clip(cmd.lx, -1.0, 1.0)) * tilt)
            self._drive_stick(self._stick_arts[1], self._stick_joint_ids[1],
                              -float(np.clip(cmd.rz, -1.0, 1.0)) * tilt, 0.0)

        # 2) 페달 2(1-DOF 리볼루트): 답압량만큼 드라이브 목표각을 안식각→수평(0°)으로.
        #    물리적으로 발이 닿아 눌러도 같은 한계 안에서 움직인다(스프링 복원).
        press = float(np.clip(cmd.pedal, 0.0, 1.0))
        target_deg = math.degrees(PEDAL_TILT_REST) * (1.0 - press)
        for jp in self._pedal_joints:
            jp.GetAttribute("drive:angular:physics:targetPosition").Set(target_deg)
