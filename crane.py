"""갠트리 크레인: 프리즘 3축 아큘레이션 구동(주행 Y + 횡행 X + 권상 Z).

구조(assets/crane.usd):
  - 정적: 고가 레일 + 기둥(야드 양쪽 x=1/17)
  - 아큘레이션: Portal(Y 조인트, 월드 고정) → Trolley(X 조인트) → Hook(Z 조인트)
    각 축은 위치 제어 드라이브를 가진다(게인은 config.CRANE_DRIVE).
  - 후크 링크에 자석 디스크가 붙어 있고, 와이어는 시각용 캡슐(런타임 갱신).

런타임 동작:
  - 스틱 속도 명령을 적분해 조인트 위치 목표로 쓴다(작업 범위 클램프).
  - 실제 관절 위치/속도에서 자석 위치를 계산해 magnet_logic 로 전달.
"""
import numpy as np

import config as C
from input_source import Command
from usd_utils import place_segment

HOOK_REST_Z = 3.5          # 후크 링크 작성 높이(crane.usd 와 일치)
JOINT_NAMES = {"x": "TrolleyTravelJoint", "y": "PortalTravelJoint", "z": "HoistJoint"}


class GantryCrane:
    def __init__(self, stage, articulation):
        """articulation: Isaac Lab Articulation 핸들(assets/crane.usd 래핑). 필수."""
        if articulation is None:
            raise ValueError("크레인 아큘레이션 핸들이 필요합니다(assets/crane.usd)")
        self.root = "/World/Crane"
        self._art = articulation

        from usd_utils import attach_capsule
        self._cable = attach_capsule(stage, f"{self.root}/Cable")
        # 조인트 인덱스: [주행(y), 횡행(x), 권상(z)] 순서로 고정
        ids, names = self._art.find_joints(
            [JOINT_NAMES["y"], JOINT_NAMES["x"], JOINT_NAMES["z"]])
        assert len(ids) == 3, f"크레인 조인트 발견 실패: {names}"
        self._joint_ids = ids
        print(f"[crane] 프리즘 3축 아큘레이션 구동: {names}")

        # ── 상태 ──
        self.x, self.y, self.z = C.CRANE_START          # 명령 적분값(목표)
        self.magnet_pos = np.zeros(3)
        self.magnet_vel = np.zeros(3)
        # 리셋 후 첫 update에서 관절 상태를 읽는다

    # ───────────────────────── 메인 갱신 ─────────────────────────
    def update(self, dt, cmd: Command):
        # 1) 스틱 속도 명령 적분 + 작업 범위 클램프 → 조인트 목표
        self.x = float(np.clip(self.x + cmd.lx * C.CRANE_SPEED_XY * dt, *C.CRANE_X_LIMITS))
        self.y = float(np.clip(self.y + cmd.ly * C.CRANE_SPEED_XY * dt, *C.CRANE_Y_LIMITS))
        self.z = float(np.clip(self.z + cmd.rz * C.CRANE_SPEED_Z * dt, *C.HOOK_Z_LIMITS))

        # 2) 위치 목표 기록+커밋(주행 y, 횡행 x, 권상 z=후크 높이-작성높이)
        import torch

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

    def state(self):
        """스크립트 파일럿 등에 전달할 현재 상태."""
        jp = self._art.data.joint_pos[0, self._joint_ids].detach().cpu().numpy()
        return {"x": float(jp[1]), "y": float(jp[0]), "z": float(jp[2]) + HOOK_REST_Z}
