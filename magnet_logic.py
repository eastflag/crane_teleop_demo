"""전자석 로직: 근접 부착(키네마틱 용접) + 자석 OFF 시 해제.

동작 규약(웹 페이지와 동일한 자석 ON/OFF 모델):
  - 자석이 ON이면 하부 필드 안의 철근/분철이 자동 부착된다.
    실제 전자석처럼 살짝 떨어진 철근도 빨아들인다(근접 판정).
  - 부착된 철근은 매 스텝 자석 기준 상대 자세로 강제 이동(키네마틱 용접)되며,
    속도도 자석 속도로 기록되므로 해제 순간 관성이 자연스럽게 이어진다.
  - 자석을 OFF하면 모든 철근이 떨어진다(지정 구역 판정은 데모가 수행).

v2 업그레이드 경로: 런타임 fixed joint 생성 또는 Isaac Lab의 attachable rigid
object API로 교체하면 하중 질량이 크레인에 실리는 물리가 필요할 때 대응 가능.
"""
import numpy as np
import torch

import config as C


class MagnetController:
    def __init__(self, rebar_asset, device):
        self.rebar = rebar_asset
        self.device = device
        self.welded = {}  # 철근 인덱스 → (자석 기준 오프셋 np[3], 철근 자세 np[4])

    # ───────────────────────── 메인 갱신 ─────────────────────────
    def update(self, dt, magnet_pos, magnet_vel, mag_on):
        if not mag_on:
            self._release(magnet_vel)
        else:
            self._try_attach(np.asarray(magnet_pos, dtype=float))
        self._carry(np.asarray(magnet_pos, dtype=float), np.asarray(magnet_vel, dtype=float))

    # ───────────────────────── 부착 판정 ─────────────────────────
    def _try_attach(self, magnet):
        if len(self.welded) >= C.MAGNET_MAX_ATTACH:
            return
        # 컬렉션 데이터: (num_envs, num_objects, ·) → 환경 1개 기준 평탄화
        pos = self.rebar.data.object_pos_w.detach().reshape(-1, 3).cpu().numpy()
        quat = self.rebar.data.object_quat_w.detach().reshape(-1, 4).cpu().numpy()

        candidates = []
        for i in range(len(pos)):
            if i in self.welded:
                continue
            d = pos[i] - magnet
            dxy = float(np.hypot(d[0], d[1]))
            if dxy < C.MAGNET_FIELD_XY and C.MAGNET_FIELD_DZ[0] < d[2] < C.MAGNET_FIELD_DZ[1]:
                candidates.append((dxy, i))
        for _, i in sorted(candidates)[: C.MAGNET_MAX_ATTACH - len(self.welded)]:
            off = pos[i] - magnet
            # 흡착 스냅: 캡처 순간의 공중 거리 그대로 두면 철근이 자석에서 몇 cm
            # 떨어진 채 운반된다. 수평 오프셋만 유지하고 높이는 자석 하부에 밀착.
            xy = np.clip(off[:2], -0.18, 0.18)
            self.welded[i] = (np.array([xy[0], xy[1], -C.MAGNET_SNAP_DZ]), quat[i].copy())

    # ───────────────────────── 운반(용접 유지) ─────────────────────────
    def _carry(self, magnet, magnet_vel):
        if not self.welded:
            return
        items = list(self.welded.items())
        # 컬렉션 쓰기 API는 환경과 객체 인덱스를 분리해서 받는다(환경 1개 고정).
        object_ids = torch.tensor([i for i, _ in items], dtype=torch.long, device=self.device)
        poses = torch.tensor(
            [[*(magnet + off), *quat] for _, (off, quat) in items],
            dtype=torch.float, device=self.device,
        )
        vels = torch.tensor(
            [[*magnet_vel, 0.0, 0.0, 0.0]] * len(items),
            dtype=torch.float, device=self.device,
        )
        self.rebar.write_object_pose_to_sim(poses.unsqueeze(0), object_ids=object_ids)
        self.rebar.write_object_velocity_to_sim(vels.unsqueeze(0), object_ids=object_ids)

    # ───────────────────────── 해제(패들) ─────────────────────────
    def _release(self, magnet_vel):
        if not self.welded:
            return
        items = list(self.welded.items())
        object_ids = torch.tensor([i for i, _ in items], dtype=torch.long, device=self.device)
        vels = torch.tensor(
            [[magnet_vel[0], magnet_vel[1], magnet_vel[2] - 0.3, 0.0, 0.0, 0.0]] * len(items),
            dtype=torch.float, device=self.device,
        )
        self.rebar.write_object_velocity_to_sim(vels.unsqueeze(0), object_ids=object_ids)
        self.welded.clear()
