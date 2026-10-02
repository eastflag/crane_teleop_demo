"""갠트리 크레인 원격조종 데모 - 메인 진입점.

시나리오: 조종실의 로봇이 스틱 2개를 조종하면 야드의 갠트리 크레인이 움직이고,
자석으로 철근을 흡착해 운반하며, 패들을 밟으면 철근을 놓는다.

실행(Isaac Lab 설치 디렉터리 기준):
  ./isaaclab.sh -p crane_teleop_demo/demo.py --input scripted     # 하드웨어 없는 자동 검증
  ./isaaclab.sh -p crane_teleop_demo/demo.py --input gamepad      # 게임패드 조종

신호 흐름:
  [게임패드] → input_source(필터) → cockpit(스틱 각도 + 로봇 IK)
                              ↘ crane(포탈/트롤리/권상 + 진자) → magnet_logic(부착/해제) → 철근(물리)
"""
import argparse

# ── AppLauncher는 isaaclab 다른 모듈보다 먼저 ──
from isaaclab.app import AppLauncher

import config as C  # 순수 파이썬이라 앱 실행 전 import 가능

parser = argparse.ArgumentParser(description="갠트리 크레인 원격조종 데모")
AppLauncher.add_app_launcher_args(parser)
parser.add_argument("--input", choices=["auto", "gamepad", "scripted"], default="auto",
                    help="입력 소스 (auto: 게임패드 시도 후 실패 시 scripted)")
parser.add_argument("--num-rebar", type=int, default=C.REBAR_NUM, help="철근 개수")
parser.add_argument("--seed", type=int, default=7, help="철근 배치 시드")
parser.add_argument("--camera", default="external", choices=list(C.CAMERAS), help="초기 카메라")
parser.add_argument("--max-steps", type=int, default=0, help="0이면 무한(스모크 테스트용)")
args_cli = parser.parse_args()

try:
    app_launcher = AppLauncher(args_cli)
except TypeError:  # 구형 시그니처 폴백
    app_launcher = AppLauncher(headless=args_cli.headless, device=args_cli.device)
simulation_app = app_launcher.app

# ═══════════════════ 앱 실행 이후에만 isaaclab/pxr import ═══════════════════
import numpy as np  # noqa: E402
import omni.usd  # noqa: E402
import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.assets import AssetBaseCfg  # noqa: E402
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg  # noqa: E402
from isaaclab.sim import SimulationContext, SimulationCfg  # noqa: E402
from isaaclab.utils import configclass  # noqa: E402

from cockpit import CockpitRig  # noqa: E402
from crane import GantryCrane  # noqa: E402
from input_source import make_source  # noqa: E402
from magnet_logic import MagnetController  # noqa: E402
import yard  # noqa: E402

device = args_cli.device if args_cli.device else C.DEVICE_DEFAULT

# ── 시뮬레이션 컨텍스트 ──
sim = SimulationContext(SimulationCfg(dt=C.SIM_DT, device=device))


# ── 씬 설정: 바닥/조명/철근(동적 물리) ──
@configclass
class YardSceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(prim_path="/World/ground", spawn=sim_utils.GroundPlaneCfg())
    dome = AssetBaseCfg(prim_path="/World/Dome", spawn=sim_utils.DomeLightCfg(intensity=2000.0, color=(0.9, 0.92, 1.0)))
    rebar = yard.make_rebar_collection_cfg(args_cli.num_rebar, args_cli.seed)


scene = InteractiveScene(YardSceneCfg(num_envs=1, env_spacing=2.0))

stage = omni.usd.get_context().get_stage()

# ── 씬 구성 요소(전부 비물리 시각+운동학 요소) ──
crane = GantryCrane(stage)
cockpit = CockpitRig(stage, parent_path="/World")   # 조종실은 월드 고정(크레인 이동과 독립)
yard.add_drop_zone(stage)
magnet = MagnetController(scene["rebar"], device)
source, source_name = make_source(args_cli.input)

cam = C.CAMERAS[args_cli.camera]
sim.set_camera_view(eye=list(cam["eye"]), target=list(cam["target"]))

sim.reset()
scene.reset()
print(f"[demo] 시작 | 입력={source_name} | 철근={args_cli.num_rebar}개 | 카메라={args_cli.camera}")
print("[demo] 조작: 왼스틱=XY 이동, 오른쪽 스틱 상하=권상, RT(또는 A)=패들(자석 해제)")

# ── 메인 루프 ──
step = 0
while simulation_app.is_running():
    dt = C.SIM_DT  # SimulationCfg(dt=...)로 지정한 물리 스텝

    cmd = source.poll(dt, crane.state())       # 1) 입력
    cockpit.update(dt, cmd)                    # 2) 스틱 각도 + 로봇 팔/다리 IK
    crane.update(dt, cmd)                      # 3) 크레인 운동학 + 진자
    magnet.update(dt, crane.magnet_pos, crane.magnet_vel, cmd.pedal)  # 4) 부착/운반/해제

    sim.step()
    scene.update(dt)
    step += 1

    if step % 240 == 0:
        phase = getattr(source, "phase", "")
        extra = f" 단계={phase}" if phase else ""
        print(f"[demo] t={step * dt:5.1f}s pos=({crane.x:5.2f},{crane.y:5.2f}) "
              f"hook_z={crane.z:4.2f} 흔들림=({crane.th[0]:+.2f},{crane.th[1]:+.2f}) "
              f"부착={len(magnet.welded)}{extra}")

    if args_cli.max_steps and step >= args_cli.max_steps:
        print(f"[demo] max-steps({args_cli.max_steps}) 도달 - 종료")
        break

simulation_app.close()
