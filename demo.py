"""갠트리 크레인 원격조종 데모 - 메인 진입점.

시나리오(웹 페이지 원격하차크레인시뮬레이터.html 참고):
  조종실에 앉은 로봇이 왼 스틱으로 크레인을 상하좌우(횡행 X·주행 Y)로 움직이고
  오른 스틱 상하로 권상(Z)한다. 앞 발판(페달)은 브레이크, 버튼은 전자석 ON/OFF.
  트럭에 실려 온 분철을 자석으로 흡착해 등급별 지정 구역(중량A/B·경량A/B)에
  하차하고 적치 판정을 출력한다.

씬은 USD 애셋으로 구성하고 데모는 reference로 로드만 한다:
  assets/yard.usd · truck_load.usd · rebar_set.usd · crane.usd(프리즘 3축
  아큘레이션) · cockpit.usd(조이스틱 2자유도 회전 아큘레이션 포함)

실행(Isaac Lab 설치 디렉터리 기준):
  ./isaaclab.sh -p crane_teleop_demo/demo.py --input scripted     # 하드웨어 없는 자동 검증
  ./isaaclab.sh -p crane_teleop_demo/demo.py --input gamepad      # 게임패드 조종

신호 흐름:
  [게임패드] → input_source(필터) → cockpit(스틱 관절 목표 + 로봇 착석)
                              ↘ crane(프리즘 3축 아큘레이션) → magnet_logic(부착/해제) → 철근(물리)
"""
import argparse

# ── AppLauncher는 isaaclab 다른 모듈보다 먼저 ──
from isaaclab.app import AppLauncher

import config as C  # 순수 파이썬이라 앱 실행 전 import 가능

parser = argparse.ArgumentParser(description="갠트리 크레인 원격조종 데모")
AppLauncher.add_app_launcher_args(parser)
parser.add_argument("--input", choices=["auto", "gamepad", "scripted"], default="auto",
                    help="입력 소스 (auto: 게임패드 시도 후 실패 시 scripted)")
parser.add_argument("--num-rebar", type=int, default=C.REBAR_NUM,
                    help=f"개별 철근/분철 수(최대 {C.REBAR_MAX})")
parser.add_argument("--camera", default="external", choices=list(C.CAMERAS), help="초기 카메라")
parser.add_argument("--max-steps", type=int, default=0, help="0이면 무한(스모크 테스트용)")
args_cli = parser.parse_args()

try:
    app_launcher = AppLauncher(args_cli)
except TypeError:  # 구형 시그니처 폴백
    app_launcher = AppLauncher(headless=args_cli.headless, device=args_cli.device)
simulation_app = app_launcher.app

# ═══════════════════ 앱 실행 이후에만 isaaclab/pxr import ═══════════════════
import omni.usd  # noqa: E402
import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.actuators import ImplicitActuatorCfg  # noqa: E402
from isaaclab.assets import ArticulationCfg, AssetBaseCfg  # noqa: E402
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg  # noqa: E402
from isaaclab.sim import SimulationContext, SimulationCfg  # noqa: E402
from isaaclab.utils import configclass  # noqa: E402

from cockpit import CockpitRig  # noqa: E402
from crane import GantryCrane, HOOK_REST_Z, JOINT_NAMES  # noqa: E402
from input_source import Command, make_source  # noqa: E402
from magnet_logic import MagnetController  # noqa: E402
import yard  # noqa: E402

device = args_cli.device if args_cli.device else C.DEVICE_DEFAULT

# ── 시뮬레이션 컨텍스트 ──
sim = SimulationContext(SimulationCfg(dt=C.SIM_DT, device=device))
stage = omni.usd.get_context().get_stage()

# ── 씬 애셋 reference(InteractiveScene 구성 전에 스테이지에 올린다) ──
yard.load_scene_assets(stage)                   # yard / truck / rebar_set / crane
cockpit = CockpitRig(stage, parent_path="/World")   # 조종실(로봇·스틱 포함 애셋)

# ── 시뮬레이션 객체 설정: 애셋 프림을 래핑(spawn=None) ──
entries = {
    "ground": AssetBaseCfg(prim_path="/World/ground", spawn=sim_utils.GroundPlaneCfg()),
    "dome": AssetBaseCfg(prim_path="/World/Dome",
                         spawn=sim_utils.DomeLightCfg(intensity=2000.0, color=(0.9, 0.92, 1.0))),
    "rebar": yard.make_rebar_collection_cfg(args_cli.num_rebar),
}
# 크레인: 프리즘 3축 아큘레이션(crane.usd). 초기 관절위치 = CRANE_START.
# 액추에이터 게인은 crane.usd 의 드라이브(config.CRANE_DRIVE)와 동일하게.
tr, tt, th = C.CRANE_DRIVE["travel"], C.CRANE_DRIVE["trolley"], C.CRANE_DRIVE["hoist"]
entries["crane_art"] = ArticulationCfg(
    prim_path="/World/Crane/CraneArticulation",
    spawn=None,
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            JOINT_NAMES["y"]: C.CRANE_START[1],
            JOINT_NAMES["x"]: C.CRANE_START[0],
            JOINT_NAMES["z"]: C.CRANE_START[2] - HOOK_REST_Z,
        },
    ),
    actuators={
        "travel": ImplicitActuatorCfg(
            joint_names_expr=[JOINT_NAMES["y"]], stiffness=tr[0], damping=tr[1],
            effort_limit=tr[2], velocity_limit=10.0),
        "trolley": ImplicitActuatorCfg(
            joint_names_expr=[JOINT_NAMES["x"]], stiffness=tt[0], damping=tt[1],
            effort_limit=tt[2], velocity_limit=10.0),
        "hoist": ImplicitActuatorCfg(
            joint_names_expr=[JOINT_NAMES["z"]], stiffness=th[0], damping=th[1],
            effort_limit=th[2], velocity_limit=10.0),
    },
)
# 조이스틱 2자유도 회전 아큘레이션(cockpit.usd 안)
sk, sd, sf = C.STICK_DRIVE
stick_cfg = lambda path: ArticulationCfg(
    prim_path=path, spawn=None,
    actuators={"gimbal": ImplicitActuatorCfg(
        joint_names_expr=[".*"], stiffness=sk, damping=sd, effort_limit=sf)},
)
entries["stick_l"] = stick_cfg("/World/Cabin/StickL")
entries["stick_r"] = stick_cfg("/World/Cabin/StickR")

YardSceneCfg = configclass(type("YardSceneCfg", (InteractiveSceneCfg,), dict(entries)))
scene = InteractiveScene(YardSceneCfg(num_envs=1, env_spacing=2.0))

magnet = MagnetController(scene["rebar"], device)
source, source_name = make_source(args_cli.input)

cam = C.CAMERAS[args_cli.camera]
sim.set_camera_view(eye=list(cam["eye"]), target=list(cam["target"]))

# 아큘레이션 핸들은 리셋 이후에 접근 가능(physx 뷰가 늦게 생김)
sim.reset()
scene.reset()
crane = GantryCrane(stage, articulation=scene["crane_art"])
cockpit.set_stick_articulations(scene["stick_l"], scene["stick_r"])
crane.update(1.0 / 120.0, Command())   # 초기 목표 전송 + 자석 위치 계산

mag_on = False   # 자석은 끄고 시작(웹 페이지처럼 버튼으로 켠다) - settle에서 ON
judge_ok = 0
judge_total = 0
prev_welded = 0
print(f"[demo] 시작 | 입력={source_name} | 철근={magnet.rebar.num_objects}개 | 카메라={args_cli.camera}")
print("[demo] 조작: 왼스틱=XY 이동, 오른스틱 상하=권상, RT/A(누르는 동안)=브레이크, B/X=자석 ON·OFF")
print(f"[demo] 적치 구역: {' · '.join(z['name'] for z in C.ZONES)}")

# ── 메인 루프 ──
step = 0
while simulation_app.is_running():
    dt = C.SIM_DT  # SimulationCfg(dt=...)로 지정한 물리 스텝

    cmd = source.poll(dt, crane.state())       # 1) 입력
    if cmd.pedal > 0.5:                        # 2) 브레이크 페달: 명령 차단(크레인 정지)
        cmd = Command(pedal=1.0)
    if cmd.mag > 0.5:                          # 3) 자석 토글
        mag_on = not mag_on
        print(f"[demo] 자석 {'ON' if mag_on else 'OFF'}")
    cockpit.update(dt, cmd)                    # 4) 스틱 관절 목표
    crane.update(dt, cmd)                      # 5) 크레인 조인트 목표 + 상태 읽기
    magnet.update(dt, crane.magnet_pos, crane.magnet_vel, mag_on)  # 6) 부착/운반/해제

    # 7) 하차 판정: 자석 OFF 직후(부착물이 있던 상태에서 해제) 자석 아래가
    #    지정 구역인지 판정(웹 페이지의 적치 판정 메시지 재현)
    if not mag_on and prev_welded > 0:
        target = getattr(source, "target_zone", -1)
        ok, msg = yard.judge_drop(crane.magnet_pos[0], crane.magnet_pos[1], target)
        judge_total += 1
        judge_ok += 1 if ok else 0
        print(f"[판정] {msg} | 정위치 {judge_ok}/{judge_total}")
    prev_welded = len(magnet.welded)

    sim.step()
    scene.update(dt)
    step += 1

    if step % 240 == 0:
        phase = getattr(source, "phase", "")
        extra = f" 단계={phase}" if phase else ""
        st = crane.state()
        print(f"[demo] t={step * dt:5.1f}s pos=({st['x']:5.2f},{st['y']:5.2f}) "
              f"hook_z={st['z']:4.2f} 자석={'ON' if mag_on else 'OFF'} "
              f"부착={len(magnet.welded)} 정위치={judge_ok}/{judge_total}{extra}")

    if args_cli.max_steps and step >= args_cli.max_steps:
        print(f"[demo] max-steps({args_cli.max_steps}) 도달 - 종료")
        break

if args_cli.max_steps:
    # 스모크 테스트 모드: simulation_app.close()가 매달리는 경우가 있어(키트 종료
    # 세그폴트/교착) 출력을 플러시하고 강제 종료.
    import os as _os
    import sys as _sys

    print("[demo] 스모크 모드 - 즉시 종료", flush=True)
    _sys.stdout.flush()
    _sys.stderr.flush()
    _os._exit(0)

simulation_app.close()
