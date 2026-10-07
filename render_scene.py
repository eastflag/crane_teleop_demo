"""씬 미리보기 렌더러 - USD 애셋으로 구성한 장면을 헤드리스 렌더해 PNG로 저장.

demo.py 와 동일하게 애셋을 reference 로 올리고(물리 스텝 몇 회로 철근 안착 후)
카메라 프리셋별로 캡처한다. 씬 구성 검수용.

실행:
  ./isaaclab.sh -p crane_teleop_demo/render_scene.py --out /tmp/scene
  → /tmp/scene_external.png, /tmp/scene_yard.png, ... 생성
"""
import argparse
from pathlib import Path

from isaaclab.app import AppLauncher

import config as C  # 앱 실행 전 로드

parser = argparse.ArgumentParser(description="크레인 데모 씬 미리보기 렌더")
AppLauncher.add_app_launcher_args(parser)
parser.add_argument("--out", default="/tmp/scene", help="출력 PNG 접두어")
parser.add_argument("--views", default="external,yard,wide,truck",
                    help="캡처할 카메라 프리셋(쉼표 구분, config.CAMERAS 키 + truck)")
args_cli = parser.parse_args()

app_launcher = AppLauncher(headless=True, enable_cameras=True)
simulation_app = app_launcher.app

import omni.usd  # noqa: E402
from pxr import Gf, UsdGeom, UsdLux  # noqa: E402

from isaaclab.sim import SimulationContext, SimulationCfg  # noqa: E402
from isaaclab.sensors import Camera, CameraCfg  # noqa: E402
import isaaclab.sim as sim_utils  # noqa: E402

from cockpit import CockpitRig  # noqa: E402
import yard  # noqa: E402

# ── 추가 카메라 프리셋(트럭 클로즈업) ──
VIEWS = dict(C.CAMERAS)
VIEWS["truck"] = {"eye": (6.8, -4.6, 2.6), "target": (5.0, -2.5, 1.1)}

sim = SimulationContext(SimulationCfg(dt=C.SIM_DT, device="cpu"))
stage = omni.usd.get_context().get_stage()
mounts = yard.load_scene_assets(stage)
cockpit = CockpitRig(stage, parent_path="/World")

# 바닥 + 조명(demo.py의 ground/dome과 동일 역할)
ground = UsdGeom.Mesh.Define(stage, "/World/GroundVisual")
ground.CreatePointsAttr([Gf.Vec3f(-40, -40, 0), Gf.Vec3f(40, -40, 0), Gf.Vec3f(40, 40, 0), Gf.Vec3f(-40, 40, 0)])
ground.CreateFaceVertexCountsAttr([4])
ground.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
dome = UsdLux.DomeLight.Define(stage, "/World/DomePreview")
dome.CreateIntensityAttr(1200.0)
dome.CreateColorAttr(Gf.Vec3f(0.9, 0.92, 1.0))


def look_at_matrix(eye, target, up=(0.0, 0.0, 1.0)):
    """eye→target를 바라보는 카메라 월드 행렬(행벡터 관례, 전방=-Z)."""
    f = Gf.Vec3d(*[float(v) for v in target]) - Gf.Vec3d(*[float(v) for v in eye])
    f.Normalize()
    s = Gf.Cross(f, Gf.Vec3d(*up))
    s.Normalize()
    u = Gf.Cross(s, f)
    e = [float(v) for v in eye]
    return Gf.Matrix4d(
        s[0], s[1], s[2], 0.0,
        u[0], u[1], u[2], 0.0,
        -f[0], -f[1], -f[2], 0.0,
        e[0], e[1], e[2], 1.0)


sim.reset()
# 철근이 더미 위에 안착하도록 잠시 스텝(1cm 낙하 + 정지).
# 렌더 프로덕트를 만들기 전이라 이 구간은 렌더 비용이 없다.
for _ in range(60):
    sim.step()

# Isaac Lab Camera 센서로 캡처: 변형 설정 → 강제 렌더 → 버퍼 읽기 순서가 중요.
# (렌더 없이 update하면 스폰 자세의 스테일 프레임이 나온다)
cam_cfg = CameraCfg(
    prim_path="/World/PreviewCam",
    height=1080, width=1920,
    data_types=["rgb"],
    spawn=sim_utils.PinholeCameraCfg(focal_length=22.0),
)
camera = Camera(cam_cfg)

sim.reset()
# 철근이 더미 위에 안착하도록 잠시 스텝(1cm 낙하 + 정지)
for _ in range(60):
    sim.step()

from PIL import Image  # noqa: E402

for name in [v.strip() for v in args_cli.views.split(",") if v.strip()]:
    v = VIEWS[name]
    prim = stage.GetPrimAtPath("/World/PreviewCam")
    xfable = UsdGeom.Xformable(prim)
    xfable.ClearXformOpOrder()
    xfable.AddTransformOp().Set(look_at_matrix(v["eye"], v["target"]))
    sim.render()               # 새 카메라 자세로 렌더
    camera.update(0.0)         # 직후 버퍼 읽기
    img = camera.data.output["rgb"][0].cpu().numpy()
    out = f"{args_cli.out}_{name}.png"
    Image.fromarray(img[..., :3]).save(out)
    print(f"[render] {out} 저장")

import os as _os

print("", flush=True)
_os._exit(0)
