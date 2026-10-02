"""조종실 USD 애셋 생성기.

cockpit.py의 절차적 생성(_build)을 그대로 실행해 assets/ 로 내보낸다.
착석 A3 로봇은 raise_a3_ultra_t3d0/payloads/base.usda 를 reference로 유지하므로
(36MB 메시를 임베딩하지 않기 위해) 루트 레이어만 내보낸다. 즉 이 애셋은
같은 저장소의 raise_a3_ultra_t3d0/ 폴더가 있어야 완전하게 로드된다.

demo.py는 애셋이 있으면 자동으로 reference 로드하고(스틱·페달 애니메이션은
로드 후에도 동일하게 동작), 없으면 절차적 생성으로 폴백한다.

실행:
  ./isaaclab.sh -p crane_teleop_demo/make_cockpit_asset.py

결과:
  assets/cockpit.usd   루트 레이어(데모가 로드하는 파일)
  assets/cockpit.usda  텍스트 사본(git diff / 검수용)
"""
import sys
from pathlib import Path

DEMO_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(DEMO_DIR))
import config  # noqa: F401  # 앱 실행 전 로드 - 키트의 cv2 config 그림자 임포트 방지(demo.py 참고)

from isaaclab.app import AppLauncher  # noqa: E402

app_launcher = AppLauncher(headless=True)
simulation_app = app_launcher.app

from pxr import Usd  # noqa: E402

from cockpit import CockpitRig  # noqa: E402

# 독립 레이어에 루트 프림 /Cabin 으로 빌드(원점 기준 - 위치는 데모가 참조할 때 부여)
stage = Usd.Stage.CreateInMemory()
rig = CockpitRig(stage, parent_path="", usd_asset=None, root_pos=(0.0, 0.0, 0.0))
stage.SetDefaultPrim(stage.GetPrimAtPath(rig.root))

# ── 착석 포즈 검증(로봇 국부 좌표: +X=정면, 골반=원점) ──
from cockpit import _prim_local_mat  # noqa: E402


def _local(path):
    prim = stage.GetPrimAtPath(path)
    assert prim.IsValid(), f"로봇 프림 없음: {path}"
    m = _prim_local_mat(prim)  # 열벡터 관례 4x4 → 이동은 마지막 열
    return m[0, 3], m[1, 3], m[2, 3]


hip = _local("/Cabin/Robot/left_hip_pitch_Link")
knee = _local("/Cabin/Robot/left_knee_Link")
ankle = _local("/Cabin/Robot/left_ankle_roll_Link")
print(f"[asset] 착석 검증 | hip=({hip[0]:+.3f},{hip[2]:+.3f}) knee=({knee[0]:+.3f},{knee[2]:+.3f}) "
      f"ankle=({ankle[0]:+.3f},{ankle[2]:+.3f})")
assert knee[0] - hip[0] > 0.30, "허벅지가 앞으로 뻗지 않음(hip_pitch 부호 확인)"
assert abs(knee[2] - hip[2]) < 0.10, "허벅지가 수평이 아님"
assert ankle[2] - knee[2] < -0.30, "정강이가 아래로 향하지 않음(knee 부호 확인)"
# 정강이는 수직에서 앞쪽으로 30° 기울어야 함: 전방 오프셋 ≈ 0.415·sin30° ≈ 0.21
assert 0.14 < ankle[0] - knee[0] < 0.28, f"정강이 전방 30° 경사 오류: {ankle[0]-knee[0]:.3f}"
shin = ((ankle[0] - knee[0]) ** 2 + (ankle[2] - knee[2]) ** 2) ** 0.5
assert 0.36 < shin < 0.47, f"정강이 길이 오류({shin:.3f}) - FK 구성 순서 확인"
print("[asset] 착석 포즈 OK: 허벅지 수평 전방, 정강이 전방 30° 경사")

# 고정 관절 자식(커버·손)이 부모에서 떨어지지 않았는지 - 분리 버그 재발 감지
shell = _local("/Cabin/Robot/left_knee_shell_Link")
wrist = _local("/Cabin/Robot/left_wrist_yaw_Link")
palm = _local("/Cabin/Robot/L_palm")
d_shell = ((shell[0] - knee[0]) ** 2 + (shell[2] - knee[2]) ** 2) ** 0.5
d_palm = ((palm[0] - wrist[0]) ** 2 + (palm[2] - wrist[2]) ** 2) ** 0.5
assert d_shell < 0.15, f"정강이 커버 분리({d_shell:.3f}m) - 고정 관절 트리 확인"
assert d_palm < 0.15, f"손바닥 분리({d_palm:.3f}m) - 고정 관절 트리 확인"
print(f"[asset] 연속성 OK: 무릎-정강이커버 {d_shell:.3f}m, 손목-손바닥 {d_palm:.3f}m")

out_dir = DEMO_DIR / "assets"
out_dir.mkdir(exist_ok=True)
for name in ("cockpit.usd", "cockpit.usda"):
    out = out_dir / name
    stage.GetRootLayer().Export(str(out))  # 루트 레이어만(로봇은 reference 유지)
    print(f"[asset] 저장: {out} ({out.stat().st_size:,} bytes)")

n_prims = sum(1 for _ in stage.Traverse())
print(f"[asset] 프림 수: {n_prims} | 기본 프림: {rig.root}")
print("[asset] 완료 - demo.py 실행 시 자동 로드됨")

# simulation_app.close()는 키트 종료 중 세그폴트를 일으키는 경우가 있어 강제 종료
import os as _os  # noqa: E402

print("", flush=True)
_os._exit(0)
