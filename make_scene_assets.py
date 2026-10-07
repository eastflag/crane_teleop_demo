"""씬 USD 애셋 생성기 - 웹 페이지(원격하차크레인시뮬레이터.html) 레이아웃 참고.

scene_build.py 로 4종 애셋을 만들어 assets/ 로 내보낸다(전부 Z-up, 미터 단위).

  assets/yard.usd        등급 적치구역 + AIMOS 게이트 + 제강소 + 소품
  assets/truck_load.usd  덤프트럭 + 시각용 분철 더미(정적 충돌체)
  assets/rebar_set.usd   개별 철근/분철 강체 30개(더미 위 1cm 낙하용)
  assets/crane.usd       갠트리 크레인(프리즘 3축 아큘레이션 + 드라이브)
  assets/textures/*.png  구역 명판·AIMOS 현판(한글, Noto Sans CJK)

실행:
  ./isaaclab.sh -p crane_teleop_demo/make_scene_assets.py [--num-rebar 30]

demo.py 는 애셋이 있으면 reference 로 로드하고, 없으면 절차적 생성으로 폴백한다.
"""
import argparse
import sys
from pathlib import Path

DEMO_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(DEMO_DIR))
import config  # noqa: F401  # 앱 실행 전 로드 - 키트의 cv2 config 그림자 임포트 방지

parser = argparse.ArgumentParser()
parser.add_argument("--num-rebar", type=int, default=config.REBAR_NUM,
                    help=f"개별 강체 수(최대 {config.REBAR_MAX})")
args_cli = parser.parse_args()

from isaaclab.app import AppLauncher  # noqa: E402

app_launcher = AppLauncher(headless=True)
simulation_app = app_launcher.app

from pxr import Usd, UsdGeom, UsdPhysics  # noqa: E402

import scene_build as SB  # noqa: E402
import config as C  # noqa: E402


# ═══════════════════════ 한글 명판 텍스처(PIL) ═══════════════════════
def _korean_font(size):
    """Noto Sans CJK 중 한글 글리프가 있는 면을 찾아 반환(실패 시 None)."""
    try:
        from PIL import ImageFont
    except ImportError:
        return None
    ttc = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"
    if not Path(ttc).is_file():
        return None
    for idx in range(8):
        try:
            font = ImageFont.truetype(ttc, size, index=idx)
            if font.getmask("중량").getbbox() is not None:
                return font
        except Exception:
            continue
    return None


def make_label_png(png, title, subtitle, hex_color, w=512, h=160):
    """구역 명판: 색띠 + 한글 명칭(+보조 영문)."""
    try:
        from PIL import Image, ImageDraw
    except ImportError:
        print(f"[asset] PIL 없음 - 명판 생략: {png}")
        return False
    font_t = _korean_font(64)
    font_s = _korean_font(30)
    img = Image.new("RGB", (w, h), (18, 22, 28))
    d = ImageDraw.Draw(img)
    d.rectangle([0, 0, 26, h], fill=hex_color)
    d.rectangle([w - 26, 0, w, h], fill=hex_color)
    if font_t:
        d.text((w / 2, h / 2 - 18), title, font=font_t, fill=(235, 240, 245), anchor="mm")
    else:
        d.text((w / 2, h / 2 - 18), title, fill=(235, 240, 245), anchor="mm")
    if subtitle and font_s:
        d.text((w / 2, h - 26), subtitle, font=font_s, fill=(150, 160, 172), anchor="mm")
    d.rectangle([0, 0, w - 1, h - 1], outline=hex_color, width=4)
    png.parent.mkdir(parents=True, exist_ok=True)
    img.save(png)
    print(f"[asset] 명판 저장: {png}")
    return True


def build_textures():
    for z in C.ZONES:
        make_label_png(SB.TEXTURE_DIR / f"zone_{z['key']}.png", z["name"],
                       f"GRADE ZONE {z['key']}", z["hex"])
    make_label_png(SB.TEXTURE_DIR / "aimos_board.png", "AIMOS 등급 판정",
                   "GRADE INSPECTION GATE", "#8F87EE", w=1024, h=192)


# ═══════════════════════ 애셋별 검증 ═══════════════════════
def check_stage(stage, default_prim, expects):
    prim = stage.GetPrimAtPath(default_prim)
    assert prim.IsValid(), f"기본 프림 없음: {default_prim}"
    stage.SetDefaultPrim(prim)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    for desc, path, apis in expects:
        p = stage.GetPrimAtPath(path)
        assert p.IsValid(), f"{desc}: 프림 없음 {path}"
        for api in apis:
            assert p.HasAPI(api), f"{desc}: {path}에 {api.__name__} 없음"
    n = sum(1 for _ in stage.Traverse())
    return n


def export(stage, name):
    out = SB.ASSET_DIR / name
    SB.ASSET_DIR.mkdir(exist_ok=True)
    stage.GetRootLayer().Export(str(out))
    print(f"[asset] 저장: {out} ({out.stat().st_size:,} bytes)")
    stage.GetRootLayer().Export(str(SB.ASSET_DIR / name.replace(".usd", ".usda")))
    return out


def main():
    SB.ASSET_DIR.mkdir(exist_ok=True)
    build_textures()

    # ── 1) yard.usd ──
    stage = Usd.Stage.CreateInMemory()
    SB.build_yard(stage, "/Yard")
    n = check_stage(stage, "/Yard", [("적치구역 LA", "/Yard/Zone_LA/Paint", ()),
                                     ("AIMOS 게이트", "/Yard/AimosGate/Beam", ()),
                                     ("제강소 벽", "/Yard/Mill/Wall", (UsdPhysics.CollisionAPI,)),
                                     ("배리어", "/Yard/Barrier0/Base", (UsdPhysics.CollisionAPI,))])
    export(stage, "yard.usd")
    print(f"[asset] yard 프림 수: {n}")

    # ── 2) truck_load.usd ──
    stage = Usd.Stage.CreateInMemory()
    SB.build_truck(stage, "/Truck")
    n = check_stage(stage, "/Truck", [("적재대 바닥", "/Truck/BedFloor", (UsdPhysics.CollisionAPI,)),
                                      ("분철 더미", "/Truck/Mound0_L1_0", (UsdPhysics.CollisionAPI,)),
                                      ("운전석", "/Truck/Cab", ())])
    export(stage, "truck_load.usd")
    print(f"[asset] truck 프림 수: {n}")

    # ── 3) rebar_set.usd(개별 강체 ≤30, 더미 위 1cm) ──
    stage = Usd.Stage.CreateInMemory()
    SB.build_rebar_set(stage, "/RebarSet", num=args_cli.num_rebar, seed=7)
    n_rb = sum(1 for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI))
    assert n_rb == min(args_cli.num_rebar, C.REBAR_MAX), f"강체 수 불일치: {n_rb}"
    check_stage(stage, "/RebarSet", [("첫 철근", "/RebarSet/rebar_0", (UsdPhysics.RigidBodyAPI,)),
                                     ("첫 분철", "/RebarSet/rebar_15", (UsdPhysics.RigidBodyAPI,))])
    # 더미 위 1cm 낙하 높이 검증
    from pxr import Gf
    z0 = UsdGeom.Xformable(stage.GetPrimAtPath("/RebarSet/rebar_0")).ComputeLocalToWorldTransform(0.0)
    top = C.TRUCK_BED_Z + 6 * (C.REBAR_RADIUS + 0.001) + 0.01
    assert abs(z0[3][2] - (top + C.REBAR_RADIUS)) < 0.05, f"철근 스폰 높이 오류: {z0[3][2]:.3f}"
    print(f"[asset] 강체 {n_rb}개 - 더미 마루 z={top:.3f} 위 ~1cm 스폰 OK")
    export(stage, "rebar_set.usd")

    # ── 4) crane.usd(프리즘 3축 아큘레이션) ──
    stage = Usd.Stage.CreateInMemory()
    SB.build_crane(stage, "/CraneRoot")
    n = check_stage(stage, "/CraneRoot", [
        ("아큘레이션 루트", "/CraneRoot/CraneArticulation", (UsdPhysics.ArticulationRootAPI,)),
        ("포탈 링크", "/CraneRoot/CraneArticulation/Portal", (UsdPhysics.RigidBodyAPI,)),
        ("트롤리 링크", "/CraneRoot/CraneArticulation/Trolley", (UsdPhysics.RigidBodyAPI,)),
        ("후크 링크", "/CraneRoot/CraneArticulation/Hook", (UsdPhysics.RigidBodyAPI,)),
        ("주행 조인트", "/CraneRoot/CraneArticulation/PortalTravelJoint", ()),
        ("횡행 조인트", "/CraneRoot/CraneArticulation/TrolleyTravelJoint", ()),
        ("권상 조인트", "/CraneRoot/CraneArticulation/HoistJoint", ()),
    ])
    for jn in ("PortalTravelJoint", "TrolleyTravelJoint", "HoistJoint"):
        j = stage.GetPrimAtPath(f"/CraneRoot/CraneArticulation/{jn}")
        assert j.HasAPI(UsdPhysics.DriveAPI), f"{jn}에 드라이브 없음"
        assert j.GetAttribute("physics:axis").Get() in ("X", "Y", "Z"), f"{jn} 축 없음"
        lo = j.GetAttribute("physics:lowerLimit").Get()
        hi = j.GetAttribute("physics:upperLimit").Get()
        assert lo < hi, f"{jn} 한계 오류"
        print(f"[asset] {jn}: axis={j.GetAttribute('physics:axis').Get()} limits=[{lo:.2f}, {hi:.2f}]")
    # 후크 자석 디스크가 MAGNET_DROP 만큼 아래에 있는지(월드 변환으로 검증)
    xform_cache = UsdGeom.XformCache()
    hook_m = xform_cache.GetLocalToWorldTransform(stage.GetPrimAtPath("/CraneRoot/CraneArticulation/Hook"))
    mag_m = xform_cache.GetLocalToWorldTransform(stage.GetPrimAtPath("/CraneRoot/CraneArticulation/Hook/Magnet"))
    dz = mag_m[3][2] - hook_m[3][2]
    assert abs(dz + C.MAGNET_DROP) < 1e-6, f"자석 오프셋 오류: {dz}"
    export(stage, "crane.usd")
    print(f"[asset] crane 프림 수: {n} | 후크 작성 높이 3.5, 자석 -{C.MAGNET_DROP} OK")

    print("[asset] 완료 - demo.py 실행 시 자동 로드됨")
    import os as _os

    print("", flush=True)
    _os._exit(0)  # simulation_app.close()의 세그폴트 회피(make_cockpit_asset.py와 동일)


if __name__ == "__main__":
    main()
