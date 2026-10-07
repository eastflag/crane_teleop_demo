"""씬 USD 애셋 빌더 - 웹페이지(원격하차크레인시뮬레이터.html) 레이아웃 참고.

make_scene_assets.py 가 호출해 assets/ 아래 4종 애셋을 만든다(전부 Z-up, 미터).

  yard.usd       야드: 등급별 적치구역(중량A/B·경량A/B) + AIMOS 판정 게이트 +
                 제강소 배경 건물 + 차선 표시 + 안전 소품(배리어·드럼·타이어·CCTV)
  truck_load.usd 덤프트럭 + 적재대 위 시각용 분철 더미(정적 충돌체)
  rebar_set.usd  자석으로 집을 수 있는 개별 철근/분철 30개 강체(더미 위 1cm)
  crane.usd      갠트리 크레인: 프리즘 3축(주행 Y·횡행 X·권상 Z) 아큘레이션 + 드라이브

좌표계는 기존 데모(config.py)와 동일: +X 동, +Y 북, +Z 위. 레일 x=1/17,
브리지 높이 8m, 트롤리 z 7.45, 앵커 z 7.2.
"""
import math
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade

import config as C
from usd_utils import box, capsule, cylinder, make_material, quat_axis_angle, quat_between, sphere

ASSET_DIR = Path(__file__).resolve().parent / "assets"
TEXTURE_DIR = ASSET_DIR / "textures"


# ═══════════════════════════ 공통 헬퍼 ═══════════════════════════
def emat(stage, path, color, emissive, intensity=1.0, roughness=0.5, metallic=0.0, opacity=None):
    """발광 재질(구역 경계선·경광등·표지판용)."""
    mat = make_material(stage, path, color, roughness=roughness, metallic=metallic,
                        opacity=opacity if opacity is not None else 1.0)
    shader = UsdShade.Shader(stage.GetPrimAtPath(f"{path}/Shader"))
    shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(*[float(e) for e in emissive]))
    shader.CreateInput("emissiveIntensity", Sdf.ValueTypeNames.Float).Set(float(intensity))
    return mat


def group(stage, path, pos=(0.0, 0.0, 0.0)):
    from usd_utils import set_xform, get_op
    xf = UsdGeom.Xform.Define(stage, path)
    op = xf.MakeMatrixXform() if hasattr(xf, "MakeMatrixXform") else xf.AddTransformOp()
    m = Gf.Matrix4d()
    m.SetTranslateOnly(Gf.Vec3d(*[float(v) for v in pos]))
    op.Set(m)
    return op


def static_collision(stage, path):
    """정적 충돌체(리지드바디 없음)로 지정."""
    UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(path))


def textured_plane(stage, path, png, width, height, pos, quat=(1.0, 0.0, 0.0, 0.0),
                   emissive=0.0, opacity=None):
    """UV가 달린 평면 메시 + PNG 텍스처 재질(구역 명판·게이트 현판)."""
    w, h = float(width) / 2.0, float(height) / 2.0
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr([
        Gf.Vec3f(-w, 0.0, h), Gf.Vec3f(w, 0.0, h), Gf.Vec3f(w, 0.0, -h), Gf.Vec3f(-w, 0.0, -h),
    ])
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateDoubleSidedAttr(True)
    st = UsdGeom.PrimvarsAPI(mesh.GetPrim()).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
    st.Set([Gf.Vec2f(0.0, 0.0), Gf.Vec2f(1.0, 0.0), Gf.Vec2f(1.0, 1.0), Gf.Vec2f(0.0, 1.0)])

    xf = UsdGeom.Xformable(mesh)
    op = xf.MakeMatrixXform() if hasattr(xf, "MakeMatrixXform") else xf.AddTransformOp()
    from usd_utils import set_xform
    set_xform(op, pos, quat)

    looks = "/".join(path.split("/")[:-2]) + "/Looks"
    mat = UsdShade.Material.Define(stage, f"{looks}/M_{path.split('/')[-1]}")
    surface = UsdShade.Shader.Define(stage, f"{looks}/M_{path.split('/')[-1]}/Surface")
    surface.CreateIdAttr("UsdPreviewSurface")
    reader = UsdShade.Shader.Define(stage, f"{looks}/M_{path.split('/')[-1]}/UVReader")
    reader.CreateIdAttr("UsdPrimvarReader_float2")
    reader.CreateInput("varname", Sdf.ValueTypeNames.String).Set("st")
    tex = UsdShade.Shader.Define(stage, f"{looks}/M_{path.split('/')[-1]}/Texture")
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(str(png)))
    tex.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("clamp")
    tex.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("clamp")
    tex.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
    tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(), "result")
    surface.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
        tex.ConnectableAPI(), "rgb")
    if emissive > 0.0:
        surface.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(1.0, 1.0, 1.0))
        surface.CreateInput("emissiveIntensity", Sdf.ValueTypeNames.Float).Set(float(emissive))
    if opacity is not None:
        surface.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(float(opacity))
    mat.CreateSurfaceOutput().ConnectToSource(surface.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI(mesh).Bind(mat)
    return mesh


# ═══════════════════════════ yard.usd ═══════════════════════════
def build_yard(stage, root="/Yard"):
    """야드 전체(적치구역·AIMOS 게이트·제강소·차선·소품). 전부 정적."""
    looks = f"{root}/Looks"
    mats = {
        "paint_y": make_material(stage, f"{looks}/PaintYellow", (0.85, 0.68, 0.05), roughness=0.8),
        "paint_w": make_material(stage, f"{looks}/PaintWhite", (0.88, 0.88, 0.86), roughness=0.8),
        "concrete": make_material(stage, f"{looks}/Concrete", (0.42, 0.43, 0.45), roughness=0.9),
        "steel": make_material(stage, f"{looks}/SteelBlue", (0.30, 0.38, 0.48), roughness=0.5, metallic=0.6),
        "dark": make_material(stage, f"{looks}/Dark", (0.12, 0.12, 0.14), roughness=0.7),
        "rust": make_material(stage, f"{looks}/Rust", (0.45, 0.24, 0.12), roughness=0.85, metallic=0.3),
        "glow_w": emat(stage, f"{looks}/GlowWhite", (0.9, 0.9, 0.95), (0.9, 0.9, 0.95), 2.0),
        "lamp_g": emat(stage, f"{looks}/LampGreen", (0.1, 0.7, 0.3), (0.1, 0.8, 0.35), 3.0),
    }
    zone_mats = {}
    for i, z in enumerate(C.ZONES):
        r, g, b = z["color"]
        zone_mats[z["name"]] = {
            "paint": emat(stage, f"{looks}/Zone{i}_Paint", (r * 0.55 + 0.35, g * 0.55 + 0.35, b * 0.55 + 0.35),
                          (r, g, b), 0.06, roughness=0.85, opacity=0.9),
            "edge": emat(stage, f"{looks}/Zone{i}_Edge", (r, g, b), (r, g, b), 0.9),
            "post": emat(stage, f"{looks}/Zone{i}_Post", (0.1, 0.1, 0.12), (r, g, b), 2.5),
        }

    # ── 1) 등급별 적치 구역(웹페이지: 2×2 그리드, 중량=북행/경량=남행, A=서/B=동) ──
    zw, zd = C.ZONE_SIZE
    for z in C.ZONES:
        cx, cy = z["cx"], z["cy"]
        zm = zone_mats[z["name"]]
        base = f"{root}/Zone_{z['key']}"
        # 바닥 페인트(얇은 데칼 - 철근이 지면(z=0)에 놓여도 시각적으로 묻히지 않게 4mm)
        box(stage, f"{base}/Paint", 1.0, zm["paint"], pos=(cx, cy, 0.003), scale=(zw, zd, 0.004))
        # 경계 스트립 4면(웹페이지의 발광 테두리)
        for j, (px, py, sx, sy) in enumerate([
            (cx, cy + zd / 2, zw, 0.09), (cx, cy - zd / 2, zw, 0.09),
            (cx - zw / 2, cy, 0.09, zd), (cx + zw / 2, cy, 0.09, zd),
        ]):
            box(stage, f"{base}/Edge{j}", 1.0, zm["edge"], pos=(px, py, 0.008), scale=(sx, sy, 0.010))
        # 코너 레이저 포스트 4개(웹페이지의 레이저 표시 코너)
        for j, (px, py) in enumerate([
            (cx - zw / 2, cy + zd / 2), (cx + zw / 2, cy + zd / 2),
            (cx - zw / 2, cy - zd / 2), (cx + zw / 2, cy - zd / 2),
        ]):
            cylinder(stage, f"{base}/Post{j}", radius=0.025, height=0.55, mat=mats["dark"],
                     pos=(px, py, 0.275))
            sphere(stage, f"{base}/Post{j}/Cap", radius=0.035, mat=zm["post"], pos=(0.0, 0.0, 0.30))
        # 명판: 북엣지에 세운 표지판(한글 텍스처) + 바닥 데칼
        png = TEXTURE_DIR / f"zone_{z['key']}.png"
        if png.is_file():
            textured_plane(stage, f"{base}/Board", png, 1.05, 0.32,
                           pos=(cx, cy + zd / 2 - 0.02, 0.78), emissive=0.55)
            box(stage, f"{base}/BoardBack", 1.0, mats["dark"], pos=(cx, cy + zd / 2 - 0.035, 0.78),
                scale=(1.11, 0.02, 0.38))
            for px in (cx - 0.48, cx + 0.48):
                cylinder(stage, f"{base}/BoardPost_{abs(px)*1000:.0f}", radius=0.018, height=0.78,
                         mat=mats["dark"], pos=(px, cy + zd / 2 - 0.02, 0.39))
            textured_plane(stage, f"{base}/FloorLabel", png, 1.6, 0.48,
                           pos=(cx, cy, 0.006), quat=quat_axis_angle((1.0, 0.0, 0.0), math.pi / 2),
                           emissive=0.0, opacity=0.95)

    # 적치장 전체 외곽 노란 박스(웹페이지의 12×8.5 노란 경계선)
    x0, y0, x1, y1 = C.ZONE_BORDER
    mx, my = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    for j, (px, py, sx, sy) in enumerate([
        (mx, y1, x1 - x0, 0.14), (mx, y0, x1 - x0, 0.14),
        (x0, my, 0.14, y1 - y0), (x1, my, 0.14, y1 - y0),
    ]):
        box(stage, f"{root}/ZoneBorder{j}", 1.0, mats["paint_y"], pos=(px, py, 0.006), scale=(sx, sy, 0.006))

    # ── 2) AIMOS 등급판정 게이트(트럭 진입로 북쪽, 웹페이지의 스캐너 타워) ──
    g = C.AIMOS_GATE
    base = f"{root}/AimosGate"
    for j, gx in enumerate((g["x"] - g["span"] / 2.0, g["x"] + g["span"] / 2.0)):
        box(stage, f"{base}/Column{j}", 1.0, mats["steel"], pos=(gx, g["y"], g["h"] / 2.0),
            scale=(0.26, 0.26, g["h"]))
        box(stage, f"{base}/Foot{j}", 1.0, mats["concrete"], pos=(gx, g["y"], 0.05), scale=(0.6, 0.6, 0.1))
    box(stage, f"{base}/Beam", 1.0, mats["steel"], pos=(g["x"], g["y"], g["h"]),
        scale=(g["span"] + 0.5, 0.22, 0.30))
    box(stage, f"{base}/Housing", 1.0, mats["dark"], pos=(g["x"], g["y"], g["h"] - 0.62),
        scale=(0.55, 0.35, 0.42), quat=quat_axis_angle((1.0, 0.0, 0.0), 0.45))
    # 스캔 플래시 링 + 상태등
    cylinder(stage, f"{base}/FlashRing", radius=0.16, height=0.05, mat=mats["glow_w"],
             pos=(g["x"], g["y"] - 0.19, g["h"] - 0.62), quat=quat_axis_angle((1.0, 0.0, 0.0), 0.45))
    box(stage, f"{base}/Lamp", 1.0, mats["lamp_g"], pos=(g["x"] + g["span"] / 2.0 - 0.02, g["y"], g["h"] + 0.22),
        scale=(0.12, 0.12, 0.14))
    # 제어반(기둥 하단)
    box(stage, f"{base}/Cabinet", 1.0, mats["dark"], pos=(g["x"] - g["span"] / 2.0 - 0.35, g["y"], 0.45),
        scale=(0.45, 0.3, 0.9))
    png = TEXTURE_DIR / "aimos_board.png"
    if png.is_file():
        textured_plane(stage, f"{base}/Board", png, 1.7, 0.36,
                       pos=(g["x"], g["y"] + 0.13, g["h"] - 0.02), emissive=0.4)
    # 판정 카메라 보조 광(렌더러에서만 적용)
    try:
        rl = UsdLux.RectLight.Define(stage, f"{base}/ScanLight")
        rl.CreateIntensityAttr(0.0)  # 평시 꺼짐 - 촬영 연출 시 런타임에서 올림
        rl.CreateWidthAttr(2.0)
        rl.CreateHeightAttr(0.4)
        rl.CreateColorAttr(Gf.Vec3f(0.8, 0.85, 1.0))
        m = Gf.Matrix4d()
        m.SetTranslate(Gf.Vec3d(g["x"], g["y"], g["h"] - 0.62))
        m.SetRotateOnly(Gf.Rotation(Gf.Vec3d(1.0, 0.0, 0.0), 66.0))
        UsdGeom.Xformable(rl).AddTransformOp().Set(m)
    except Exception:
        pass

    # ── 3) 제강소 배경 건물(서쪽, 웹페이지의 창·용광로·굴뚝·코일) ──
    b = C.BUILDING
    base = f"{root}/Mill"
    box(stage, f"{base}/Wall", 1.0, mats["concrete"], pos=((b["x0"] + b["x1"]) / 2.0, 0.0, b["h"] / 2.0),
        scale=(b["x1"] - b["x0"], b["len"], b["h"]))
    static_collision(stage, f"{base}/Wall")
    # 동쪽 면(야드 쪽) 창문 그리드
    win = emat(stage, f"{looks}/Window", (0.16, 0.22, 0.30), (0.35, 0.50, 0.75), 0.5, roughness=0.2, metallic=0.4)
    face_x = b["x1"] + 0.03
    n_y = int(b["len"] // 2.2)
    for iy in range(n_y):
        wy = -b["len"] / 2.0 + 1.1 + iy * 2.2
        if b["door_y0"] - 0.6 < wy < b["door_y1"] + 0.6 or b["furnace_y0"] - 0.6 < wy < b["furnace_y1"] + 0.6:
            continue
        for iz, wz in enumerate((2.2, 4.4, 6.6)):
            box(stage, f"{base}/Win_{iy}_{iz}", 1.0, win, pos=(face_x, wy, wz),
                scale=(0.06, 1.4, 1.1))
    # 샤프터 문 2짝
    dm = make_material(stage, f"{looks}/MillDoor", (0.35, 0.38, 0.42), roughness=0.7, metallic=0.4)
    for j, dy in enumerate((b["door_y0"], b["door_y1"])):
        box(stage, f"{base}/Door{j}", 1.0, dm, pos=(face_x, dy, 1.9), scale=(0.08, 1.8, 3.8))
    # 용광로(발광 패널 + 광원)
    fy = (b["furnace_y0"] + b["furnace_y1"]) / 2.0
    fire = emat(stage, f"{looks}/FurnaceGlow", (1.0, 0.45, 0.12), (1.0, 0.42, 0.10), 2.5)
    box(stage, f"{base}/Furnace", 1.0, dm, pos=(face_x, fy, 1.6), scale=(0.16, b["furnace_y1"] - b["furnace_y0"], 3.0))
    box(stage, f"{base}/FurnaceGlow", 1.0, fire, pos=(face_x + 0.09, fy, 1.4), scale=(0.03, 1.4, 1.6))
    try:
        fl = UsdLux.RectLight.Define(stage, f"{base}/FurnaceLight")
        fl.CreateIntensityAttr(6.0)
        fl.CreateWidthAttr(1.4)
        fl.CreateHeightAttr(1.6)
        fl.CreateColorAttr(Gf.Vec3f(1.0, 0.45, 0.15))
        m = Gf.Matrix4d()
        m.SetTranslate(Gf.Vec3d(face_x + 0.15, fy, 1.4))
        m.SetRotateOnly(Gf.Rotation(Gf.Vec3d(0.0, 1.0, 0.0), -90.0))
        UsdGeom.Xformable(fl).AddTransformOp().Set(m)
    except Exception:
        pass
    # 굴뚝 2개
    for j, cy in enumerate((-3.0, 3.0)):
        cylinder(stage, f"{base}/Stack{j}", radius=0.45, height=3.5, mat=mats["dark"],
                 pos=(b["x0"] + 1.2, cy, b["h"] + 1.75))
    # 코일 적치(웹 code: 야드에 눕힌 강판 코일) 2기둥 × 3단
    coil = make_material(stage, f"{looks}/Coil", (0.48, 0.52, 0.58), roughness=0.35, metallic=0.85)
    hole = make_material(stage, f"{looks}/CoilHole", (0.08, 0.08, 0.09), roughness=0.9)
    for j, (kx, ky) in enumerate(((b["x1"] + 0.95, 1.2), (b["x1"] + 0.95, 3.2))):
        for k in range(3):
            cz = 0.28 + k * 0.52
            cylinder(stage, f"{base}/Coil{j}_{k}", radius=0.52, height=0.5, mat=coil, pos=(kx, ky, cz))
            cylinder(stage, f"{base}/Coil{j}_{k}/Hole", radius=0.26, height=0.52, mat=hole,
                     pos=(0.0, 0.0, 0.0))
    # 벽 상부 안전 난간(웹 code: 지주 + 2횡대)
    rail = make_material(stage, f"{looks}/Rail", (0.75, 0.22, 0.12), roughness=0.5, metallic=0.5)
    ry = b["len"] / 2.0 - 0.1
    for j in range(int(ry / 1.5) + 1):
        py = -ry + j * 1.5
        cylinder(stage, f"{base}/RailPost{j}", radius=0.025, height=1.0, mat=rail,
                 pos=(b["x1"] - 0.1, py, b["h"] + 0.5))
    for k, rz in enumerate((b["h"] + 0.85, b["h"] + 1.45)):
        box(stage, f"{base}/RailBar{k}", 1.0, rail, pos=(b["x1"] - 0.1, 0.0, rz),
            scale=(0.04, b["len"] - 0.2, 0.04))

    # ── 4) 트럭 진입 차선 표시(웹페이지: 남쪽에서 진입) ──
    lane = f"{root}/Lane"
    for j in range(6):
        dy = -6.9 + j * 0.62
        box(stage, f"{lane}/Dash{j}", 1.0, mats["paint_w"], pos=(C.TRUCK_XY[0], dy, 0.004),
            scale=(0.12, 0.35, 0.004))
    box(stage, f"{lane}/StopLine", 1.0, mats["paint_w"], pos=(C.TRUCK_XY[0], -3.75, 0.004),
        scale=(2.6, 0.14, 0.004))
    for j, side in enumerate((-1.3, 1.3)):
        for k in range(5):
            box(stage, f"{lane}/Edge{ j }_{k}", 1.0, mats["paint_y"],
                pos=(C.TRUCK_XY[0] + side, -6.6 + k * 0.72, 0.004), scale=(0.1, 0.4, 0.004))

    # ── 5) 안전 소품(배리어·드럼·타이어·CCTV·배전반) ──
    barrier = make_material(stage, f"{looks}/Barrier", (0.72, 0.73, 0.75), roughness=0.8)
    for j, (bx, by, yaw) in enumerate(
        [(x, -7.55, 0.0) for x in np.arange(1.2, 17.5, 1.8)] +
        [(17.55, y, math.pi / 2.0) for y in np.arange(-6.0, 6.5, 1.8)]
    ):
        p = f"{root}/Barrier{j}"
        box(stage, f"{p}/Base", 1.0, barrier, scale=(1.6, 0.34, 0.4), pos=(bx, by, 0.20),
            quat=quat_axis_angle((0.0, 0.0, 1.0), yaw))
        box(stage, f"{p}/Top", 1.0, mats["paint_y"], scale=(1.5, 0.26, 0.16), pos=(bx, by, 0.48),
            quat=quat_axis_angle((0.0, 0.0, 1.0), yaw))
        static_collision(stage, f"{p}/Base")
    drum_colors = ((0.15, 0.35, 0.6), (0.6, 0.25, 0.1), (0.2, 0.2, 0.22))
    for j, (dx, dy) in enumerate(((16.4, -5.9), (16.8, -5.35), (16.15, -4.95))):
        dm2 = make_material(stage, f"{looks}/Drum{j}", drum_colors[j % 3], roughness=0.6, metallic=0.3)
        cylinder(stage, f"{root}/Drum{j}", radius=0.30, height=0.9, mat=dm2, pos=(dx, dy, 0.45))
        cylinder(stage, f"{root}/Drum{j}/Rim", radius=0.31, height=0.06, mat=mats["dark"], pos=(0.0, 0.0, 0.2))
        static_collision(stage, f"{root}/Drum{j}")
    tyre = make_material(stage, f"{looks}/Tyre", (0.06, 0.06, 0.07), roughness=0.95)
    hub = make_material(stage, f"{looks}/Hub", (0.55, 0.57, 0.6), roughness=0.4, metallic=0.8)
    for j, (tx, ty) in enumerate(((15.7, 5.8), (16.35, 5.45), (15.4, 5.0))):
        for k in range(2):
            cylinder(stage, f"{root}/Tyre{j}_{k}", radius=0.34, height=0.16, mat=tyre,
                     pos=(tx, ty, 0.08 + k * 0.17))
        cylinder(stage, f"{root}/Tyre{j}_hub", radius=0.14, height=0.38, mat=hub, pos=(tx, ty, 0.19))
        static_collision(stage, f"{root}/Tyre{j}_0")
    # CCTV 폴 2기(건물 모서리 + 동쪽 울타리)
    for j, (px, py, yaw) in enumerate(((0.2, -6.9, 0.6), (17.6, 6.7, math.pi + 0.6))):
        p = f"{root}/CCTV{j}"
        cylinder(stage, f"{p}/Pole", radius=0.05, height=3.0, mat=mats["dark"], pos=(px, py, 1.5))
        box(stage, f"{p}/Head", 1.0, mats["dark"], pos=(px, py, 3.05), scale=(0.14, 0.30, 0.14),
            quat=quat_axis_angle((0.0, 0.0, 1.0), yaw))
        box(stage, f"{p}/Lens", 1.0, mats["glow_w"], pos=(px, py, 3.05), scale=(0.16, 0.06, 0.08),
            quat=quat_axis_angle((0.0, 0.0, 1.0), yaw))
    # 배전반 함
    box(stage, f"{root}/UtilityBox", 1.0, mats["steel"], pos=(0.55, 6.2, 0.6), scale=(0.7, 0.5, 1.2))
    static_collision(stage, f"{root}/UtilityBox")

    # ── 6) 남동쪽 자투리 분철 더미(웹페이지 Re 적치장의 잔여 분철) ──
    rng = np.random.default_rng(11)
    for j in range(16):
        a = rng.uniform(0, 2 * math.pi)
        px = 1.6 + rng.uniform(-0.5, 0.5)
        py = 4.0 + rng.uniform(-0.5, 0.5)
        _bent_piece(stage, f"{root}/Scrap{j}", length=0.6, angle=rng.uniform(1.2, 2.4),
                    yaw=a, pos=(px, py, 0.05 + rng.uniform(0, 0.12)), mat=mats["rust"])
        static_collision(stage, f"{root}/Scrap{j}")
    return root


# ═══════════════════════════ truck_load.usd ═══════════════════════════
def build_truck(stage, root="/Truck"):
    """덤프트럭(6륜) + 적재대 위 시각용 분철 더미. 더미는 정적 충돌체라서
    그 위에 떨어뜨린 개별 강체(rebar_set.usd)가 얹혀 안착한다."""
    looks = f"{root}/Looks"
    body = make_material(stage, f"{looks}/TruckBody", (0.66, 0.68, 0.71), roughness=0.45, metallic=0.35)
    dark = make_material(stage, f"{looks}/TruckDark", (0.13, 0.14, 0.16), roughness=0.6, metallic=0.3)
    glass = make_material(stage, f"{looks}/TruckGlass", (0.25, 0.35, 0.45), roughness=0.1, metallic=0.5,
                          opacity=0.7)
    bed = make_material(stage, f"{looks}/TruckBed", (0.40, 0.28, 0.16), roughness=0.75, metallic=0.25)
    rust1 = make_material(stage, f"{looks}/Rod1", (0.48, 0.28, 0.14), roughness=0.8, metallic=0.35)
    rust2 = make_material(stage, f"{looks}/Rod2", (0.55, 0.38, 0.20), roughness=0.75, metallic=0.35)
    rust3 = make_material(stage, f"{looks}/Rod3", (0.38, 0.22, 0.12), roughness=0.85, metallic=0.3)
    steelblue = make_material(stage, f"{looks}/Bent", (0.35, 0.40, 0.47), roughness=0.45, metallic=0.7)
    tx, ty = C.TRUCK_XY
    bed_top = C.TRUCK_BED_Z

    # 샤시
    box(stage, f"{root}/Frame1", 1.0, dark, pos=(tx, ty + 0.45, 0.62), scale=(5.4, 0.12, 0.18))
    box(stage, f"{root}/Frame2", 1.0, dark, pos=(tx, ty - 0.45, 0.62), scale=(5.4, 0.12, 0.18))
    # 운전석(동쪽 끝)
    cab_cx = tx + 2.6
    box(stage, f"{root}/Cab", 1.0, body, pos=(cab_cx, ty, 1.35), scale=(1.55, 2.05, 1.45))
    box(stage, f"{root}/CabGlass", 1.0, glass, pos=(cab_cx - 0.76, ty, 1.62), scale=(0.06, 1.85, 0.65))
    box(stage, f"{root}/CabDeflector", 1.0, body, pos=(cab_cx + 0.35, ty, 2.25), scale=(0.85, 1.85, 0.35),
        quat=quat_axis_angle((0.0, 1.0, 0.0), -0.2))
    box(stage, f"{root}/Bumper", 1.0, mats_bumper(stage, looks), pos=(cab_cx + 0.85, ty, 0.55),
        scale=(0.18, 2.15, 0.5))
    hl = emat(stage, f"{looks}/HeadLight", (0.95, 0.93, 0.8), (0.95, 0.93, 0.75), 3.0)
    for j, hy in enumerate((ty - 0.75, ty + 0.75)):
        box(stage, f"{root}/HeadLight{j}", 1.0, hl, pos=(cab_cx + 0.93, hy, 0.72), scale=(0.05, 0.28, 0.16))
    cylinder(stage, f"{root}/Exhaust", radius=0.06, height=1.8, mat=dark, pos=(cab_cx - 0.85, ty + 0.9, 1.4))
    for j, fy in enumerate((ty - 0.78, ty + 0.78)):
        cylinder(stage, f"{root}/FuelTank{j}", radius=0.24, height=0.95, mat=body,
                 pos=(cab_cx - 0.6, fy, 0.55), quat=quat_axis_angle((0.0, 1.0, 0.0), math.pi / 2.0))
    # 적재대(바닥 윗면 = TRUCK_BED_Z)
    box(stage, f"{root}/BedFloor", 1.0, bed, pos=(tx, ty, bed_top - 0.06), scale=(3.6, 1.9, 0.12))
    for side, ry in (("L", ty + 0.925), ("R", ty - 0.925)):
        box(stage, f"{root}/BedRail{side}", 1.0, bed, pos=(tx, ry, bed_top + 0.17), scale=(3.6, 0.05, 0.34))
    box(stage, f"{root}/BedTail", 1.0, bed, pos=(tx - 1.775, ty, bed_top + 0.17), scale=(0.05, 1.9, 0.34))
    box(stage, f"{root}/BedHead", 1.0, bed, pos=(tx + 1.775, ty, bed_top + 0.17), scale=(0.05, 1.9, 0.34))
    tl = emat(stage, f"{looks}/TailLight", (0.7, 0.08, 0.06), (0.8, 0.1, 0.08), 1.5)
    for j, hy in enumerate((ty - 0.7, ty + 0.7)):
        box(stage, f"{root}/TailLight{j}", 1.0, tl, pos=(tx - 1.81, hy, bed_top + 0.2), scale=(0.04, 0.2, 0.1))
    # 바퀴(조향 1축 + 구동 2축)
    wheel_q = quat_axis_angle((1.0, 0.0, 0.0), math.pi / 2.0)
    tyre = make_material(stage, f"{looks}/Wheel", (0.07, 0.07, 0.08), roughness=0.95)
    hub = make_material(stage, f"{looks}/WheelHub", (0.6, 0.62, 0.65), roughness=0.35, metallic=0.85)
    for i, ax in enumerate((tx + 2.3, tx - 1.5, tx - 2.5)):
        for j, wy in enumerate((ty - 0.82, ty + 0.82)):
            cylinder(stage, f"{root}/Wheel{i}{j}", radius=0.42, height=0.32, mat=tyre,
                     pos=(ax, wy, 0.42), quat=wheel_q)
            cylinder(stage, f"{root}/Wheel{i}{j}/Hub", radius=0.16, height=0.34, mat=hub, pos=(0.0, 0.0, 0.0),
                     quat=quat_axis_angle((1.0, 0.0, 0.0), math.pi / 2.0))
    # 충돌체: 적재대 관련(강체가 더미 위에서 굴러떨어지지 않게)
    for name in ("BedFloor", "BedRailL", "BedRailR", "BedTail", "BedHead"):
        static_collision(stage, f"{root}/{name}")

    # ── 시각용 분철 더미(정적): 3개 다발 × (막대 2층 + 상단 굽은 분철) ──
    rng = np.random.default_rng(5)
    rod_mats = (rust1, rust2, rust3)
    r = C.REBAR_RADIUS + 0.001
    for bi, dx in enumerate((-1.15, 0.0, 1.15)):
        bx = tx + dx
        # 1층 8본( Y방향 장축)
        for k in range(8):
            py = ty - 0.66 + k * 0.19
            capsule(stage, f"{root}/Mound{bi}_L1_{k}", radius=r, height=C.REBAR_LENGTH - 2 * r,
                    mat=rod_mats[k % 3], pos=(bx + rng.uniform(-0.012, 0.012), py + rng.uniform(-0.01, 0.01), bed_top + r),
                    quat=quat_axis_angle((1.0, 0.0, 0.0), math.pi / 2.0))
            static_collision(stage, f"{root}/Mound{bi}_L1_{k}")
        # 2층 7본(홀수 위치에 얹힘)
        for k in range(7):
            py = ty - 0.57 + k * 0.19
            capsule(stage, f"{root}/Mound{bi}_L2_{k}", radius=r, height=C.REBAR_LENGTH - 2 * r,
                    mat=rod_mats[(k + 1) % 3], pos=(bx + rng.uniform(-0.012, 0.012), py, bed_top + 3 * r),
                    quat=quat_axis_angle((1.0, 0.0, 0.0), math.pi / 2.0))
            static_collision(stage, f"{root}/Mound{bi}_L2_{k}")
        # 3층 3본(마루)
        for k in range(3):
            py = ty - 0.38 + k * 0.38
            capsule(stage, f"{root}/Mound{bi}_L3_{k}", radius=r, height=C.REBAR_LENGTH - 2 * r,
                    mat=rod_mats[(k + 2) % 3], pos=(bx, py, bed_top + 5 * r),
                    quat=quat_axis_angle((1.0, 0.0, 0.0), math.pi / 2.0))
            static_collision(stage, f"{root}/Mound{bi}_L3_{k}")
        # 상단 굽은 분철 4개(거의 눕힘)
        for k in range(4):
            _bent_piece(stage, f"{root}/Mound{bi}_Bent{k}", length=0.55, angle=rng.uniform(1.9, 2.6),
                        yaw=rng.uniform(0, 2 * math.pi), mat=steelblue,
                        pos=(bx + rng.uniform(-0.25, 0.25), ty - 0.5 + k * 0.34, bed_top + 5 * r + 0.01))
            static_collision(stage, f"{root}/Mound{bi}_Bent{k}")
    return root


def mats_bumper(stage, looks):
    return make_material(stage, f"{looks}/Bumper", (0.55, 0.57, 0.6), roughness=0.5, metallic=0.6)


# ═══════════════════════════ rebar_set.usd ═══════════════════════════
def _bent_piece(stage, path, length, angle, yaw, pos, mat, radius=None, dynamic=False, mass=None):
    """두 캡슐을 꺾은 분철 형상. dynamic=True면 하나의 리지드바디로 구성."""
    r = radius if radius is not None else C.REBAR_RADIUS
    root_prim = UsdGeom.Xform.Define(stage, path)
    from usd_utils import set_xform, get_op
    op = get_op(root_prim, create=True)
    set_xform(op, pos, quat_axis_angle((0.0, 0.0, 1.0), yaw))
    if dynamic:
        UsdPhysics.RigidBodyAPI.Apply(root_prim.GetPrim())
        if mass:
            UsdPhysics.MassAPI.Apply(root_prim.GetPrim()).CreateMassAttr(float(mass))

    # 조인트 점을 원점으로: 세그먼트 A는 +Y, B는 XZ 평면에서 angle만큼 벌어짐
    la, lb = length, length * 0.85
    dir_a = np.array([0.0, 1.0, 0.0])
    d = np.array([0.0, math.cos(angle), math.sin(angle)])
    def seg(name, dvec, seg_len):
        capsule(stage, f"{path}/{name}", radius=r, height=max(seg_len - 2 * r, 0.02), mat=mat,
                pos=tuple(dvec * (seg_len / 2.0)), quat=quat_between((0.0, 0.0, 1.0), dvec / np.linalg.norm(dvec)))
    seg("SegA", dir_a, la)
    seg("SegB", -d, lb)
    for child in ("SegA", "SegB"):
        UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(f"{path}/{child}"))
    return root_prim


def build_rebar_set(stage, root="/RebarSet", num=C.REBAR_NUM, seed=7):
    """트럭 적재 분철 더미 위 1cm에서 떨어지는 개별 강체 ≤30개.

    직선 철근(캡슐)과 굽은 분철(2캡슐 강체)을 섞는다. 시뮬레이션 시작 직후
    ~1cm 낙하로 더미 위에 자연스럽게 안착한다(수백 개 강체의 충돌 비용 회피).
    """
    num = min(num, C.REBAR_MAX)
    looks = f"{root}/Looks"
    rusts = [make_material(stage, f"{looks}/RebarMat{j}", c, roughness=0.8, metallic=0.35)
             for j, c in enumerate(((0.50, 0.30, 0.15), (0.58, 0.40, 0.22), (0.42, 0.24, 0.13)))]
    steel = make_material(stage, f"{looks}/BentMat", (0.36, 0.41, 0.48), roughness=0.45, metallic=0.7)
    rng = np.random.default_rng(seed)
    tx, ty = C.TRUCK_XY
    r = C.REBAR_RADIUS
    n_straight = min(15, num)
    n_bent = num - n_straight
    top_z = C.TRUCK_BED_Z + 6 * (r + 0.001) + 0.01  # 더미 마루 + 1cm

    made = 0
    # 직선 철근: 3 다발 × 5본(트럭 Y방향 장축, 미세 요동)
    for bi, dx in enumerate((-1.15, 0.0, 1.15)):
        for k in range(5):
            if made >= n_straight:
                break
            yaw = math.pi / 2.0 + rng.uniform(-0.06, 0.06)
            quat = quat_axis_angle((1.0, 0.0, 0.0), yaw)
            p = f"{root}/rebar_{made}"
            capsule(stage, p, radius=r, height=C.REBAR_LENGTH - 2 * r, mat=rusts[made % 3],
                    pos=(tx + dx + rng.uniform(-0.02, 0.02), ty - 0.55 + k * 0.28 + rng.uniform(-0.02, 0.02),
                         top_z + r), quat=quat)
            _make_rigid(stage, p, mass=7.5)
            made += 1
    # 굽은 분철: 다발 사이 틈과 마루 위에 흩뿌림
    for k in range(n_bent):
        yaw = rng.uniform(0, 2 * math.pi)
        px = tx + rng.uniform(-1.5, 1.5)
        py = ty + rng.uniform(-0.6, 0.6)
        _bent_piece(stage, f"{root}/rebar_{made}", length=0.62, angle=rng.uniform(1.3, 2.5), yaw=yaw,
                    pos=(px, py, top_z + 0.02), mat=steel, dynamic=True, mass=5.8)
        made += 1
    return root


def _make_rigid(stage, path, mass):
    prim = stage.GetPrimAtPath(path)
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.CollisionAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(float(mass))


# ═══════════════════════════ crane.usd ═══════════════════════════
def build_crane(stage, root="/CraneRoot"):
    """갠트리 크레인 - 프리즘 3축 아큘레이션(주행 Y / 횡행 X / 권상 Z).

    정적 부분(레일+기둥)과 아큘레이션(포탈→트롤리→후크)으로 나뉜다.
    조인트 변위 0 = 포탈 y0, 트롤리 x0, 후크 z 3.5(작성 자세). 구동은 위치 제어
    드라이브(스티프니스/댐핑은 config.CRANE_DRIVE).
    """
    looks = f"{root}/Looks"
    yellow = make_material(stage, f"{looks}/Yellow", (0.85, 0.66, 0.10), roughness=0.5, metallic=0.3)
    gray = make_material(stage, f"{looks}/Gray", (0.30, 0.32, 0.36), roughness=0.6, metallic=0.4)
    dark = make_material(stage, f"{looks}/Dark", (0.12, 0.12, 0.14), roughness=0.7)
    red = make_material(stage, f"{looks}/Red", (0.80, 0.15, 0.12), roughness=0.4, metallic=0.5)

    # ── 정적: 고가 주행 레일 + 기둥(웹페이지: 기둥 위 레일) ──
    rail_top = 7.7
    for i, x in enumerate(C.RAIL_X):
        box(stage, f"{root}/Fixed/RailBeam{i}", 1.0, dark, pos=(x, 0.0, rail_top - 0.21),
            scale=(0.5, C.RAIL_LEN_Y + 1.0, 0.42))
        static_collision(stage, f"{root}/Fixed/RailBeam{i}")
        for j, y in enumerate(np.arange(-6.0, 6.1, 3.0)):
            box(stage, f"{root}/Fixed/Column{i}_{j}", 1.0, yellow, pos=(x, y, (rail_top - 0.42) / 2.0),
                scale=(0.34, 0.34, rail_top - 0.42))
            box(stage, f"{root}/Fixed/ColFoot{i}_{j}", 1.0, gray, pos=(x, y, 0.04), scale=(0.62, 0.62, 0.08))
            static_collision(stage, f"{root}/Fixed/Column{i}_{j}")
        # 레일 종단 스토퍼
        for j, sy in enumerate((-C.RAIL_LEN_Y / 2.0 - 0.3, C.RAIL_LEN_Y / 2.0 + 0.3)):
            box(stage, f"{root}/Fixed/Stop{i}_{j}", 1.0, red, pos=(x, sy, rail_top + 0.1), scale=(0.5, 0.12, 0.3))

    # ── 아큘레이션 루트 ──
    art = UsdGeom.Xform.Define(stage, f"{root}/CraneArticulation")
    UsdPhysics.ArticulationRootAPI.Apply(art.GetPrim())

    # 고정 베이스 링크(더미): 아큘레이션에서 world 직결 조인트는 무시되므로
    # 베이스→포탈 조인트로 주행 축을 만든다(로봇 페달링크와 같은 패턴).
    base = UsdGeom.Xform.Define(stage, f"{root}/CraneArticulation/Base")
    UsdPhysics.RigidBodyAPI.Apply(base.GetPrim())
    UsdPhysics.MassAPI.Apply(base.GetPrim()).CreateMassAttr(1.0)

    # 포탈(주행 Y): 다리 2 + 브리지 + 상부 보도
    portal = UsdGeom.Xform.Define(stage, f"{root}/CraneArticulation/Portal")
    UsdPhysics.RigidBodyAPI.Apply(portal.GetPrim())
    UsdPhysics.MassAPI.Apply(portal.GetPrim()).CreateMassAttr(2500.0)
    for i, x in enumerate(C.RAIL_X):
        box(stage, f"{root}/CraneArticulation/Portal/Leg{i}", 1.0, yellow, pos=(x, 0.0, rail_top + 0.31),
            scale=(0.44, 0.44, 0.62))
    bridge_mid_x = (C.RAIL_X[0] + C.RAIL_X[1]) / 2.0
    bridge_len = C.RAIL_X[1] - C.RAIL_X[0] + 0.9
    box(stage, f"{root}/CraneArticulation/Portal/Bridge", 1.0, yellow, pos=(bridge_mid_x, 0.0, rail_top + 0.72),
        scale=(bridge_len, 0.62, 0.58))
    # 보도 그리팅 + 난간(웹페이지의 정비 보도)
    box(stage, f"{root}/CraneArticulation/Portal/Walkway", 1.0, gray, pos=(bridge_mid_x, 0.42, rail_top + 1.06),
        scale=(bridge_len - 0.4, 0.7, 0.05))
    for j in range(9):
        wx = C.RAIL_X[0] + 0.4 + j * (bridge_len - 0.8) / 8.0
        cylinder(stage, f"{root}/CraneArticulation/Portal/WPost{j}", radius=0.02, height=0.9, mat=gray,
                 pos=(wx, 0.74, rail_top + 1.5))
    for k, rz in enumerate((rail_top + 1.35, rail_top + 1.9)):
        box(stage, f"{root}/CraneArticulation/Portal/WRail{k}", 1.0, gray,
            pos=(bridge_mid_x, 0.74, rz), scale=(bridge_len - 0.4, 0.035, 0.035))
    # 사선 브레이스(외관)
    for j, sgn in enumerate((1.0, -1.0)):
        box(stage, f"{root}/CraneArticulation/Portal/Brace{j}", 1.0, yellow, pos=(bridge_mid_x, 0.0, rail_top + 0.30),
            quat=(0.9806, 0.0, sgn * 0.1961, 0.0), scale=(bridge_len - 1.5, 0.10, 0.10))

    # 트롤리(횡행 X): 작성 자세 x=0, z=TROLLEY_Z. 브리지 밑에 매달림
    trolley = UsdGeom.Xform.Define(stage, f"{root}/CraneArticulation/Trolley")
    UsdPhysics.RigidBodyAPI.Apply(trolley.GetPrim())
    UsdPhysics.MassAPI.Apply(trolley.GetPrim()).CreateMassAttr(400.0)
    tz = C.TROLLEY_Z
    box(stage, f"{root}/CraneArticulation/Trolley/Body", 1.0, gray, pos=(0.0, 0.0, tz), scale=(0.95, 0.75, 0.50))
    box(stage, f"{root}/CraneArticulation/Trolley/Drum", 1.0, dark, pos=(0.0, 0.0, tz + 0.32),
        scale=(0.5, 0.42, 0.30))
    box(stage, f"{root}/CraneArticulation/Trolley/Motor", 1.0, dark, pos=(0.55, 0.0, tz + 0.30),
        scale=(0.28, 0.28, 0.42))
    for side, hy in enumerate((-0.30, 0.30)):
        box(stage, f"{root}/CraneArticulation/Trolley/Hanger{side}", 1.0, gray,
            pos=(0.0, hy, tz + 0.52), scale=(0.7, 0.06, 0.42))

    # 후크(권상 Z): 링크 원점 = 후크 판 중심. 작성 자세 z=3.5
    hook_rest_z = 3.5
    hook = UsdGeom.Xform.Define(stage, f"{root}/CraneArticulation/Hook")
    UsdPhysics.RigidBodyAPI.Apply(hook.GetPrim())
    UsdPhysics.MassAPI.Apply(hook.GetPrim()).CreateMassAttr(120.0)
    hook_path = f"{root}/CraneArticulation/Hook"
    box(stage, f"{hook_path}/Plate", 1.0, gray, pos=(0.0, 0.0, 0.0), scale=(0.30, 0.30, 0.09))
    box(stage, f"{hook_path}/CrossBar", 1.0, dark, pos=(0.0, 0.0, 0.20), scale=(0.62, 0.10, 0.10))
    for j, cy in enumerate((-0.18, 0.18)):
        cylinder(stage, f"{hook_path}/Bar{j}", radius=0.02, height=0.30, mat=dark,
                 pos=(0.0, cy, 0.10))
    magnet = cylinder(stage, f"{hook_path}/Magnet", radius=0.28, height=0.12,
                      mat=red, pos=(0.0, 0.0, -C.MAGNET_DROP))
    mring = emat(stage, f"{looks}/MagnetRing", (0.2, 0.02, 0.02), (1.0, 0.15, 0.1), 1.2)
    cylinder(stage, f"{hook_path}/Magnet/Ring", radius=0.285, height=0.03, mat=mring,
             pos=(0.0, 0.0, -0.045))
    cable_stub = capsule(stage, f"{hook_path}/CableStub", radius=0.02, height=0.25,
                         mat=dark, pos=(0.0, 0.0, 0.38))
    _m = Gf.Matrix4d()
    _m.SetTranslate(Gf.Vec3d(0.0, 0.0, hook_rest_z))
    hook.AddTransformOp().Set(_m)

    # ── 조인트 3축(전부 프리즘 + 위치 드라이브) ──
    def prismatic(path, body0, body1, axis, local_pos0, local_pos1, lo, hi, k, d, fmax):
        joint = UsdPhysics.PrismaticJoint.Define(stage, path)
        if body0 is not None:
            joint.CreateBody0Rel().SetTargets([body0])
        joint.CreateBody1Rel().SetTargets([body1])
        joint.CreateAxisAttr(axis)
        joint.CreateLowerLimitAttr(float(lo))
        joint.CreateUpperLimitAttr(float(hi))
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "linear")
        drive.CreateTargetPositionAttr(0.0)
        drive.CreateTargetVelocityAttr(0.0)
        drive.CreateMaxForceAttr(float(fmax))
        drive.CreateStiffnessAttr(float(k))
        drive.CreateDampingAttr(float(d))
        joint.CreateLocalPos0Attr(Gf.Vec3f(*local_pos0))
        joint.CreateLocalPos1Attr(Gf.Vec3f(*local_pos1))
        return joint

    art_path = f"{root}/CraneArticulation"
    tr, tt, th = C.CRANE_DRIVE["travel"], C.CRANE_DRIVE["trolley"], C.CRANE_DRIVE["hoist"]
    prismatic(f"{art_path}/PortalTravelJoint", f"{art_path}/Base", f"{art_path}/Portal", "Y",
              (bridge_mid_x, 0.0, rail_top + 0.7), (bridge_mid_x, 0.0, rail_top + 0.7),
              C.CRANE_Y_LIMITS[0], C.CRANE_Y_LIMITS[1], tr[0], tr[1], tr[2])
    prismatic(f"{art_path}/TrolleyTravelJoint", f"{art_path}/Portal", f"{art_path}/Trolley", "X",
              (0.0, 0.0, tz), (0.0, 0.0, tz),
              C.CRANE_X_LIMITS[0], C.CRANE_X_LIMITS[1], tt[0], tt[1], tt[2])
    prismatic(f"{art_path}/HoistJoint", f"{art_path}/Trolley", f"{art_path}/Hook", "Z",
              (0.0, 0.0, hook_rest_z - tz), (0.0, 0.0, 0.0),
              C.HOOK_Z_LIMITS[0] - hook_rest_z, C.HOOK_Z_LIMITS[1] - hook_rest_z, th[0], th[1], th[2])

    # 와이어(시각용 - 런타임이 앵커→후크로 늘린다)
    capsule(stage, f"{root}/Cable", radius=0.02, height=1.0, mat=dark, pos=(0.0, 0.0, 5.3))
    return root
