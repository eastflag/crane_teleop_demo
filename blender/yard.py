"""적치 야드 모델링 스크립트 (Blender bpy) - 웹 판본 그대로, 1차: 구역·배리어

crane_teleop_demo/drawing/원격하차크레인시뮬레이터.html 의 Three.js 씬에 나오는
야드를 Blender 로 되살린다. 이번 판은 전체 구조 잡기 전 1차분으로 웹에 구현된
'등급 적치 구역(Zone) 4곳'과 '외곽 콘크리트 배리어'만 넣는다. 바닥·건물·카메라
게이트·철근 더미 등 나머지는 전체 구조를 잡은 뒤 추가한다.

좌표계 (three.js → Blender 변환: (x,y,z) → (x, z, y), 크레인·트럭과 동일)
    X = 동서 (구역 4개가 x −5.9..5.9 에 2×2 로 배치)
    Y = 남북 (구역은 y −10.5..−2.3, 남쪽(y −)이 중량 행)
    Z = 높이 (바닥 z=0, 구역 마킹은 z≈0.05 레이어)

웹 판본 값 (씬 코드에서 추출한 그대로)
    Zone 4곳 ($l 배열) - x0/x1/z0/z1 와 색(Ci/is):
        중량A #8F87EE  x −5.9..−0.15, z −10.5..−6.45   (북서, 보라)
        중량B #4A9BEA  x  0.15..5.9,  z −10.5..−6.45   (북동, 파랑)
        경량A #2DB386  x −5.9..−0.15, z  −6.25..−2.3   (남서, 초록)
        경량B #F2A93B  x  0.15..5.9,  z  −6.25..−2.3   (남동, 주황)
      (웹의 z 는 Blender y 로 매핑. 서·동 사이 0.3 m, 행 사이 0.2 m 간격)
    구역당 마킹:
      - 테두리: 발광 박스 4개(폭 .09, 두께 .014, z .045..0.059) - 재질은
        존 색 + emissive 존 색 × 0.2 (웹 pe)
      - 라벨: 투명 PNG(512x128) 평면 2.2×0.55, z .062, 웹 opacity .9 는
        PNG 알파(230/255)에 구워 넣는다(Blender 5.2 USD 익스포터가 재질에
        넣은 opacity '값'은 1 로 내보내는 버그 회피). 웹의 700 80px
        Noto Sans KR 캔버스 → textures/zone_label_*.png 로 미리 렌더해 둠.
        파일이 없으면 존 색 단색 밴드로 폴백
      - 필름(하이라이트 쿼드): 존 색 반투명 평면, 웹 기본 opacity 0(런타임
        깜빡임용) - USD 프리뷰에서 구역이 보이도록 0.2 로 고정. 이 값은
        내보내기 후 fix_usd_opacity() 후처리(pxr)로 USD 에 직접 기록한다
    배리어 (concrete_road_barrier GLB 클론):
      - 동쪽 라인: x=9.2, y −14..13.2 를 1.6 간격 18개 (길이방향 Y)
      - 남쪽 라인: y=−15.2, x −8..8 을 1.6 간격 11개 (길이방향 X)
      - 형상: GLB 실측 1.545(길이)×0.64(폭)×0.83(높이) 저지 배리어를
        절차 프로파일(사다리꼴 단면 6점)로 재현

라벨 PNG 생성(재현용, 이미 textures/ 에 들어 있음):
    PIL + /usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc 80px,
    투명 캔버스 512x128, 존 색 글씨, 중앙 baseline y=96

이름 규칙 - USD 변환 시 한글이 깨지므로 오브젝트·재질 이름은 ASCII 영어만
쓴다(ZoneHA, ZoneBorder_HA, Barrier_E_00, ...). 주석만 한글.

사용법
    GUI      : Blender 의 Scripting 탭에서 이 파일을 열고 Run Script
    터미널   : blender --background --python blender/yard.py
    USD 내보내기 :
        blender --background --python blender/yard.py -- \
            --export ../assets/yard_blender.usd
"""

import sys


def P(*a):
    """진행 상황 출력(헤드리스 실행에서도 즉시 보이도록 flush)."""
    print(*a, flush=True)


import bpy
import bmesh
import os
from math import pi
from mathutils import Matrix, Vector, Euler

# --------------------------------------------------------------------------
# 파라미터 - 웹 씬 코드에서 추출한 값 그대로
# --------------------------------------------------------------------------
# (키, 한국어 이름, hex, x0, x1, z0(web z→blender y), z1)
ZONES = [
    ("HA", "중량A", "8F87EE", -5.9, -0.15, -10.5, -6.45),
    ("HB", "중량B", "4A9BEA",  0.15,  5.9, -10.5, -6.45),
    ("LA", "경량A", "2DB386", -5.9, -0.15,  -6.25, -2.3),
    ("LB", "경량B", "F2A93B",  0.15,  5.9,  -6.25, -2.3),
]
ZONE_MARK_Z = 0.052          # 웹: 테두리 박스 중심 y (박스 두께 .014)
ZONE_BORDER_W = 0.09         # 웹: 테두리 박스 폭
ZONE_LABEL_Z = 0.062         # 웹: 라벨 평면 y
ZONE_LABEL_SIZE = (2.2, 0.55)  # 웹: 라벨 평면 크기
ZONE_LABEL_OPACITY = 0.9     # 웹: 라벨 평면 opacity
ZONE_FILL_Z = 0.056          # 웹: 하이라이트 쿼드 y
ZONE_FILL_OPACITY = 0.2      # 웹 기본 0(런타임 깜빡임) - 프리뷰 가시용 0.2

# 배리어 - 웹: for(z=-14;z<=14;z+=1.6) pos(9.2,0,z) rot90 / for(x=-8;x<=8;x+=1.6) pos(x,0,-15.2)
BARRIER_LEN, BARRIER_W, BARRIER_H = 1.545, 0.64, 0.83   # GLB 실측
BARRIER_GAP = 1.6
BARRIER_EAST_X = 9.2
BARRIER_EAST_Y0, BARRIER_EAST_Y1 = -14.0, 14.0
BARRIER_SOUTH_Y = -15.2
BARRIER_SOUTH_X0, BARRIER_SOUTH_X1 = -8.0, 8.0

def _tex_dir():
    """textures/ 폴더 찾기. GUI 에서 열 때 __file__ 이 스크립트 이름만
    ('/yard.py')로 주어지는 경우가 있어 후보를 여러 개 확인한다:
    스크립트 폴더 → 현재 작업 폴더/blender/textures → 현재 작업 폴더/textures."""
    here = os.path.dirname(os.path.abspath(__file__)) if "__file__" in globals() else os.getcwd()
    cands = [os.path.join(here, "textures"),
             os.path.join(os.getcwd(), "blender", "textures"),
             os.path.join(os.getcwd(), "textures")]
    for d in cands:
        if os.path.isdir(d):
            return d
    return cands[0]


TEX_DIR = _tex_dir()

COLL = None            # 만들어진 오브젝트가 들어가는 콜렉션 (main 에서 설정)


# --------------------------------------------------------------------------
# 재질 헬퍼 (crane.py/truck_load.py 와 같은 구성)
# --------------------------------------------------------------------------
def _hex(s):
    if isinstance(s, str):
        s = int(s, 16)
    return ((s >> 16 & 255) / 255, (s >> 8 & 255) / 255, (s & 255) / 255, 1.0)


def _set(node, names, value):
    for n in names:
        if n in node.inputs:
            node.inputs[n].default_value = value
            return


def make_material(name, hexcol, rough=0.5, metal=0.5, emit=None, emit_strength=0.0,
                  alpha=1.0, map_img=None, map_alpha=False):
    """Principled BSDF 재질. map_img=이미지 → Base Color 연결(알파 사용 시
    투명 재질). 웹의 ct(..., {emissive, emissiveIntensity}) 재현."""
    mat = bpy.data.materials.get(name)
    if mat is None:
        mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    nt = mat.node_tree
    for n in list(nt.nodes):                      # 재실행 시 최신 설정으로 재구성
        nt.nodes.remove(n)
    nodes, links = nt.nodes, nt.links
    out = nodes.new("ShaderNodeOutputMaterial")
    out.location = (140, 300)
    bsdf = nodes.new("ShaderNodeBsdfPrincipled")
    bsdf.location = (-140, 300)
    links.new(bsdf.outputs["BSDF"], out.inputs["Surface"])

    _set(bsdf, ["Base Color"], _hex(hexcol))
    _set(bsdf, ["Metallic"], metal)
    _set(bsdf, ["Roughness"], rough)
    if emit is not None:
        _set(bsdf, ["Emission Color", "Emission"], _hex(emit))
        _set(bsdf, ["Emission Strength"], emit_strength)
    if alpha < 1.0:
        _set(bsdf, ["Alpha"], alpha)
        mat.blend_method = "BLEND"

    if map_img is not None:
        tex = nodes.new("ShaderNodeTexImage")
        tex.location = (-640, 300)
        tex.image = map_img
        links.new(tex.outputs["Color"], bsdf.inputs["Base Color"])
        if map_alpha:
            links.new(tex.outputs["Alpha"], bsdf.inputs["Alpha"])
            mat.blend_method = "BLEND"
    return mat


def build_materials():
    """구역 재질(테두리 발광/라벨/필름)과 콘크리트 배리어 재질."""
    m = {}
    for key, _name, hexcol, *_ in ZONES:
        # 웹 pe: ct(색, rough .6, metal 0, {emissive: 색, emissiveIntensity .2})
        m[f"border_{key}"] = make_material(f"ZoneBorder_{key}", hexcol,
                                           rough=.6, metal=0., emit=hexcol,
                                           emit_strength=.2)
        # 라벨: PNG 텍스처(투명). 파일 없으면 존 색 단색 밴드 이미지로 폴백.
        img = _load_label_img(key, hexcol)
        m[f"label_{key}"] = make_material(f"ZoneLabel_{key}", "FFFFFF",
                                          rough=.6, metal=0.,
                                          alpha=ZONE_LABEL_OPACITY,
                                          map_img=img, map_alpha=True)
        # 필름(하이라이트): 웹은 opacity 0 에서 런타임 점멸 - 프리뷰용 0.2 고정
        m[f"fill_{key}"] = make_material(f"ZoneFill_{key}", hexcol,
                                         rough=.6, metal=0., alpha=ZONE_FILL_OPACITY)
    m["concrete"] = make_material("Barrier_Concrete", "B5B1A8", rough=.92, metal=0.)
    return m


def _load_label_img(key, hexcol):
    """웹 캔버스(512x128, 투명 배경, 존 색 한글 글씨)를 재현한 PNG 로드.
    없으면 절차 생성(투명 캔버스 + 존 색 가운데 밴드)해 동작을 보장한다."""
    fname = f"zone_label_{key}.png"
    img = bpy.data.images.get(fname)
    if img is None:
        path = os.path.join(TEX_DIR, fname)
        if os.path.isfile(path):
            img = bpy.data.images.load(path)
            img.name = fname
        else:
            # 폴백: 텍스트 없는 존 색 밴드 (512x128 전체 픽셀 채우기)
            img = bpy.data.images.new(fname, width=512, height=128)
            img.colorspace_settings.name = "sRGB"   # 픽셀 할당 전에(5.2 버그)
            col = _hex(hexcol)[:3]
            row_on = (*col, 1.0) * 512              # 글자 밴드에 해당하는 한 줄
            row_off = (0.0, 0.0, 0.0, 0.0) * 512    # 투명 한 줄
            px = []
            for yy in range(128):
                px.extend(row_on if 34 <= yy <= 96 else row_off)
            img.pixels = px
            img.pack()
    img.colorspace_settings.name = "sRGB"
    return img


# --------------------------------------------------------------------------
# 지오메트리 헬퍼 (bmesh 만 사용)
# --------------------------------------------------------------------------
CUBE_CORNERS = [(-1, -1, -1), (1, -1, -1), (1, 1, -1), (-1, 1, -1),
                (-1, -1, 1), (1, -1, 1), (1, 1, 1), (-1, 1, 1)]
CUBE_FACES = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (3, 7, 6, 2), (0, 4, 7, 3), (1, 2, 6, 5)]


class Part:
    """하나의 Blender 오브젝트로 나갈 부품군. 원시형을 추가하고 한 번에 메시로 굽는다."""

    def __init__(self, name, materials):
        self.name = name
        self.materials = materials
        self.bm = bmesh.new()

    def box(self, size, loc, mat_idx=0, bevel=0.0, rot=None):
        """size=(x,y,z) 중심 loc (웹 je/dn 헬퍼)."""
        bm = self.bm
        pre_v, pre_f, pre_e = set(bm.verts), set(bm.faces), set(bm.edges)
        vs = [bm.verts.new(Vector(c) * (Vector(size) * .5)) for c in CUBE_CORNERS]
        for f in CUBE_FACES:
            bm.faces.new([vs[i] for i in f])
        if bevel > 0:
            b = min(bevel, min(size) * .24)
            if b >= 1e-3:
                try:
                    bmesh.ops.bevel(bm, geom=[e for e in bm.edges if e not in pre_e],
                                    offset=b, offset_type="WIDTH",
                                    segments=3, profile=.7, affect="EDGES")
                except Exception:
                    pass
        self._place(pre_v, loc, rot)
        self._paint(pre_f, mat_idx)

    def plane(self, size, loc, mat_idx=0, rot=None):
        """바닥 평면용 얇은 쿼드(웹 PlaneGeometry + rot.x=-π/2 에 해당)."""
        bm = self.bm
        pre_v, pre_f = set(bm.verts), set(bm.faces)
        half = Vector((size[0] * .5, size[1] * .5, 0.0))
        vs = [bm.verts.new(Vector(c) * half)
              for c in ((-1, -1, 0), (1, -1, 0), (1, 1, 0), (-1, 1, 0))]
        face = bm.faces.new(vs)
        # 평면 UV (0..1) - 라벨 텍스처용
        uv = bm.loops.layers.uv.new("UVMap")
        for l in face.loops:
            c = l.vert.co
            l[uv].uv = ((c.x / size[0] + .5), (c.y / size[1] + .5))
        self._place(pre_v, loc, rot)
        self._paint(pre_f, mat_idx)

    def extrude_profile(self, profile, length, loc=None, rot=None):
        """XZ 평면 2D 프로파일[(x,z)...]을 Y 방향으로 깎아낸 프리즘(배리어 단면)."""
        bm = self.bm
        pre_v, pre_f = set(bm.verts), set(bm.faces)
        n = len(profile)
        face_a = [bm.verts.new(Vector((x, -length / 2, z))) for x, z in profile]
        face_b = [bm.verts.new(Vector((x,  length / 2, z))) for x, z in profile]
        bm.faces.new(face_a[::-1])
        bm.faces.new(face_b)
        for i in range(n):
            j = (i + 1) % n
            bm.faces.new((face_a[i], face_a[j], face_b[j], face_b[i]))
        bmesh.ops.recalc_face_normals(bm, faces=bm.faces)   # 면 방향 정규화
        if loc is not None or rot is not None:
            self._place(pre_v, loc or (0, 0, 0), rot)
        self._paint(pre_f, 0)

    def _place(self, pre_v, loc, rot):
        m = Matrix.Translation(Vector(loc))
        if rot is not None:
            m = m @ (rot.to_matrix().to_4x4() if isinstance(rot, Euler) else rot)
        bmesh.ops.transform(self.bm, matrix=m, verts=[v for v in self.bm.verts if v not in pre_v])

    def _paint(self, pre_f, mat_idx):
        for f in self.bm.faces:
            if f not in pre_f:
                f.material_index = mat_idx

    def finish(self, parent=None):
        me = bpy.data.meshes.new(self.name)
        self.bm.to_mesh(me)
        self.bm.free()
        for mat in self.materials:
            me.materials.append(mat)
        obj = bpy.data.objects.new(self.name, me)
        COLL.objects.link(obj)
        if parent:
            obj.parent = parent
        return obj


# --------------------------------------------------------------------------
# 구역(Zone) 4곳 - 웹 $l.map(...) 그대로
# --------------------------------------------------------------------------
def build_zone(mats, zones_root, key, name_ko, hexcol, x0, x1, y0, y1):
    """구역 하나 = 발광 테두리 박스 4개 + 한글 라벨 평면 + 하이라이트 필름."""
    grp = bpy.data.objects.new(f"Zone{key}", None)
    grp.empty_display_size = 0.5
    grp.parent = zones_root
    COLL.objects.link(grp)

    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    wx, wy = x1 - x0, y1 - y0

    # 테두리 박스 4개 - 웹처럼 구역 경계선 중심에 놓인다(절반씩 안팎)
    p = Part(f"ZoneBorder_{key}", [mats[f"border_{key}"]])
    p.box((wx, ZONE_BORDER_W, .014), (cx, y0, ZONE_MARK_Z))
    p.box((wx, ZONE_BORDER_W, .014), (cx, y1, ZONE_MARK_Z))
    p.box((ZONE_BORDER_W, wy, .014), (x0, cy, ZONE_MARK_Z))
    p.box((ZONE_BORDER_W, wy, .014), (x1, cy, ZONE_MARK_Z))
    p.finish(grp)

    # 한글 라벨 평면 - 중량(북쪽 행)은 북쪽 가장자리, 경량(남쪽 행)은 남쪽 가장자리
    # (웹: E<2 ? z0+.5 : z1-.5, rot.x=-π/2 → Blender 평면도 X축 −90° 로 위를 향하게)
    ly = y0 + 0.5 if key in ("HA", "HB") else y1 - 0.5
    lp = Part(f"ZoneLabel_{key}", [mats[f"label_{key}"]])
    lp.plane(ZONE_LABEL_SIZE, (cx, ly, ZONE_LABEL_Z),
             rot=Euler((-pi / 2, 0, 0)))
    lp.finish(grp)

    # 하이라이트 필름(쿼드) - 웹 fill: 런타임 점멸용, 여기선 반투명 고정
    fp = Part(f"ZoneFill_{key}", [mats[f"fill_{key}"]])
    fp.plane((wx - .12, wy - .12), (cx, cy, ZONE_FILL_Z),
             rot=Euler((-pi / 2, 0, 0)))
    fp.finish(grp)
    return grp


# --------------------------------------------------------------------------
# 콘크리트 배리어 - 웹 concrete_road_barrier 클론 배치 그대로
# --------------------------------------------------------------------------
# 저지 배리어 단면(XZ 평면, +x 반쪽만 정의하고 미러로 닫는다) -
# GLB 실측 0.64폭×0.83높이의 전형적인 F형 프로파일 근사.
_BARRIER_HALF = [(.320, 0.0), (.320, 0.10), (.240, 0.24), (.110, 0.60),
                 (.085, 0.72), (.085, 0.83)]


def build_barrier_template(mats):
    """배리어 원본 메시 1개(길이 Y, 바닥 z=0)를 만들어 반환(복제용 데이터)."""
    prof = [(x, z) for x, z in reversed(_BARRIER_HALF)] + \
           [(-x, z) for x, z in _BARRIER_HALF]
    p = Part("BarrierTemplate", [mats["concrete"]])
    p.extrude_profile(prof, BARRIER_LEN)
    me = bpy.data.meshes.new("Barrier")
    p.bm.to_mesh(me)
    p.bm.free()
    me.materials.append(mats["concrete"])
    return me


def place_barriers(mats, root):
    """웹 배치 그대로: 동쪽 라인 18개(Y 방향) + 남쪽 라인 11개(X 방전, 90° 회전)."""
    me = build_barrier_template(mats)

    def spawn(name, loc, rot_z):
        obj = bpy.data.objects.new(name, me)
        obj.location = loc
        obj.rotation_euler = (0, 0, rot_z)
        COLL.objects.link(obj)
        obj.parent = root
        return obj

    n = 0
    y = BARRIER_EAST_Y0
    while y <= BARRIER_EAST_Y1 + 1e-6:          # 웹: for(z=-14; z<=14; z+=1.6)
        spawn(f"Barrier_E_{n:02d}", (BARRIER_EAST_X, y, 0), 0.0)
        n += 1
        y += BARRIER_GAP
    n = 0
    x = BARRIER_SOUTH_X0
    while x <= BARRIER_SOUTH_X1 + 1e-6:         # 웹: for(x=-8; x<=8; x+=1.6)
        spawn(f"Barrier_S_{n:02d}", (x, BARRIER_SOUTH_Y, 0), pi / 2)
        n += 1
        x += BARRIER_GAP


# --------------------------------------------------------------------------
# 프리뷰 (GUI 로 열 때만) / 정리
# --------------------------------------------------------------------------
COLL_NAME = "Yard"
PREVIEW_NAME = "Preview"

MAT_PREFIXES = ("ZoneBorder_", "ZoneLabel_", "ZoneFill_", "Barrier_",
                "Preview_", "Zone")
OBJ_NAMES = ("YardRoot", "Zones", "Barriers", "BarrierTemplate")
OBJ_PREFIXES = ("Zone", "Barrier_", "Preview_")


def cleanup_previous():
    """같은 스크립트를 다시 실행했을 때 이전 결과만 지운다 (사용자 데이터는 건드리지 않음)."""
    for name in (COLL_NAME, PREVIEW_NAME):
        coll = bpy.data.collections.get(name)
        if coll:
            for obj in list(coll.objects):
                bpy.data.objects.remove(obj, do_unlink=True)
            bpy.data.collections.remove(coll)
    for obj in list(bpy.data.objects):
        if obj.name.startswith(OBJ_PREFIXES) or obj.name in OBJ_NAMES:
            bpy.data.objects.remove(obj, do_unlink=True)
    for mat in list(bpy.data.materials):
        if mat.name.startswith(MAT_PREFIXES):
            bpy.data.materials.remove(mat)
    for mesh in list(bpy.data.meshes):              # 고아 메시 정리
        if mesh.users == 0:
            bpy.data.meshes.remove(mesh)


def add_preview():
    col = bpy.data.collections.new(PREVIEW_NAME)
    bpy.context.scene.collection.children.link(col)

    ground = bpy.data.meshes.new("Preview_Ground")
    bm = bmesh.new()
    bmesh.ops.create_grid(bm, x_segments=1, y_segments=1, size=40)
    bm.to_mesh(ground)
    bm.free()
    g = bpy.data.objects.new("Preview_Ground", ground)
    g.rotation_euler = (pi / 2, 0, 0)
    g.location = (1, -5, -0.001)
    col.objects.link(g)
    ground.materials.append(make_material("Preview_Ground", "6E7276", rough=.95, metal=.02))

    sun = bpy.data.lights.new("Preview_Sun", "SUN")
    sun.energy = 4.0
    sun.angle = .02
    sl = bpy.data.objects.new("Preview_Sun", sun)
    sl.rotation_euler = Vector((26, 5, -11)).to_track_quat("-Z", "Y").to_euler()
    col.objects.link(sl)

    cam_data = bpy.data.cameras.new("Preview_Camera")
    cam_data.lens = 35
    c = bpy.data.objects.new("Preview_Camera", cam_data)
    c.location = (1.0, 12.5, 11.0)
    c.rotation_euler = (Vector((0, -6.5, 0.05)) - c.location).to_track_quat("-Z", "Y").to_euler()
    col.objects.link(c)
    bpy.context.scene.camera = c

    world = bpy.context.scene.world or bpy.data.worlds.new("Preview_World")
    bpy.context.scene.world = world
    world.use_nodes = True
    bg = world.node_tree.nodes.get("Background")
    if bg:
        bg.inputs[0].default_value = _hex("B9BDC2")
        bg.inputs[1].default_value = .45

    try:
        for area in bpy.context.screen.areas:
            if area.type == "VIEW_3D":
                area.spaces.active.shading.type = "MATERIAL"
                area.spaces.active.region_3d.view_perspective = "PERSP"
    except Exception:
        pass


def parse_args():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    out = {"export": None}
    i = 0
    while i < len(argv):
        if argv[i] == "--export" and i + 1 < len(argv):
            out["export"] = argv[i + 1]
            i += 2
        else:
            i += 1
    return out


def fix_usd_opacity(path):
    """내보낸 USD 의 반투명 재질을 바로잡는 후처리.

    Blender 5.2 USD 익스포터는 Principled 'Alpha' 에 넣은 '값'은 UsdPreviewSurface
    opacity 로 내보내지 않고 1 로 쓴다(이미지 텍스처 알파 '연결'만 내보냄).
    Blender 안에 pxr 이 들어있으므로 내보낸 파일을 다시 열어 ZoneFill_* 재질의
    opacity 를 0.2 로 직접 기록한다. 라벨(0.9)은 PNG 알파(230/255)에 미리 굽는
    방식으로 해결했다.
    """
    try:
        from pxr import Usd, UsdShade
    except ImportError:
        P("[warn] pxr 없음 - ZoneFill opacity 후처리를 건너뜀(USD 에서 opacity=1)")
        return
    stage = Usd.Stage.Open(path)
    if not stage:
        P(f"[warn] USD 다시 열기 실패: {path}")
        return
    fixed = 0
    for prim in stage.Traverse():
        if prim.IsA(UsdShade.Material) and str(prim.GetName()).startswith("ZoneFill_"):
            shader = UsdShade.Shader.Get(stage, prim.GetPath().AppendChild("Principled_BSDF"))
            if shader:
                shader.GetInput("opacity").Set(ZONE_FILL_OPACITY)
                fixed += 1
    stage.GetRootLayer().Save()
    P(f"USD post-process: ZoneFill opacity={ZONE_FILL_OPACITY} x {fixed}")


def main():
    global COLL
    args = parse_args()

    scene = bpy.context.scene
    scene.unit_settings.system = "METRIC"
    scene.unit_settings.scale_length = 1.0

    cleanup_previous()
    COLL = bpy.data.collections.new(COLL_NAME)
    scene.collection.children.link(COLL)

    mats = build_materials()

    # ── 계층: YardRoot ─ Zones(구역 4곳) / Barriers(외곽 배리어)
    root = bpy.data.objects.new("YardRoot", None)
    root.empty_display_size = 1.0
    COLL.objects.link(root)

    zones_root = bpy.data.objects.new("Zones", None)
    zones_root.parent = root
    COLL.objects.link(zones_root)

    P("[1/3] Zones: 4 grade areas with borders / labels / fills ...")
    for key, name_ko, hexcol, x0, x1, y0, y1 in ZONES:
        build_zone(mats, zones_root, key, name_ko, hexcol, x0, x1, y0, y1)

    P("[2/3] Barriers: east line (18) + south line (11) ...")
    barriers_root = bpy.data.objects.new("Barriers", None)
    barriers_root.parent = root
    COLL.objects.link(barriers_root)
    place_barriers(mats, barriers_root)

    P("[3/3] Done.")
    n_obj = sum(1 for o in COLL.objects)
    verts = sum(len(o.data.vertices) for o in COLL.objects if o.data)
    P(f"Done: {n_obj} objects, {verts:,} verts")
    P("Yard part 1: zones x −5.9..5.9 / y −10.5..−2.3, barriers at x=9.2 & y=−15.2")

    if args["export"]:
        out = args["export"]
        bpy.ops.object.select_all(action="DESELECT")
        for obj in COLL.objects:
            obj.select_set(True)
        bpy.ops.wm.usd_export(filepath=out, selected_objects_only=True)
        fix_usd_opacity(out)
        P(f"USD exported: {out}")
    else:
        add_preview()


if __name__ == "__main__":
    main()
