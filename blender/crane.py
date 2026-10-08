"""천장 크레인(원격 하차용) 모델링 스크립트 (Blender bpy) - 웹 판본 그대로

crane_teleop_demo/drawing/원격하차크레인시뮬레이터.html 의 Three.js 씬에 나오는
Overhead Crane 을 Blender 로 되살린다. 웹은 크레인 브리지를 Poly Haven 의
overhead_crane GLB(내장 base64)로 싣고 레일을 Z 방향 30 m 로 늘린 뒤, 레일을
제외한 전 부품을 setRGB(1.75,1.18,.16) 옐로우로 재도색한다. 자석 블록·훅
블록·와이어는 씬 코드로 절차 모델링된다. 여기서는 GLB 를 정점 분석해 실제
형상(아래 '웹 판본 구조')을 bmesh 원시형으로 그대로 재현한다(단위: 미터).

좌표계 (three.js → Blender 변환: (x,y,z) → (x, z, y), 트럭 스크립트와 동일)
    X = 주거더 방향 (레일 게이지 12.5 m, 트롤리가 X 로 다닌다, ±4.6)
    Y = 주행 방향 (브리지 전체가 레일을 따라 다닌다, ±11.5)
    Z = 높이 (웹 Q.Hc = 11.5 가 크레인 기준면, 레일 상면 z = 11.46)

웹 판본 구조 (GLB 정점 분석 + 씬 코드에서 추출한 값)
    Fixed  : 주행 레일 ×2 - I빔 (높이 0.73, 상면 z=11.46, 길이 30 m, x=±6.25)
             기둥 ×10 - I형 (플랜지 2장+웨브, 높이 10.73, x=±6.05[웹 k],
             z = -13.5..13.5 를 6.75 간격 5개씩, 함수 Ae() 그대로)
             안전가드 빔 ×2 - 레일 하단보다 1.6 m 아래(z 9.13) 전 길이 횡빔
    Portal : 엔드트럭 ×2 - 1.1×2.7×3.0 m 대형 박스가 레일을 감싸고, 보 위로
             솟는다(상단 z 12.61). 측면 휠 디스크 + 끝단 완충 패드
             주거더 - 상부 단일 보 10.4 m × 0.56 폭 × 0.19 높이 (z 12.30) +
             양끝 거셋. (웹 판본에는 걷는길·트러스가 없다)
    Trolley: 권상기 - GLB 위너치 상부(z>10.28) 복제. 몸통 박스 4.2×1.15×1.1 +
             상판 + 드럼 2(X 축, r .32, 와이어 홈 2줄씩) + 모터 2 + 접속함 2
    Hook   : 웹 rt()/Je() 공식 그대로 - 와이어 앵커 Ve=10.34 기준으로
             훅 블록 z = Ve-1.8, 자석 블록 z = Ve-3.22
             와이어 4(r .017, ±.07/±.19 배치) / 훅 블록(옐로우 측판+크로스
             실린더+하부 박스+스트랩+굽은 후크[1.45π 아크]) / 체인 3(r .025) /
             자석 블록(원통 r .78 + 하부 캡 + 경고 스트라이프 밴드 + 발광 링 +
             러그 10개 + 아이 + AIMOS 카메라)

링크 구성 (웹의 운동 그룹 미러링 - 씬 재구성 시 물리 부착용)
    CraneRoot ─ Fixed / Portal(Y 주행) / Trolley(X 횡행) / Hook(Z 권상)
    작성 자세는 웹 초기값: Portal y=0, Trolley x=0, Hook z=7.18(자석 중심)

재질 (웹 코드의 ct()/setRGB() 값)
    크레인 옐로우 - GLB 재도색색 setRGB(1.75,1.18,.16)의 표시 등가 #EFBB24
    기둥 = metal_plate 텍스처 틴트 #9AA0A6(웹 O), 가드 = 틴트 #E6B422(웹 ge),
    자석 블록 #3A3D42/#6B7078(웹 ve/ne), 와이어 #2B2B2B(ke), 체인 #55595F(q),
    훅 블록 측판 #E3B21F(Pe), AIMOS 카메라 #D9A21B,
    자석 링 = 바탕 #1A1830(L) / 발광 #7F77DD(웹: 자석 ON 색, strength 1).
    경고 스트라이프는 웹의 512x32 캔버스(#1b1b1b 바탕 + #e3b21f 사선)를 절차
    생성한 이미지로 재현. 텍스처 파일은 blender/textures/ 에 있어야 하고
    없으면 단색 폴백. 솔리드 뷰포트에선 텍스처가 안 보이므로 Z 키로
    '머티리얼 프리뷰' 이상으로 볼 것(스크립트가 자동으로 바꿔준다).

이름 규칙
    USD 변환 시 한글이 깨지므로 오브젝트·재질 이름은 ASCII 영어만 쓴다
    (CraneRoot, RailBeam0, EndTruck1, HoistDrums, MagnetRing, ...).
    주석과 이 문서 문자열만 한글로 유지한다.

사용법
    GUI      : Blender 의 Scripting 탭에서 이 파일을 열고 Run Script
               (프리뷰용 조명·카메라·바닥이 자동으로 붙는다)
    터미널   : blender --background --python blender/crane.py
    USD 내보내기 :
        blender --background --python blender/crane.py -- \
            --export ../assets/crane_blender.usd
"""

import sys


def P(*a):
    """진행 상황 출력(헤드리스 실행에서도 즉시 보이도록 flush)."""
    print(*a, flush=True)


import bpy
import bmesh
import os
from math import pi, atan2, cos, sin
from mathutils import Matrix, Vector, Euler

# --------------------------------------------------------------------------
# 파라미터 - 웹 씬(원격하차크레인시뮬레이터.html)에서 추출한 값 그대로
# --------------------------------------------------------------------------
HC = 11.5                    # 웹 Q.Hc - 크레인 기준 높이(모델 원점)
RAIL_LEN = 30.0              # 웹 W - 레일 길이(스트레치 후)
RAIL_X = 6.25                # 레일 x 위치 (GLB: ±6.25, 게이지 12.5 m)
RAIL_TOP_Z = 11.46           # 레일 상면 (GLB: z 10.73..11.46)
RAIL_BOT_Z = 10.73           # 레일 하면 (= 웹 X = 기둥 높이 Se)
COL_X = 6.05                 # 웹 k = U.max.x-.2 - 기둥 x 위치
COLUMN_ZS = (-13.5, -6.75, 0.0, 6.75, 13.5)   # 웹: -13.5..13.6 step 6.75
GUARD_Z = RAIL_BOT_Z - 1.6   # 웹: Se-1.6 = 9.13 - 가드 빔 높이

TRAVEL_LIM = 11.5            # 웹: Q.bz 클램프 ±11.5 (주행)
TROLLEY_LIM = 4.6            # 웹: Q.tx 클램프 ±4.6 (횡행)

# 권상 장치 - 웹 rt()/Je() 공식의 오프셋 그대로 (앵커 Ve 기준)
VE = 10.40                   # 와이어 앵커 z (웹 Ve, 트롤리 드럼 하단)
ZE_Z = VE - 1.8              # 훅 블록 중심 (웹 b = Q.my+.445+me-Fe)
XE_Z = VE - 3.22             # 자석 블록 중심 (웹 xe = Q.my+.445)
WIRE_R = 0.017               # 웹 와이어 반지름
WIRE_OFF = ((-.07, -.19), (.07, -.19), (-.07, .19), (.07, .19))  # 웹 nt
CHAIN_R = 0.025              # 웹 체인 반지름
TE = 0.78                    # 웹 te - 자석 블록 반지름

def _tex_dir():
    """textures/ 폴더 찾기. GUI 에서 열 때 __file__ 이 스크립트 이름만
    주어지는 경우가 있어 후보를 여러 개 확인한다(스크립트 폴더 →
    현재 작업 폴더/blender/textures → 현재 작업 폴더/textures)."""
    here = os.path.dirname(os.path.abspath(__file__)) if "__file__" in globals() else os.getcwd()
    cands = [os.path.join(here, "textures"),
             os.path.join(os.getcwd(), "blender", "textures"),
             os.path.join(os.getcwd(), "textures")]
    for d in cands:
        if os.path.isdir(d):
            return d
    return cands[0]


TEX_DIR = _tex_dir()
STRIPE_IMG = "stripe_canvas_512x32.png"   # 웹 캔버스를 절차 재현한 스트라이프 이미지

COLL = None            # 만들어진 오브젝트가 들어가는 콜렉션 (main 에서 설정)


# --------------------------------------------------------------------------
# 재질 헬퍼 (truck_load.py 와 같은 구성 - 텍스처 있으면 PBR, 없으면 단색)
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


def _load_img(fname, non_color=False):
    """textures/ 폴더의 이미지를 로드(재사용). 없으면 None."""
    path = os.path.join(TEX_DIR, fname)
    if not os.path.isfile(path):
        return None
    img = bpy.data.images.get(fname)
    if img is None:
        img = bpy.data.images.load(path)
        img.name = fname
    img.colorspace_settings.name = "Non-Color" if non_color else "sRGB"
    return img


def _mix_multiply(nodes):
    try:
        n = nodes.new("ShaderNodeMix")
        n.data_type = "RGBA"
        n.blend_type = "MULTIPLY"
        return n
    except Exception:
        n = nodes.new("ShaderNodeMixRGB")
        n.blend_type = "MULTIPLY"
        return n


def make_material(name, hexcol, rough=0.5, metal=0.5, emit=None, emit_strength=0.0,
                  maps=None, uv_scale=1.0):
    """Principled BSDF 재질. maps="metal_plate" → HTML 웹 버전과 같은 PBR 텍스처
    (diff × 틴트, arm.G × roughness). 파일이 없으면 그 맵은 건너뛴다."""
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

    if maps:
        img_d = _load_img(f"{maps}_diff.jpg")
        img_n = _load_img(f"{maps}_nor.jpg", non_color=True)
        img_r = _load_img(f"{maps}_arm.jpg", non_color=True)
        if [i for i in (img_d, img_n, img_r) if i]:
            coord = nodes.new("ShaderNodeTexCoord")
            coord.location = (-1350, 100)
            mapping = nodes.new("ShaderNodeMapping")
            mapping.location = (-1150, 100)
            _set(mapping, ["Scale"], (1.0 / uv_scale,) * 3)
            links.new(coord.outputs["UV"], mapping.inputs["Vector"])
            vector = mapping.outputs["Vector"]
            if img_d:                                    # Base Color = diff × 틴트
                td = nodes.new("ShaderNodeTexImage")
                td.location = (-930, 420)
                td.image = img_d
                links.new(vector, td.inputs["Vector"])
                mix = _mix_multiply(nodes)
                mix.location = (-640, 420)
                _set(mix, ["Factor"], 1.0)
                mix.inputs["A"].default_value = _hex(hexcol)
                links.new(td.outputs["Color"], mix.inputs["B"])
                links.new(mix.outputs["Result"], bsdf.inputs["Base Color"])
            if img_n:                                    # Normal
                tn = nodes.new("ShaderNodeTexImage")
                tn.location = (-930, 120)
                tn.image = img_n
                links.new(vector, tn.inputs["Vector"])
                nmap = nodes.new("ShaderNodeNormalMap")
                nmap.location = (-640, 120)
                links.new(tn.outputs["Color"], nmap.inputs["Color"])
                links.new(nmap.outputs["Normal"], bsdf.inputs["Normal"])
            if img_r:                                    # Roughness = arm.G × rough
                tr = nodes.new("ShaderNodeTexImage")
                tr.location = (-930, -180)
                tr.image = img_r
                links.new(vector, tr.inputs["Vector"])
                try:                                      # 5.x: SeparateColor
                    sep = nodes.new("ShaderNodeSeparateColor")
                    sep.mode = "RGB"
                    in_sock, out_sock = "Color", "Green"
                except Exception:
                    sep = nodes.new("ShaderNodeSeparateRGB")
                    in_sock, out_sock = "Image", "G"
                sep.location = (-640, -180)
                links.new(tr.outputs["Color"], sep.inputs[in_sock])
                mul = nodes.new("ShaderNodeMath")
                mul.location = (-400, -180)
                mul.operation = "MULTIPLY"
                _set(mul, ["Value"], rough)
                links.new(sep.outputs[out_sock], mul.inputs[0])
                links.new(mul.outputs["Value"], bsdf.inputs["Roughness"])
    return mat


def make_stripe_material():
    """자석 블록의 경고 스트라이프 밴드용 재질.

    웹은 512x32 캔버스에 #1b1b1b 바탕 + #e3b21f 사선 스트라이프(20px 주기)를
    그려 open-ended 실린더에 repeat.x=2 로 얹었다. 여기서 같은 패턴을 Blender
    이미지로 절차 생성한다(외부 파일 의존 없음). UV 는 밴드를 만들 때 원통
    좌표로 직접 입힌다(Part.cyl_uvs).
    """
    mat = bpy.data.materials.get("Magnet_Stripe")
    if mat is None:
        mat = bpy.data.materials.new("Magnet_Stripe")
    mat.use_nodes = True
    nt = mat.node_tree
    for n in list(nt.nodes):
        nt.nodes.remove(n)
    nodes, links = nt.nodes, nt.links
    out = nodes.new("ShaderNodeOutputMaterial")
    out.location = (140, 300)
    bsdf = nodes.new("ShaderNodeBsdfPrincipled")
    bsdf.location = (-140, 300)
    links.new(bsdf.outputs["BSDF"], out.inputs["Surface"])
    _set(bsdf, ["Base Color"], _hex("FFFFFF"))
    _set(bsdf, ["Metallic"], 0.2)
    _set(bsdf, ["Roughness"], 0.6)

    img = bpy.data.images.get(STRIPE_IMG)
    if img is None:
        img = bpy.data.images.new(STRIPE_IMG, width=512, height=32)
        # 주의: 색상공간 변경이 픽셀을 초기화하므로(Blender 5.2) 반드시 할당 전에.
        img.colorspace_settings.name = "sRGB"
        img.name = STRIPE_IMG
        yellow = (0.890, 0.698, 0.122, 1.0)     # #e3b21f
        dark = (0.106, 0.106, 0.106, 1.0)       # #1b1b1b
        px = []
        for yy in range(32):
            for xx in range(512):
                # 사선: 위로 갈수록 12px 만큼 옮겨진 평행사변형 스트라이프
                u = (xx + (31 - yy) * 0.375) % 20.0
                px.extend(yellow if u < 10.0 else dark)
        img.pixels = px
    img.use_fake_user = True
    tex = nodes.new("ShaderNodeTexImage")
    tex.location = (-640, 300)
    tex.image = img
    links.new(tex.outputs["Color"], bsdf.inputs["Base Color"])
    return mat


def build_materials():
    m = {}
    # 크레인 옐로우 - 웹이 GLB 전체에 칠한 setRGB(1.75,1.18,.16)(linear)의
    # 표시 등가. 도장 강판이라 금속성은 낮게.
    m["yellow"] = make_material("Crane_Yellow", "EFBB24", rough=.45, metal=.15)
    m["rail"] = make_material("Rail_Steel", "6E747B", rough=.5, metal=.6)
    m["column"] = make_material("Column_Steel", "9AA0A6", rough=.6, metal=.6,   # 웹 O
                                maps="metal_plate", uv_scale=1.0)
    m["guard"] = make_material("Guard_Yellow", "E6B422", rough=.5, metal=.35,   # 웹 ge
                               maps="metal_plate", uv_scale=1.0)
    m["steel"] = make_material("Crane_Steel", "3A3D42", rough=.45, metal=.85)   # 웹 ve
    m["gray"] = make_material("Machinery_Gray", "6B7078", rough=.4, metal=.9)   # 웹 ne
    m["wire"] = make_material("Wire_Dark", "2B2B2B", rough=.45, metal=.8)       # 웹 ke
    m["chain"] = make_material("Chain_Gray", "55595F", rough=.35, metal=.95)    # 웹 q
    m["hookplate"] = make_material("HookPlate_Yellow", "E3B21F", rough=.45, metal=.35)  # 웹 Pe
    m["camera"] = make_material("Aimos_Camera", "D9A21B", rough=.5, metal=.3)   # 웹 w
    # 자석 링 - 꺼짐 바탕 #1A1830(웹 L), 켜짐 발광 #7F77DD(웹 yo() 의 ON 색).
    # strength 를 1.0 으로 두면 USD emissiveColor 가 정확히 #7F77DD 가 된다
    # (2 이상은 R/G 가 포화돼 흰색으로 보임).
    m["ring"] = make_material("MagnetRing", "1A1830", rough=.4, metal=.1,
                              emit="7F77DD", emit_strength=1.0)
    m["stripe"] = make_stripe_material()
    return m


# --------------------------------------------------------------------------
# 지오메트리 헬퍼 (bmesh 만 사용 - 오퍼레이터/컨텍스트 의존 없음)
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
        """size=(x,y,z) 중심 loc, 모서리 둥글기 bevel (웹 je/dn 헬퍼)."""
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

    def cone(self, r, h, loc, mat_idx=0, seg=24, rot=None, cap=True):
        """Z 축 원뿔대/원기둥을 loc 에 추가 (rot 로 방향 조정)."""
        bm = self.bm
        pre_v, pre_f = set(bm.verts), set(bm.faces)
        bmesh.ops.create_cone(bm, cap_ends=cap, cap_tris=False, segments=seg,
                              radius1=r, radius2=r, depth=h)
        self._place(pre_v, loc, rot)
        self._paint(pre_f, mat_idx)

    def rod(self, a, b, r, mat_idx=0, seg=10, cap=True):
        """두 점을 잇는 봉(원기둥) - 와이어·체인용 (웹 Je() 와 동일한 배치식)."""
        bm = self.bm
        pre_v, pre_f = set(bm.verts), set(bm.faces)
        a, b = Vector(a), Vector(b)
        d = b - a
        bmesh.ops.create_cone(bm, cap_ends=cap, cap_tris=False, segments=seg,
                              radius1=r, radius2=r, depth=d.length)
        m = Matrix.Translation((a + b) / 2) @ d.to_track_quat("Z", "Y").to_matrix().to_4x4()
        bmesh.ops.transform(bm, matrix=m, verts=[v for v in bm.verts if v not in pre_v])
        self._paint(pre_f, mat_idx)

    def torus(self, R, r, loc, mat_idx=0, seg=48, rseg=8, rot=None, arc=2 * pi,
              arc_roll=0.0):
        """Z 축 도넛. arc<2π 면 열린 부분 원호(굽은 후크), arc_roll 은 원호의
        시작 각도(웹 fs(.12,.035,12,28,π*1.45) + rotation.z 재현용)."""
        bm = self.bm
        pre_v, pre_f = set(bm.verts), set(bm.faces)
        closed = arc >= 2 * pi - 1e-6
        n = seg if closed else max(8, int(seg * arc / (2 * pi)))
        grid = []
        for i in range(n + (0 if closed else 1)):
            a = arc_roll + arc * i / n
            ca, sa = cos(a), sin(a)
            ring = []
            for j in range(rseg):
                b = j / rseg * 2 * pi
                rr = R + r * cos(b)
                ring.append(bm.verts.new(Vector((rr * ca, rr * sa, r * sin(b)))))
            grid.append(ring)
        steps = len(grid) if closed else len(grid) - 1
        for i in range(steps):
            i2 = (i + 1) % len(grid) if closed else i + 1
            for j in range(rseg):
                j2 = (j + 1) % rseg
                bm.faces.new((grid[i][j], grid[i2][j], grid[i2][j2], grid[i][j2]))
        self._place(pre_v, loc, rot)
        self._paint(pre_f, mat_idx)

    def _place(self, pre_v, loc, rot):
        m = Matrix.Translation(Vector(loc))
        if rot is not None:
            m = m @ (rot.to_matrix().to_4x4() if isinstance(rot, Euler) else rot)
        bmesh.ops.transform(self.bm, matrix=m, verts=[v for v in self.bm.verts if v not in pre_v])

    def _paint(self, pre_f, mat_idx):
        for f in self.bm.faces:
            if f not in pre_f:
                f.material_index = mat_idx

    def finish(self, parent=None, uv_scale=0.0):
        """uv_scale>0 이면 박스(3평면) UV 생성 - 텍스처용(웹 jl() 과 동일 방식)."""
        me = bpy.data.meshes.new(self.name)
        if uv_scale > 0:
            self.box_uvs(self.bm, uv_scale)
        self.bm.to_mesh(me)
        self.bm.free()
        for mat in self.materials:
            me.materials.append(mat)
        obj = bpy.data.objects.new(self.name, me)
        COLL.objects.link(obj)
        if parent:
            obj.parent = parent
        return obj

    @staticmethod
    def box_uvs(bm, scale=1.0):
        """면 법선의 주축 평면으로 투영하는 박스 UV (월드 좌표/scale)."""
        uv = bm.loops.layers.uv.new("UVMap")
        for f in bm.faces:
            n = f.normal
            ax, ay, az = abs(n.x), abs(n.y), abs(n.z)
            for l in f.loops:
                c = l.vert.co
                if ax >= ay and ax >= az:
                    u, v = c.z, c.y
                elif ay >= az:
                    u, v = c.x, c.z
                else:
                    u, v = c.x, c.y
                l[uv].uv = (u / scale, v / scale)

    @staticmethod
    def cyl_uvs(bm, z0, z1, u_tiles=2.0):
        """원통 좌표 UV (u=둘레각, v=높이) - 스트라이프 밴드용(웹 repeat.x=2)."""
        uv = bm.loops.layers.uv.new("UVMap")
        for f in bm.faces:
            for l in f.loops:
                c = l.vert.co
                u = (atan2(c.y, c.x) / (2 * pi) + 0.5) * u_tiles
                v = (c.z - z0) / max(z1 - z0, 1e-6)
                l[uv].uv = (u, v)


# --------------------------------------------------------------------------
# 고정 구조물 - 웹의 st.attach(S)(레일 분리) + Ae(기둥) + ge(가드 빔)
# --------------------------------------------------------------------------
def build_rails(mats, fixed):
    """주행 레일: GLB 레일(높이 0.73, x=±6.25)을 30 m 로 늘린 것.
    I빔 단면 - 하플랜지(0.5폭) / 웨브 / 상플랜지(0.3폭), 상면 z=11.46."""
    for i, x in enumerate((-RAIL_X, RAIL_X)):
        p = Part(f"RailBeam{i}", [mats["rail"]])
        p.box((.50, RAIL_LEN, .12), (x, 0, RAIL_BOT_Z + .06))          # 하 플랜지
        p.box((.10, RAIL_LEN, .51), (x, 0, RAIL_BOT_Z + .12 + .255))   # 웨브
        p.box((.30, RAIL_LEN, .10), (x, 0, RAIL_TOP_Z - .05))          # 상 플랜지
        p.finish(fixed)


def build_columns(mats, fixed):
    """기둥: 웹 Ae() 그대로 - 플랜지 2장(0.36×0.035) + 웨브 + 베이스/캡 플레이트.
    x=±k(=6.05), z -13.5..13.5 를 6.75 간격 5개, 높이 = 레일 하면(10.73)."""
    h = RAIL_BOT_Z
    for side, x in enumerate((-COL_X, COL_X)):
        for j, z in enumerate(COLUMN_ZS):
            p = Part(f"Column{side}_{j}", [mats["column"]])
            p.box((.36, .035, h), (x, z - .17, h / 2))                  # 플랜지 (-Y 쪽)
            p.box((.36, .035, h), (x, z + .17, h / 2))                  # 플랜지 (+Y 쪽)
            p.box((.05, .34, h), (x, z, h / 2))                         # 웨브
            p.box((.60, .60, .04), (x, z, .02))                         # 베이스 플레이트
            p.box((.50, .50, .05), (x, z, h + .025))                    # 캡 플레이트
            p.box((.50, .30, .06), (x, z, h - .40))                     # 중간 보강(웹과 동일)
            p.finish(fixed, uv_scale=1.0)


def build_guards(mats, fixed):
    """안전가드 빔: 웹 ge - 레일 바깥 x=±(k+.05), 레일 하단보다 1.6 m 아래에
    전 길이(30 m)로 달리는 노란 횡빔."""
    for i, x in enumerate((-(COL_X + .05), COL_X + .05)):
        p = Part(f"GuardRail{i}", [mats["guard"]])
        p.box((.12, RAIL_LEN, .22), (x, 0, GUARD_Z))
        p.finish(fixed, uv_scale=1.0)


# --------------------------------------------------------------------------
# 포탈(주행 브리지) - 웹 N(overhead_crane 노드)의 형상 복제
# --------------------------------------------------------------------------
def build_end_truck(mats, portal, side):
    """엔드트럭: GLB 정점 분석값 그대로 - 1.1(X)×2.7(Y)×3.0(Z) 대형 박스가
    레일(z 10.73..11.46)을 감싸고 보 위로 0.22 솟는다(상단 12.61).
    바깥면 휠 디스크 2개 + 진행 방향 끝단 완충 패드."""
    x = (-5.55, 5.55)[side]
    cz = (9.58 + 12.61) / 2
    p = Part(f"EndTruck{side}", [mats["yellow"]])
    p.box((1.10, 2.70, 3.03), (x, -0.56, cz), bevel=.05)               # 본체
    rx = Euler((0, pi / 2, 0))                                          # Z축 → X축(휠 축)
    for y in (-0.85, 0.85):                                             # 측면 휠 디스크
        wx = x + (0.60 if side else -0.60)
        p.cone(.34, .08, (wx, y - 0.56, 11.10), seg=24, rot=rx)
    for s in (-1, 1):                                                   # 완충 패드
        p.box((.25, .08, .50), (x, -0.56 + s * 1.39, 11.0), bevel=.02)
    return p.finish(portal)


def build_beam(mats, portal):
    """주거더: GLB 중앙부 그대로 - 상부 단일 보(10.4×0.56×0.19, z 12.30)와
    양끝 거셋(브리지 x ±4..5 의 z 10.1..12.2 브래킷). 웹 판본에는 걷는길이
    없다(정점수가 박스 보 하나 분량뿐). 보 위(y=-0.8)에는 얇은 아치 라인이
    한 줄 걸쳐 있다(중앙에서 0.26 m 솟음 - GLB 정점 z 12.46..12.72)."""
    p = Part("TopBeam", [mats["yellow"]])
    p.box((10.4, .56, .19), (0, 0, 12.30), bevel=.02)                   # 단일 보
    for s in (-1, 1):
        p.box((1.0, .50, 2.10), (s * 4.5, 0, 11.15), bevel=.03)         # 양끝 거셋
    pts = []
    for i in range(17):                                                 # 아치 라인 16분할
        t = i / 16 * 2 - 1
        pts.append((t * 6.1, -0.8, 12.46 + .26 * (1 - t * t)))
    for a, b in zip(pts, pts[1:]):
        p.rod(a, b, .03, seg=8)
    return p.finish(portal)


# --------------------------------------------------------------------------
# 트롤리(권상기) - GLB 위너치 상부(z>10.28, 4.6×1.26×1.85) 복제
# --------------------------------------------------------------------------
def build_trolley(mats, trolley):
    """웹은 위너치를 z=re(10.28) 기준으로 잘라 상부만 남긴다(하부는 버림).
    몸통 박스 + 상판(보 하면에 붙음) + 드럼 2(X축, 와이어 홈 2줄씩 -
    웹의 4와이어가 ±.19 간격 두 쌍인 이유) + 모터 2 + 접속함 2."""
    p = Part("WinchBody", [mats["yellow"]])
    p.box((4.20, 1.15, 1.10), (0, 0, 11.55), bevel=.04)                 # 몸통
    p.box((4.40, 1.25, .12), (0, 0, 12.16), bevel=.02)                  # 상판(보 하면 접합)
    p.finish(trolley)

    d = Part("HoistDrums", [mats["yellow"]])
    rx = Euler((0, pi / 2, 0))                                          # Z축 → X축(드럼 축)
    rz = Euler((pi / 2, 0, 0))                                          # 토러스 축을 X축으로
    for y in (-0.185, 0.185):                                           # 드럼 2
        d.cone(.32, 1.40, (0, y, 10.72), seg=24, rot=rx)
        for xg in (-0.07, 0.07):                                        # 와이어 홈 2줄
            d.torus(.325, .012, (xg, y, 10.72), seg=28, rseg=6, rot=rz)
    d.finish(trolley)

    m = Part("HoistMotors", [mats["yellow"]])
    for s in (-1, 1):
        m.cone(.24, .80, (s * 1.90, 0, 10.95), seg=18, rot=rx)          # 모터(축 X)
        m.box((.55, .35, .30), (s * 1.90, 0, 11.62), bevel=.02)         # 접속함
    m.finish(trolley)


# --------------------------------------------------------------------------
# 후크 장치 - 웹 xe(자석 블록) + ze(훅 블록) + fe(체인) + B(와이어 4)
# --------------------------------------------------------------------------
def build_wires(mats, hook):
    """와이어 4(웹 B): 트롤리 앵커(±.07, Ve, ±.19) → 훅 블록 상부(+.16).
    Hook 로컬(z 원점 = 자석 블록 중심 XE_Z) 좌표로 작성."""
    p = Part("HoistWires", [mats["wire"]])
    top_z = VE - XE_Z                       # = 3.22
    bot_z = ZE_Z + 0.16 - XE_Z              # = 1.58 (웹: b+.16)
    for ox, oy in WIRE_OFF:
        p.rod((ox, oy, top_z), (ox, oy, bot_z), WIRE_R, seg=10)
    return p.finish(hook)


def build_hook_block(mats, hook):
    """훅 블록(웹 ze): 옐로우 측판 2장 + 크로스 실린더 + 하부 박스 + 스트랩 +
    굽은 후크(1.45π 아크 토러스). Hook 로컬 z = ZE_Z-XE_Z = 1.42 에 중심."""
    z0 = ZE_Z - XE_Z
    p = Part("HookBlock", [mats["hookplate"], mats["steel"]])
    for s in (-1, 1):
        p.box((.05, .46, .55), (s * .15, 0, z0 + .08), mat_idx=0, bevel=.02)   # 측판(웹 je)
    p.cone(.19, .22, (0, 0, z0 + .16), mat_idx=1, seg=24,
           rot=Euler((0, pi / 2, 0)))                                          # 크로스 실린더(웹 Pn)
    p.box((.36, .30, .12), (0, 0, z0 - .22), mat_idx=1, bevel=.03)             # 하부 박스
    p.cone(.04, .16, (0, 0, z0 - .34), mat_idx=1, seg=10)                      # 스트랩
    # 굽은 후크: 세로 평면 원호(웹 fs(.12,.035,12,28,π*1.45) + rot.z=π*.78)
    p.torus(.12, .035, (0, 0, z0 - .52), mat_idx=1, seg=28, rseg=10,
            rot=Euler((pi / 2, 0, 0)), arc=1.45 * pi, arc_roll=pi * .78)
    return p.finish(hook)


def build_chains(mats, hook):
    """체인 3(웹 fe): 훅 블록 바닥(로컬 z 0.84)에서 자석 상부 림(반지름 .45,
    z +.3)으로 방사형으로 내려간다(웹: 위상 +0.5 rad)."""
    p = Part("Chains", [mats["chain"]])
    z0 = ZE_Z - XE_Z
    for i in range(3):
        a = i / 3 * 2 * pi + .5
        a2 = a + .35
        p.rod((cos(a) * .16, sin(a) * .16, z0 - .58),
              (cos(a2) * .45, sin(a2) * .45, .30), CHAIN_R, seg=8)
    return p.finish(hook)


def build_magnet(mats, hook):
    """자석 블록(웹 xe) - Hook 로컬 원점(=블록 중심) 기준:
    원통 r.78 h.42@z-.21 / 하부 캡 r.80 h.05@z-.42 / 발광 링(토러스 r.73)@z-.43 /
    상부 아이 실린더 / 러그 10개(r.42) / AIMOS 카메라. 스트라이프 밴드는
    별도 파트(원통 UV 필요)."""
    mg = Part("Magnet", [mats["steel"], mats["gray"], mats["camera"]])
    mg.cone(TE, .42, (0, 0, -.21), mat_idx=0, seg=48)                    # 몸통
    mg.cone(TE + .02, .05, (0, 0, -.42), mat_idx=1, seg=48)              # 하부 캡
    mg.cone(.22, .22, (0, 0, .10), mat_idx=1, seg=24)                    # 상부 아이(웹 Pn .2/.24/.22)
    for i in range(10):                                                  # 러그 10개(웹 볼트)
        a = i / 10 * 2 * pi
        mg.box((.55, .05, .16), (cos(a) * .42, sin(a) * .42, .06),
               mat_idx=1, bevel=.015, rot=Euler((0, 0, -a)))             # 방사형(웹 rot.y=-E)
    mg.box((.36, .24, .20), (.42, .42, .12), mat_idx=2, bevel=.03,
           rot=Euler((0, 0, .8)))                                        # AIMOS 카메라(웹 rot.y=.8)
    mg.finish(hook)

    r = Part("MagnetRing", [mats["ring"]])
    r.torus(TE - .05, .025, (0, 0, -.45), seg=48, rseg=6)                # 발광 링(웹 fs@-.43,
    r.finish(hook)                                                        #  캡 아래 살짝 더 노출)

    # 경고 스트라이프 밴드(웹 캔버스) - 몸통 상단, 원통 UV 로 둘레에 2 타일
    b = Part("MagnetStripe", [mats["stripe"]])
    bm = b.bm
    pre_v, pre_f = set(bm.verts), set(bm.faces)
    bmesh.ops.create_cone(bm, cap_ends=False, cap_tris=False, segments=48,
                          radius1=TE + .006, radius2=TE + .006, depth=.09)
    m = Matrix.Translation(Vector((0, 0, -.12)))
    bmesh.ops.transform(bm, matrix=m, verts=[v for v in bm.verts if v not in pre_v])
    Part.cyl_uvs(bm, z0=-.165, z1=-.075, u_tiles=2.0)
    b.finish(hook)


# --------------------------------------------------------------------------
# 프리뷰 (GUI 로 열 때만) / 정리
# --------------------------------------------------------------------------
COLL_NAME = "GantryCrane"
PREVIEW_NAME = "Preview"

MAT_PREFIXES = ("Crane_", "Rail_", "Column_", "Guard_", "Machinery_", "Wire_",
                "Chain_", "HookPlate_", "Aimos_", "MagnetRing", "Magnet_",
                "Preview_")

OBJ_NAMES = ("CraneRoot", "Fixed", "Portal", "Trolley", "Hook")
OBJ_PREFIXES = ("RailBeam", "Column", "GuardRail", "EndTruck", "TopBeam",
                "WinchBody", "HoistDrums", "HoistMotors", "HoistWires",
                "HookBlock", "Chains", "Magnet", "MagnetRing", "MagnetStripe",
                "Preview_")


def cleanup_previous():
    """같은 스크립트를 다시 실행했을 때 이전 결과만 지운다 (사용자 데이터는 건드리지 않음)."""
    for name in (COLL_NAME, PREVIEW_NAME):
        coll = bpy.data.collections.get(name)
        if coll:
            for obj in list(coll.objects):
                bpy.data.objects.remove(obj, do_unlink=True)
            bpy.data.collections.remove(coll)
    # 콜렉션 밖에 남은 이전 결과(프리뷰 카메라 등)도 제거
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
    col.objects.link(g)
    ground.materials.append(make_material("Preview_Ground", "7A7E83", rough=.9, metal=.05))

    sun = bpy.data.lights.new("Preview_Sun", "SUN")
    sun.energy = 4.0
    sun.angle = .02
    sl = bpy.data.objects.new("Preview_Sun", sun)
    sl.rotation_euler = Vector((26, 5, -11)).to_track_quat("-Z", "Y").to_euler()
    col.objects.link(sl)

    cam_data = bpy.data.cameras.new("Preview_Camera")
    cam_data.lens = 35
    c = bpy.data.objects.new("Preview_Camera", cam_data)
    c.location = (27.0, -21.0, 12.5)
    c.rotation_euler = (Vector((0, 0, 7.0)) - c.location).to_track_quat("-Z", "Y").to_euler()
    col.objects.link(c)
    bpy.context.scene.camera = c

    world = bpy.context.scene.world or bpy.data.worlds.new("Preview_World")
    bpy.context.scene.world = world
    world.use_nodes = True
    bg = world.node_tree.nodes.get("Background")
    if bg:
        bg.inputs[0].default_value = _hex("B9BDC2")
        bg.inputs[1].default_value = .45

    # 뷰포트를 '머티리얼 프리뷰'로 전환 - 솔리드 모드에선 텍스처/범프가 안 보인다
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

    # ── 계층: CraneRoot ─ Fixed / Portal(주행) / Trolley(횡행) / Hook(권상)
    #    웹의 운동 그룹(N/T/ze·xe) 미러링. 작성 자세는 웹 초기값 그대로.
    root = bpy.data.objects.new("CraneRoot", None)
    root.empty_display_size = 1.0
    COLL.objects.link(root)

    fixed = bpy.data.objects.new("Fixed", None)
    fixed.parent = root
    COLL.objects.link(fixed)

    P("[1/5] Fixed: rails 30m / 10 columns / guard beams ...")
    build_rails(mats, fixed)
    build_columns(mats, fixed)
    build_guards(mats, fixed)

    P("[2/5] Portal: 2 end trucks + single top beam ...")
    portal = bpy.data.objects.new("Portal", None)     # 주행 링크(y=0 작성)
    portal.parent = root
    COLL.objects.link(portal)
    build_end_truck(mats, portal, 0)
    build_end_truck(mats, portal, 1)
    build_beam(mats, portal)

    P("[3/5] Trolley: winch body / 2 drums / motors ...")
    trolley = bpy.data.objects.new("Trolley", None)   # 횡행 링크(x=0 작성)
    trolley.parent = root
    COLL.objects.link(trolley)
    build_trolley(mats, trolley)

    P("[4/5] Hook: 4 wires / hook block / 3 chains / magnet block ...")
    hook = bpy.data.objects.new("Hook", None)         # 권상 링크(자석 중심 z 작성)
    hook.location = (0, 0, XE_Z)
    hook.parent = root
    COLL.objects.link(hook)
    build_wires(mats, hook)
    build_hook_block(mats, hook)
    build_chains(mats, hook)
    build_magnet(mats, hook)

    P("[5/5] Done.")
    n_obj = sum(1 for o in COLL.objects)
    verts = sum(len(o.data.vertices) for o in COLL.objects if o.data)
    P(f"Done: {n_obj} objects, {verts:,} verts")
    P(f"Crane envelope: X ±6.25 m (gauge 12.5), Y ±15 m (rails 30 m), "
      f"rail top z={RAIL_TOP_Z}, magnet bottom z={XE_Z - .445:.2f}")

    if args["export"]:
        out = args["export"]
        # 시작 씬의 기본 오브젝트(큐브 등)가 섞이지 않도록 크레인 콜렉션만 선택해 내보낸다
        bpy.ops.object.select_all(action="DESELECT")
        for obj in COLL.objects:
            obj.select_set(True)
        bpy.ops.wm.usd_export(filepath=out, selected_objects_only=True)
        P(f"USD exported: {out}")
    else:
        add_preview()


if __name__ == "__main__":
    main()
