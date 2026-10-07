"""덤프트럭 + 철근·분철 적재 모델링 스크립트 (Blender bpy)

crane_teleop_demo/drawing/원격하차크레인시뮬레이터.html 씬 가운데의 트럭을
Blender 로 그대로 재현한다. 치수·색·부품 배치는 HTML 안의 Three.js 씬 코드에서
추출해 1:1로 옮겼다 (단위: 미터).

좌표계 (트럭 로컬)
    X = 길이방향 (앞 = -X, 운전석이 -X 쪽)
    Y = 폭방향   (좌우 ±)
    Z = 높이     (바닥 z = 0, 바퀴 6개가 지면에 접지)

구성
    운전석(캡·유리·미러) / 그릴·범퍼·헤드라이트 / 샤시(프레임·연료탱크·배기관·스텝·미등)
    적재함(바닥·측벽+리브·전벽·후문·상부레일) / 6륜(타이어·림·허브)
    화물 = 분철 더미(높이맵 몸동 + 불규칙 조각 ~2000개, 14색 팔레트)
          + 철근 다발(2열 × 2층) + 굽은 철근 2개

재질
    웹 버전이 쓰던 PBR 텍스처(metal_plate / rusty_metal_02)를 HTML 에서 추출해
    textures/ 폴더에 저장해 두고 적재함·분철 재질에 실제로 연결한다(diff×틴트,
    노멀맵, arm 팩킹의 G채널=거칠기 - three.js 와 같은 방식). 파일이 없으면 절차적
    노이즈로 폴백. 솔리드(Solid) 뷰포트 모드에선 텍스처가 안 보이므로 Z 키로
    '머티리얼 프리뷰' 이상으로 전환해서 볼 것(스크립트가 자동으로 바꿔준다).

이름 규칙
    USD 변환 시 한글 xform/프림 이름이 깨져 보이므로 오브젝트·메시·재질 이름은 모두
    ASCII 영어를 쓴다(Cab, DumpBed, Wheel_Front_L, Scrap_Chunks, Truck_Paint, ...).
    주석과 이 문서 문자열만 한글로 유지한다.

사용법
    GUI      : Blender 의 Scripting 탭에서 이 파일을 열고 Run Script
               (프리뷰용 조명·카메라·바닥이 자동으로 붙는다)
    터미널   : blender --background --python blender_truck.py
    USD 내보내기 :
        blender --background --python blender_truck.py -- \
            --export ../assets/truck_load_blender.usd [--chunks 2000] [--seed 7]
"""

import sys


def P(*a):
    """진행 상황 출력(헤드리스 실행에서도 즉시 보이도록 flush)."""
    print(*a, flush=True)


import bpy
import bmesh
import os
import random
from math import pi, sin, cos, sqrt
from mathutils import Matrix, Vector, Euler

# --------------------------------------------------------------------------
# 파라미터 (원격하차크레인시뮬레이터.html 씬 코드에서 추출한 값)
# --------------------------------------------------------------------------
BED_LEN, BED_W, BED_FLOOR_H, BED_WALL_H = 5.6, 2.34, 1.45, 1.15   # 적재함 길이/폭/바닥높이/벽높이
BED_X0 = -2.35                                                      # 적재함 시작 X (전벽 위치)
BED_CX = BED_X0 + BED_LEN / 2                                       # 적재함 중심 X = 0.45

CHUNK_COUNT = 2000     # 분철 조각 수 (HTML: 2300개 중 더미 위에 보이는 것들)
SEED = 7               # HTML 의 난수 시드(7)와 같은 의미로 재현용 고정

# 분철 조각 14색 팔레트 (HTML Ys 배열 그대로)
CHUNK_PALETTE = ["D4D7DB", "B6BABF", "9A9FA5", "80858B", "676C72", "50545A",
                 "C9CCD0", "8C9196", "A77452", "7B5A44", "B0B4B9", "5B7690",
                 "9DA1A6", "6F7378"]

rng = random.Random(SEED)
COLL = None            # 만들어진 오브젝트가 들어가는 콜렉션 (main 에서 설정)

# --------------------------------------------------------------------------
# 재질 - 웹 버전에서 쓰던 PBR 텍스처(metal_plate / rusty_metal_02)를 실제로 적용한다.
# 텍스처 파일은 HTML 에서 추출해 textures/ 폭더에 미리 저장해 둔다(아래 참조).
# 파일이 없으면 절차적 노이즈 범프로 폴백하므로 스크립트 단독으로도 동작한다.
# --------------------------------------------------------------------------
TEX_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "textures")


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
    """색 곱셈 노드(4.x 의 ShaderNodeMix, 구버전은 ShaderNodeMixRGB)."""
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
                  clearcoat=0.0, noise_bump=0.0, noise_scale=30.0,
                  maps=None, uv_scale=1.0, map_diff=True, map_nor=True, map_rough=True,
                  rough_map_generated=False):
    """Principled BSDF 재질.

    maps="metal_plate" | "rusty_metal_02"  → HTML 웹 버전과 같은 PBR 텍스처 적용.
      - diff 는 hexcol 색으로 틴트(three.js 의 color × map 방식),
      - arm(AO/Rough/Metal 팩킹)의 G 채널을 roughness 값에 곱해 연결(=roughnessMap 방식),
      - nor 은 Normal Map 노드로 연결. map_diff/map_nor/map_rough 로 조합 선택.
      - rough_map_generated=True 면 UV 없이 Generated 좌표로 샘플링(분철 조각 - HTML 은
        인스턴스 재질에 roughnessMap 만 사용).
    텍스처 파일이 없으면 그 맵은 건너뛰고, 전부 없으면 절차 노이즈 범프로 폴백.
    """
    mat = bpy.data.materials.get(name)
    if mat is None:
        mat = bpy.data.materials.new(name)
    mat.use_nodes = True
    nt = mat.node_tree
    for n in list(nt.nodes):                      # 재실행 시에도 최신 설정으로 재구성
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
    if clearcoat:
        _set(bsdf, ["Clearcoat"], clearcoat)
        _set(bsdf, ["Clearcoat Roughness"], 0.25)
    if emit is not None:
        _set(bsdf, ["Emission Color", "Emission"], _hex(emit))
        _set(bsdf, ["Emission Strength"], emit_strength)

    any_map = False
    if maps:
        img_d = _load_img(f"{maps}_diff.jpg") if map_diff else None
        img_n = _load_img(f"{maps}_nor.jpg", non_color=True) if map_nor else None
        img_r = _load_img(f"{maps}_arm.jpg", non_color=True) if map_rough else None
        imgs = [i for i in (img_d, img_n, img_r) if i]
        if imgs:
            any_map = True
            coord = nodes.new("ShaderNodeTexCoord")
            coord.location = (-1350, 100)
            if rough_map_generated:
                vector = coord.outputs["Generated"]      # UV 없는 메시(분철 조각)
            else:
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
                try:                                      # 5.x: SeparateColor, 구버전: SeparateRGB
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
    if not any_map and noise_bump > 0:                   # 텍스처 없을 때 절차 폴백
        tex = nodes.new("ShaderNodeTexNoise")
        tex.location = (-930, -180)
        _set(tex, ["Scale"], noise_scale)
        _set(tex, ["Detail"], 8.0)
        bump = nodes.new("ShaderNodeBump")
        bump.location = (-640, -180)
        _set(bump, ["Strength"], noise_bump)
        links.new(tex.outputs["Fac"], bump.inputs["Height"])
        links.new(bump.outputs["Normal"], bsdf.inputs["Normal"])
    return mat


def build_materials():
    m = {}
    m["paint"]  = make_material("Truck_Paint", "E9ECEF", rough=.35, metal=.05, clearcoat=.6)   # 캡
    m["black"]  = make_material("Truck_Black", "2A2D31", rough=.7, metal=.3)                   # 캡 하부·범퍼
    m["dark"]   = make_material("Truck_Dark", "1D1F22", rough=.6, metal=.5)                   # 프레임·스텝·미러
    m["grille"] = make_material("Grille", "15171A", rough=.5, metal=.6)
    m["slat"]   = make_material("Grille_Slat", "6B6B72", rough=.4, metal=.9)
    m["glass"]  = make_material("Glass", "0F1820", rough=.05, metal=.2, clearcoat=1.0)
    m["chrome"] = make_material("Chrome", "9A9EA4", rough=.25, metal=.95)                      # 배기관
    m["tank"]   = make_material("Fuel_Tank", "B8BCC2", rough=.3, metal=.9)
    m["steel"]  = make_material("Bed_Steel", "7D838A", rough=.55, metal=.55,
                                maps="metal_plate", uv_scale=1.4,
                                noise_bump=.25, noise_scale=60.)                              # 적재함 벽 (HTML metal_plate)
    m["rust"]   = make_material("Bed_Rust", "B8AEA6", rough=.7, metal=.35,
                                maps="rusty_metal_02", uv_scale=1.2,
                                noise_bump=.35, noise_scale=45.)                              # 적재함 바닥 (HTML rusty_metal_02)
    m["mound"]  = make_material("Scrap_Mound", "8A8E94", rough=.55, metal=.5,
                                maps="metal_plate", uv_scale=1.4,
                                map_diff=False, map_rough=False,
                                noise_bump=.3, noise_scale=80.)                               # HTML: 노멀맵만
    m["chunk"] = [make_material(f"Scrap_{i+1:02d}", c, rough=.42, metal=.62,
                                maps="metal_plate", map_diff=False, map_nor=False,
                                rough_map_generated=True)
                  for i, c in enumerate(CHUNK_PALETTE)]                                      # HTML: arm 을 거칠기맵으로
    m["rebar"]  = [make_material("Rebar_Rust1", "A77452", rough=.8, metal=.35, noise_bump=.2),
                   make_material("Rebar_Rust2", "7B5A44", rough=.85, metal=.3, noise_bump=.25)]
    m["tyre"]   = make_material("Tyre", "16181A", rough=.95, metal=0.)
    m["rim"]    = make_material("Wheel_Rim", "8C9196", rough=.5, metal=.8)
    m["head"]   = make_material("Headlight", "FFFFFF", emit="FFF3D6", emit_strength=3.0)
    m["tail_r"] = make_material("Taillight_Red", "3A0D0D", emit="D23A2A", emit_strength=.5)
    m["tail_a"] = make_material("Taillight_Amber", "222222", rough=.5, metal=.1)
    return m

# --------------------------------------------------------------------------
# 지오메트리 헬퍼 (bmesh 만 사용 - 오퍼레이터/컨텍스트 의존 없음)
# --------------------------------------------------------------------------
CUBE_CORNERS = [(-1, -1, -1), (1, -1, -1), (1, 1, -1), (-1, 1, -1),
                (-1, -1, 1), (1, -1, 1), (1, 1, 1), (-1, 1, 1)]
CUBE_FACES = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (3, 7, 6, 2), (0, 4, 7, 3), (1, 2, 6, 5)]

# 아이코사면체 12정점/20면 (HTML 의 IcosahedronGeometry(1, 0) 에 해당)
_T = (1 + sqrt(5)) / 2
ICO_VERTS = [(-1, _T, 0), (1, _T, 0), (-1, -_T, 0), (1, -_T, 0), (0, -1, _T), (0, 1, _T),
             (0, -1, -_T), (0, 1, -_T), (_T, 0, -1), (_T, 0, 1), (-_T, 0, -1), (-_T, 0, 1)]
ICO_VERTS = [Vector(v).normalized() for v in ICO_VERTS]
ICO_FACES = [(0, 11, 5), (0, 5, 1), (0, 1, 7), (0, 7, 10), (0, 10, 11),
             (1, 5, 9), (5, 11, 4), (11, 10, 2), (10, 7, 6), (7, 1, 8),
             (3, 9, 4), (3, 4, 2), (3, 2, 6), (3, 6, 8), (3, 8, 9),
             (4, 9, 5), (2, 4, 11), (6, 2, 10), (8, 6, 7), (9, 8, 1)]


class Part:
    """하나의 Blender 오브젝트로 나갈 부품군. 원시형을 추가하고 한 번에 메시로 굽는다."""

    def __init__(self, name, materials):
        self.name = name
        self.materials = materials
        self.bm = bmesh.new()

    def box(self, size, loc, mat_idx=0, bevel=0.0, rot=None):
        """size=(x,y,z) 중심 loc, 모서리 둥글기 bevel (HTML 의 je/dn 헬퍼)."""
        bm = self.bm
        pre_v, pre_f, pre_e = set(bm.verts), set(bm.faces), set(bm.edges)
        vs = [bm.verts.new(Vector(c) * (Vector(size) * .5)) for c in CUBE_CORNERS]
        for f in CUBE_FACES:
            bm.faces.new([vs[i] for i in f])
        if bevel > 0:
            # 얇은 부품에서 bevel 이 두께의 절반에 가까워지면 느려지므로 24% 로 클램프.
            # geom 에는 이번 박스의 가장자리만 넣는다(누적 분할 재bevel 방지).
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
        """Z 축 원뿔대/원기둥을 loc 에 추가 (rot 로 방향 조정). 새 정점만 변환된다."""
        bm = self.bm
        pre_v, pre_f = set(bm.verts), set(bm.faces)
        bmesh.ops.create_cone(bm, cap_ends=cap, cap_tris=False, segments=seg,
                              radius1=r, radius2=r, depth=h)
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

    def finish(self, parent=None, flat=False, uv_scale=0.0):
        """uv_scale>0 이면 박스(3평면) UV 를 생성한다 - 텍스처용. HTML 의 jl() 박스 매핑과 동일."""
        me = bpy.data.meshes.new(self.name)
        if uv_scale > 0:
            self.box_uvs(self.bm, uv_scale)
        self.bm.to_mesh(me)
        self.bm.free()
        for mat in self.materials:
            me.materials.append(mat)
        if flat:
            for p in me.polygons:
                p.use_smooth = False
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

# --------------------------------------------------------------------------
# 분철 더미 높이맵 (HTML 의 K 클래스 + Qe() 초기 적재를 정적 버전으로 이식)
# --------------------------------------------------------------------------
class Mound:
    """적재함 내부에 (1-u^4)(1-v^4) 프로파일 + 사인 노이즈 + 완화(relax)로 쌓인 분철 언덕."""

    CELL = 0.08

    def __init__(self, x0, x1, y0, y1, floor):
        self.x0, self.x1, self.y0, self.y1, self.floor = x0, x1, y0, y1, floor
        self.nx = round((x1 - x0) / self.CELL) + 1     # 길이방향 격자
        self.ny = round((y1 - y0) / self.CELL) + 1     # 폭방향 격자
        self.h = [[0.0] * self.nx for _ in range(self.ny)]
        self.vn = [[rng.random() for _ in range(self.nx)] for _ in range(self.ny)]
        self.fill()

    def fill(self):
        cx, cy = (self.x0 + self.x1) / 2, (self.y0 + self.y1) / 2
        fu = (self.y1 - self.y0) * .56          # HTML: w*.56 (폭)
        fv = (self.x1 - self.x0) * .62          # HTML: d*.62 (길이)
        for j in range(self.ny):
            y = self.y0 + j * self.CELL
            for i in range(self.nx):
                x = self.x0 + i * self.CELL
                u = (y - cy) / fu
                v = (x - cx) / fv
                noise = .12 * sin(y * 2.1 + 1.3) * cos(x * 2.7) + .08 * sin(y * 4.3 + x * 1.7)
                self.h[j][i] = max(0., min(1.32, 1.12 * (1 - u ** 4) * (1 - v ** 4) + noise))
        self.relax(20)

    def relax(self, n):
        g = .7 * self.CELL                       # HTML: 기울기 한계 .7*cell
        for _ in range(n):
            for j in range(self.ny):
                for i in range(self.nx):
                    if i + 1 < self.nx:
                        self._level(j, i, j, i + 1, g)
                    if j + 1 < self.ny:
                        self._level(j, i, j + 1, i, g)

    def _level(self, ja, ia, jb, ib, g):
        ha, hb = self.h[ja][ia], self.h[jb][ib]
        d = ha - hb
        if d > g:
            t = (d - g) * .5
            self.h[ja][ia] -= t
            self.h[jb][ib] += t
        elif -d > g:
            t = (-d - g) * .5
            self.h[jb][ib] -= t
            self.h[ja][ia] += t

    def sample(self, x, y):
        """높이 이중선형 보간 (HTML K.sample)."""
        fx = min(max((x - self.x0) / self.CELL, 0), self.nx - 1.001)
        fy = min(max((y - self.y0) / self.CELL, 0), self.ny - 1.001)
        i, j = int(fx), int(fy)
        tx, ty = fx - i, fy - j
        h = self.h
        return (h[j][i] * (1 - tx) + h[j][i + 1] * tx) * (1 - ty) + \
               (h[j + 1][i] * (1 - tx) + h[j + 1][i + 1] * tx) * ty

    def build_mesh(self, name, mat, parent):
        bm = bmesh.new()
        grid = [[bm.verts.new(Vector((self.x0 + i * self.CELL,
                                      self.y0 + j * self.CELL,
                                      self.floor + self.h[j][i] +
                                      (self.vn[j][i] - .5) * .1 * min(1., self.h[j][i] / .25))))
                 for i in range(self.nx)] for j in range(self.ny)]
        for j in range(self.ny - 1):
            for i in range(self.nx - 1):
                bm.faces.new((grid[j][i], grid[j][i + 1], grid[j + 1][i + 1], grid[j + 1][i]))
        me = bpy.data.meshes.new(name)
        Part.box_uvs(bm, 1.4)                     # 노멀맵용 UV (위에서 보는 평면 매핑)
        bm.to_mesh(me)
        bm.free()
        me.materials.append(mat)
        obj = bpy.data.objects.new(name, me)
        COLL.objects.link(obj)
        obj.parent = parent
        return obj

# --------------------------------------------------------------------------
# 트럭 본체 (치수 = HTML je/dn/Pn 호출값 그대로, three.js(x,y,z)→blender(x,z,y) 변환)
# --------------------------------------------------------------------------
def build_cab(mats, truck):
    p = Part("Cab", [mats["paint"], mats["black"], mats["glass"], mats["dark"]])
    p.box((2.1, 2.46, 2.2), (-3.75, 0, 2.05), mat_idx=0, bevel=.18)        # 캡 본체
    p.box((2.05, 2.5, .5), (-3.75, 0, .95), mat_idx=1, bevel=.08)          # 캡 하부
    p.box((.06, 2.1, .95), (-4.79, 0, 2.45), mat_idx=2, bevel=.03)         # 윈드실드
    for s in (-1, 1):
        p.box((1.1, .04, .75), (-3.95, s * 1.235, 2.45), mat_idx=2, bevel=.03)   # 사이드 유리
        p.box((.05, .35, .05), (-4.7, s * 1.38, 2.5), mat_idx=3)                 # 미러 스톡
        p.box((.08, .22, .38), (-4.7, s * 1.58, 2.35), mat_idx=3, bevel=.03)     # 미러 헤드
    return p.finish(truck)


def build_front(mats, truck):
    p = Part("Front_Grille_Bumper", [mats["grille"], mats["slat"], mats["black"], mats["head"]])
    p.box((.08, 1.6, .6), (-4.82, 0, 1.3), mat_idx=0, bevel=.03)            # 그릴
    for i in range(6):
        p.box((.02, 1.5, .03), (-4.865, 0, 1.07 + i * .09), mat_idx=1)      # 슬랫 6매
    p.box((.25, 2.5, .28), (-4.85, 0, .75), mat_idx=2, bevel=.05)           # 범퍼
    for s in (-1, 1):
        p.box((.06, .34, .16), (-4.98, s * .95, .8), mat_idx=3, bevel=.03)  # 헤드라이트
    return p.finish(truck)


def build_chassis(mats, truck):
    p = Part("Chassis_Frame", [mats["dark"], mats["tank"], mats["chrome"],
                             mats["tail_r"], mats["tail_a"]])
    p.box((7.9, .9, .32), (-.7, 0, .72), mat_idx=0, bevel=.04)              # 메인 프레임 레일
    rx = Euler((pi / 2, 0, 0))                                              # Z축 → X축
    for s in (-1, 1):
        p.box((.5, .3, .06), (-4.2, s * 1.15, .75), mat_idx=0, bevel=.02)   # 사이드 스텝
        p.cone(.28, .9, (-2.3, s * .85, .75), mat_idx=1, rot=rx)            # 연료탱크
        p.box((.05, .22, .12), (3.3, s * 1.0, .95), mat_idx=3, bevel=.02)   # 미등(적)
        p.box((.05, .12, .10), (3.3, s * .8, .95), mat_idx=4, bevel=.02)    # 미등(황)
    p.cone(.07, 1.6, (-2.6, -1.1, 2.4), mat_idx=2)                          # 배기 스택
    p.box((.17, .17, .04), (-2.6, -1.1, 3.23), mat_idx=2, bevel=.01)        # 배기 마개
    return p.finish(truck)


def build_bed(mats, truck):
    p = Part("DumpBed", [mats["steel"], mats["rust"], mats["dark"]])
    p.box((BED_LEN, BED_W + .12, .12), (BED_CX, 0, BED_FLOOR_H - .06), mat_idx=1)     # 바닥
    wall_z = BED_FLOOR_H + BED_WALL_H / 2
    for s in (-1, 1):
        p.box((BED_LEN, .07, BED_WALL_H), (BED_CX, s * (BED_W / 2 + .035), wall_z), mat_idx=0)
        rib_x = BED_X0 + .4
        while rib_x < BED_X0 + BED_LEN:                                               # 리브 6개
            p.box((.09, .1, BED_WALL_H), (rib_x, s * (BED_W / 2 + .11), wall_z), mat_idx=0)
            rib_x += .9
        p.box((BED_LEN + .05, .16, .1), (BED_CX, s * (BED_W / 2 + .06),
                                         BED_FLOOR_H + BED_WALL_H), mat_idx=0)        # 상부 레일
    p.box((.07, BED_W + .2, BED_WALL_H + .5),
          (BED_X0 - .035, 0, BED_FLOOR_H + (BED_WALL_H + .5) / 2), mat_idx=0)         # 전벽
    p.box((.08, BED_W + .14, BED_WALL_H), (BED_X0 + BED_LEN + .04, 0, wall_z),
          mat_idx=0)                                                                  # 후문
    p.box((BED_LEN, 1, .25), (BED_CX, 0, BED_FLOOR_H - .24), mat_idx=2)               # 언더빔
    return p.finish(truck, uv_scale=1.3)            # 강판/녹 텍스처용 박스 UV


def build_wheel(mats, truck, name, x, y):
    """타이어 + 림 + 허브 + 볼트 6개. 오브젝트 원점=휠 중심, 회전축 Y (HTML 접지 반경 .53)."""
    p = Part(name, [mats["tyre"], mats["rim"]])
    ryz = Euler((pi / 2, 0, 0))
    p.cone(.53, .34, (0, 0, 0), mat_idx=0, seg=28, rot=ryz)      # 타이어
    p.cone(.27, .38, (0, 0, 0), mat_idx=1, seg=24, rot=ryz)      # 림
    p.cone(.09, .42, (0, 0, 0), mat_idx=1, seg=12, rot=ryz)      # 허브
    for k in range(6):                                           # 휠 볼트
        a = k * pi / 3
        p.cone(.02, .46, (cos(a) * .16, 0, sin(a) * .16), mat_idx=1, seg=6, rot=ryz)
    obj = p.finish(truck)
    obj.location = (x, y, .53)
    return obj

# --------------------------------------------------------------------------
# 화물 - 분철 조각 + 철근
# --------------------------------------------------------------------------
def noisy_ico_template():
    """HTML 의 I: 아이코사면체를 정점별 랜덤 반경(0.55~1.3)으로 뭉개진 공유 템플릿."""
    scale = [rng.uniform(.55, 1.3) for _ in ICO_VERTS]
    return [v * s for v, s in zip(ICO_VERTS, scale)]


def build_chunks(mats, mound, parent):
    """분철 조각: 템플릿을 랜덤 회전·스케일(HTML update() 의 s*1.7/s*.5/s*1.2)로
    더미 표면에 뿌린다. 14색 팔레트를 면 단위 material_index 로 칠한다(HTML setColorAt)."""
    tpl = noisy_ico_template()
    p = Part("Scrap_Chunks", mats["chunk"])
    bm = p.bm
    made, tries = 0, 0
    while made < CHUNK_COUNT and tries < CHUNK_COUNT * 4:
        tries += 1
        x = rng.uniform(mound.x0, mound.x1)
        y = rng.uniform(mound.y0, mound.y1)
        h = mound.sample(x, y)
        if h < .05:                       # HTML: h<.04 인 조각은 숨김
            continue
        s = rng.uniform(.05, .13)
        rot = Euler((rng.uniform(0, 6), rng.uniform(0, 6), rng.uniform(0, 6))).to_matrix()
        sc = Matrix.Diagonal((s * 1.7, s * .5, s * 1.2, 1.0))
        z = mound.floor + h + .005 + rng.random() * .035
        vs = [bm.verts.new((rot @ (sc @ v)) + Vector((x, y, z))) for v in tpl]
        mi = made % len(mats["chunk"])
        for f in ICO_FACES:
            face = bm.faces.new([vs[i] for i in f])
            face.material_index = mi
        made += 1
    obj = p.finish(parent, flat=True)
    return obj, made


def build_rebars(mats, mound, parent):
    """철근 다발: 마운드 위에 길이방향 2열 × 2층 + 굽은 철근 2개(능선을 가로질러 드리움)."""
    p = Part("Rebar_Bundles", mats["rebar"])
    bm = p.bm
    r = .024
    rod_len = 5.2
    for ci, cy in enumerate((-.62, .62)):
        for n, dy, dz in ((10, 0., 0.), (8, .055, r * 1.72)):    # 1층 10개, 2층 8개(지그재그)
            for i in range(n):
                x = BED_CX + rng.uniform(-.06, .06)
                y = max(min(cy + dy - .05 + i * .105 + rng.uniform(-.012, .012),
                            mound.y1 - .05), mound.y0 + .05)
                z = mound.floor + mound.sample(x, y) + r + dz + .012
                # 원기둥(Z축)을 X축 방향로 눕히고 살짝 기울인다
                rot = Euler((rng.uniform(-.02, .02), pi / 2 + rng.uniform(-.025, .025), 0))
                pre_v, pre_f = set(bm.verts), set(bm.faces)
                bmesh.ops.create_cone(bm, cap_ends=True, cap_tris=False, segments=6,
                                      radius1=r, radius2=r, depth=rod_len)
                m = Matrix.Translation(Vector((x, y, z))) @ rot.to_matrix().to_4x4()
                bmesh.ops.transform(bm, matrix=m, verts=[v for v in bm.verts if v not in pre_v])
                for f in bm.faces:
                    if f not in pre_f:
                        f.material_index = (ci + i) % 2
    # 굽은 철근: 8분할 폴리라인 세그먼트
    for bx in (-1.15, 1.95):
        pts = []
        steps = 8
        for k in range(steps + 1):
            t = k / steps
            y = -1.18 + t * 2.36
            z = mound.floor + mound.sample(bx, y) + .03 + sin(t * pi) * .34
            pts.append(Vector((bx + sin(t * 2.2) * .18, y, z)))
        for a, b in zip(pts, pts[1:]):
            d = b - a
            pre_v, pre_f = set(bm.verts), set(bm.faces)
            bmesh.ops.create_cone(bm, cap_ends=False, cap_tris=False, segments=6,
                                  radius1=r, radius2=r, depth=d.length)
            m = Matrix.Translation((a + b) / 2) @ d.to_track_quat("Z", "Y").to_matrix().to_4x4()
            bmesh.ops.transform(bm, matrix=m, verts=[v for v in bm.verts if v not in pre_v])
            for f in bm.faces:
                if f not in pre_f:
                    f.material_index = rng.randrange(2)
    return p.finish(parent)

# --------------------------------------------------------------------------
# 프리뷰 (GUI 로 열 때만) / 정리
# --------------------------------------------------------------------------
COLL_NAME = "DumpTruck"
PREVIEW_NAME = "Preview"


MAT_PREFIXES = ("Truck_", "Grille", "Glass", "Chrome", "Fuel_Tank", "Bed_", "Scrap_",
                "Rebar_", "Tyre", "Wheel_Rim", "Headlight", "Taillight_", "Preview_")


# 예전 버전이 만든 한글 이름 결과(영어 이름으로 바꾸기 전)도 지울 수 있게 하는 레거시 접두사
LEGACY_COLL = ("덤프트럭_분철", "프리뷰")
LEGACY_OBJ_PREFIXES = ("바퀴_", "분철", "철근", "운전석", "적재함", "샤시", "앞_",
                       "바닥", "태양", "카메라")
LEGACY_MAT_PREFIXES = ("트럭_", "그릴", "유리", "크롬", "연료탱크", "적재함_", "분철_",
                       "철근_", "타이어", "휠림", "헤드라이트", "미등_", "프리뷰_")


def cleanup_previous():
    """같은 스크립트를 다시 실행했을 때 이전 결과만 지운다 (사용자 데이터는 건드리지 않음).
    재질도 다시 만들어야 텍스처 설정 변경이 반영되므로 함께 퍼지한다."""
    for name in (COLL_NAME, PREVIEW_NAME) + LEGACY_COLL:
        coll = bpy.data.collections.get(name)
        if coll:
            for obj in list(coll.objects):
                bpy.data.objects.remove(obj, do_unlink=True)
            bpy.data.collections.remove(coll)
    # 콜렉션 밖에 남은 이전 결과(프리뷰 카메라 등)도 제거
    for obj in list(bpy.data.objects):
        if obj.name.startswith(("Wheel_", "Scrap", "Rebar", "Cab", "DumpBed", "Chassis",
                                "Front_", "Preview_") + LEGACY_OBJ_PREFIXES) or obj.name == "Truck":
            bpy.data.objects.remove(obj, do_unlink=True)
    for mat in list(bpy.data.materials):
        if mat.name.startswith(MAT_PREFIXES + LEGACY_MAT_PREFIXES):
            bpy.data.materials.remove(mat)
    for mesh in list(bpy.data.meshes):              # 고아 메시 정리
        if mesh.users == 0:
            bpy.data.meshes.remove(mesh)


def add_preview():
    col = bpy.data.collections.new(PREVIEW_NAME)
    bpy.context.scene.collection.children.link(col)

    ground = bpy.data.meshes.new("Preview_Ground")
    bm = bmesh.new()
    bmesh.ops.create_grid(bm, x_segments=1, y_segments=1, size=14)
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
    sl.rotation_euler = Vector((7, 5, -11)).to_track_quat("-Z", "Y").to_euler()
    col.objects.link(sl)

    cam_data = bpy.data.cameras.new("Preview_Camera")
    cam_data.lens = 42
    c = bpy.data.objects.new("Preview_Camera", cam_data)
    c.location = (-10.5, -7.5, 6.5)
    c.rotation_euler = (Vector((0, 0, 1.5)) - c.location).to_track_quat("-Z", "Y").to_euler()
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
    out = {"export": None, "chunks": CHUNK_COUNT, "seed": SEED}
    i = 0
    while i < len(argv):
        if argv[i] == "--export" and i + 1 < len(argv):
            out["export"] = argv[i + 1]
            i += 2
        elif argv[i] == "--chunks" and i + 1 < len(argv):
            out["chunks"] = int(argv[i + 1])
            i += 2
        elif argv[i] == "--seed" and i + 1 < len(argv):
            out["seed"] = int(argv[i + 1])
            i += 2
        else:
            i += 1
    return out


def main():
    global rng, CHUNK_COUNT, COLL
    args = parse_args()
    CHUNK_COUNT = args["chunks"]
    rng = random.Random(args["seed"])

    scene = bpy.context.scene
    scene.unit_settings.system = "METRIC"
    scene.unit_settings.scale_length = 1.0

    cleanup_previous()
    COLL = bpy.data.collections.new(COLL_NAME)
    scene.collection.children.link(COLL)

    mats = build_materials()

    truck = bpy.data.objects.new("Truck", None)
    truck.empty_display_size = 2.0
    COLL.objects.link(truck)

    P("[1/6] Modeling cab / grille / chassis ...")
    build_cab(mats, truck)
    build_front(mats, truck)
    build_chassis(mats, truck)
    P("[2/6] Modeling dump bed ...")
    build_bed(mats, truck)
    P("[3/6] Assembling 6 wheels ...")
    for x, kn in ((-3.75, "Front"), (1.0, "Mid"), (2.45, "Rear")):
        for y, ks in ((-1.02, "L"), (1.02, "R")):
            build_wheel(mats, truck, f"Wheel_{kn}_{ks}", x, y)
    P("[4/6] Building scrap mound heightfield ...")
    mound = Mound(BED_X0 + .06, BED_X0 + BED_LEN - .06,
                  -BED_W / 2 + .05, BED_W / 2 - .05, BED_FLOOR_H)
    mound.build_mesh("Scrap_Pile", mats["mound"], truck)
    P(f"[5/6] Scattering {CHUNK_COUNT} scrap chunks ...")
    _, n = build_chunks(mats, mound, truck)
    build_rebars(mats, mound, truck)

    verts = sum(len(o.data.vertices) for o in truck.children if o.data)
    P(f"[6/6] Done: {len(truck.children)} objects, {verts:,} verts, {n} scrap chunks")
    P("Truck size: 8.3 m long x 2.5 m wide x 3.25 m tall (exhaust cap)")

    if args["export"]:
        out = args["export"]
        # 시작 씬의 기본 오브젝트(큐브 등)가 섞이지 않도록 트럭 콜렉션만 선택해 내보낸다
        bpy.ops.object.select_all(action="DESELECT")
        for obj in COLL.objects:
            obj.select_set(True)
        bpy.ops.wm.usd_export(filepath=out, selected_objects_only=True)
        P(f"USD exported: {out}")
    else:
        add_preview()


if __name__ == "__main__":
    main()
