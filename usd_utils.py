"""USD(pxr) 프리미티브 생성/갱신 헬퍼 - 물리 없는 시각 요소 전용.

크레인 구조물·조종실·승강 로봇은 전부 여기 헬퍼로 만든 비물리 프림이다.
(물리 시뮬레이션 대상은 철근뿐 - yard.py 참조)
"""
import math

import numpy as np
from pxr import Gf, Sdf, UsdGeom, UsdShade


# ═══════════════════════ 쿼터니언 유틸 (w, x, y, z 순서) ═══════════════════════
def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def quat_axis_angle(axis, angle):
    axis = np.asarray(axis, dtype=float)
    n = np.linalg.norm(axis)
    if n < 1e-9:
        return np.array([1.0, 0.0, 0.0, 0.0])
    axis = axis / n
    half = 0.5 * angle
    return np.array([np.cos(half), *(np.sin(half) * axis)])


def quat_between(a, b):
    """단위벡터 a → b 로 돌리는 회전."""
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    a = a / max(np.linalg.norm(a), 1e-9)
    b = b / max(np.linalg.norm(b), 1e-9)
    d = float(np.clip(np.dot(a, b), -1.0, 1.0))
    if d > 1.0 - 1e-9:
        return np.array([1.0, 0.0, 0.0, 0.0])
    if d < -1.0 + 1e-9:
        axis = np.cross(a, [1.0, 0.0, 0.0])
        if np.linalg.norm(axis) < 1e-6:
            axis = np.cross(a, [0.0, 1.0, 0.0])
        return quat_axis_angle(axis, np.pi)
    return quat_axis_angle(np.cross(a, b), np.arccos(d))


# ═══════════════════════ 재질 ═══════════════════════
def make_material(stage, path, color, roughness=0.6, metallic=0.0, opacity=1.0):
    mat = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, f"{path}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*[float(c) for c in color]))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(roughness))
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(float(metallic))
    if opacity < 1.0:
        shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(float(opacity))
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return mat


def _bind(prim, mat):
    UsdShade.MaterialBindingAPI(prim).Bind(mat)


# ═══════════════════════ 변환(이동/회전/스케일) ═══════════════════════
def _make_op(gprim, pos, quat, scale):
    op = gprim.MakeMatrixXform() if hasattr(gprim, "MakeMatrixXform") else gprim.AddTransformOp()
    set_xform(op, pos, quat, scale)
    return op


def _quat_to_rotation(quat):
    """쿼터니언(w, x, y, z) → Gf.Rotation(축-각도, 도 단위).

    최신 pxr에서 Matrix4d(Quatf/Quatd) 생성자가 제거되어, 어느 버전에서나
    존재하는 축-각도 경로로 회전을 만든다.
    """
    w, x, y, z = (float(v) for v in quat)
    if w < 0.0:  # q와 -q는 같은 회전 → w≥0으로 정규화해 최소 회전 표현
        w, x, y, z = -w, -x, -y, -z
    s = math.sqrt(max(0.0, 1.0 - w * w))
    if s < 1e-9:
        return Gf.Rotation(Gf.Vec3d(1.0, 0.0, 0.0), 0.0)
    angle = 2.0 * math.degrees(math.acos(min(1.0, w)))
    return Gf.Rotation(Gf.Vec3d(x / s, y / s, z / s), angle)


def set_xform(op, pos, quat=(1.0, 0.0, 0.0, 0.0), scale=(1.0, 1.0, 1.0)):
    """생성 시 받아둔 transform op의 위치/자세/스케일을 갱신한다."""
    # M = S * R (row-vector 관례: 스케일 먼저 적용) + 이동
    m = Gf.Matrix4d().SetScale(Gf.Vec3d(*[float(s) for s in scale])) * Gf.Matrix4d().SetRotate(_quat_to_rotation(quat))
    m.SetTranslateOnly(Gf.Vec3d(*[float(p) for p in pos]))
    op.Set(m)


# ═══════════════════════ 프리미티브 ═══════════════════════
def group(stage, path, pos=(0.0, 0.0, 0.0), quat=(1.0, 0.0, 0.0, 0.0)):
    """빈 좌표계 그룹(계층 루트용). 반환값: 갱신용 transform op."""
    xf = UsdGeom.Xform.Define(stage, path)
    return _make_op(xf, pos, quat, (1.0, 1.0, 1.0))


def box(stage, path, size=1.0, mat=None, pos=(0.0, 0.0, 0.0), quat=(1.0, 0.0, 0.0, 0.0), scale=(1.0, 1.0, 1.0)):
    g = UsdGeom.Cube.Define(stage, path)
    g.CreateSizeAttr(float(size))
    op = _make_op(g, pos, quat, scale)
    if mat:
        _bind(g, mat)
    return op


def sphere(stage, path, radius, mat=None, pos=(0.0, 0.0, 0.0)):
    g = UsdGeom.Sphere.Define(stage, path)
    g.CreateRadiusAttr(float(radius))
    op = _make_op(g, pos, (1.0, 0.0, 0.0, 0.0), (1.0, 1.0, 1.0))
    if mat:
        _bind(g, mat)
    return op


def cylinder(stage, path, radius, height, mat=None, pos=(0.0, 0.0, 0.0),
             quat=(1.0, 0.0, 0.0, 0.0), axis="Z"):
    g = UsdGeom.Cylinder.Define(stage, path)
    g.CreateRadiusAttr(float(radius))
    g.CreateHeightAttr(float(height))
    try:
        g.CreateAxisAttr(axis)      # 구형 USD에서만 필요, 실패 시 기본 Z축
    except Exception:
        pass
    op = _make_op(g, pos, quat, (1.0, 1.0, 1.0))
    if mat:
        _bind(g, mat)
    return op


def capsule(stage, path, radius, height, mat=None, pos=(0.0, 0.0, 0.0)):
    """가변 길이 캡슐(뼈대·와이어용). 반환값: (프림, transform op, height attr, 반지름)."""
    g = UsdGeom.Capsule.Define(stage, path)
    g.CreateRadiusAttr(float(radius))
    h_attr = g.CreateHeightAttr(float(height))
    op = _make_op(g, pos, (1.0, 0.0, 0.0, 0.0), (1.0, 1.0, 1.0))
    if mat:
        _bind(g, mat)
    return g, op, h_attr, radius


def place_segment(capsule_tuple, a, b):
    """캡슐을 점 a→b 구간에 정확히 놓는다(길이·자세 자동 갱신)."""
    g, op, h_attr, r = capsule_tuple
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    d = b - a
    length = float(np.linalg.norm(d))
    if length < 1e-6:
        d = np.array([0.0, 0.0, 1.0])
        length = 1e-6
    h_attr.Set(max(length - 2.0 * r, 0.02))
    quat = quat_between((0.0, 0.0, 1.0), d / length)
    set_xform(op, (a + b) * 0.5, quat)
