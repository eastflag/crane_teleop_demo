"""USD(pxr) 프림 갱신 헬퍼 - 애셋에서 로드한 프림의 런타임 핸들 묶기·변환 갱신.

씬 구성물은 전부 USD 애셋으로 준비하고, 이 모듈은 reference로 로드한 프림의
transform op 를 다시 묶어 오버라이드로 갱신하는 데만 쓴다(와이어·페달 등).
"""
import math

import numpy as np
from pxr import Gf, UsdGeom


# ═══════════════════════ 쿼터니언 유틸 (w, x, y, z 순서) ═══════════════════════
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
        return quat_axis_angle(axis, math.pi)
    return quat_axis_angle(np.cross(a, b), math.arccos(d))


# ═══════════════════════ 변환(이동/회전/스케일) ═══════════════════════
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
    """transform op의 위치/자세/스케일을 갱신한다."""
    # M = S * R (row-vector 관례: 스케일 먼저 적용) + 이동
    m = Gf.Matrix4d().SetScale(Gf.Vec3d(*[float(s) for s in scale])) * Gf.Matrix4d().SetRotate(_quat_to_rotation(quat))
    m.SetTranslateOnly(Gf.Vec3d(*[float(p) for p in pos]))
    op.Set(m)


# ═══════════════════ 애셋(.usd)에서 로드한 프림에 핸들 다시 묶기 ═══════════════════
# 로드된 프림은 만들 때의 갱신용 op/attr을 돌려받을 수 없으므로 프림에서 다시
# 꺼낸다. 층위는 로컬(참조하는 쪽) 레이어가 참조보다 강하므로 op.Set()은
# 그대로 오버라이드로 기록된다.
def get_op(prim, create=False):
    """프림의 (유일한) transform op. create=True면 없을 때 새로 만든다."""
    xf = UsdGeom.Xformable(prim)
    ops = xf.GetOrderedXformOps()
    if not ops and create:
        return xf.AddTransformOp()
    if not ops:
        raise RuntimeError(f"transform op가 없는 프림: {prim.GetPath()}")
    return ops[0]


def attach_op(stage, path):
    """로드된 프림의 transform op를 반환한다."""
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        raise RuntimeError(f"애셋에서 프림을 찾을 수 없음: {path}")
    return get_op(prim)


def attach_capsule(stage, path):
    """로드된 캡슐을 place_segment용 튜플 (프림, op, height attr, 반지름)로 반환한다."""
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        raise RuntimeError(f"애셋에서 프림을 찾을 수 없음: {path}")
    g = UsdGeom.Capsule(prim)
    return g, get_op(prim), g.GetHeightAttr(), float(g.GetRadiusAttr().Get())


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
