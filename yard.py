"""야드: 씬 애셋(USD) 로드 + 개별 철근/분철 강체 컬렉션 + 등급 구역 판정.

씬 구성물(yard.usd·truck_load.usd·rebar_set.usd·crane.usd)은 전부 USD 애셋으로
준비하고 데모에서 reference로 로드만 한다(스크립트 생성 코드는 없음).
- 트럭 위 분철 더미는 시각+정적 충돌체만 있고, 자석으로 집는 개별 강체는
  더미 위 ~1cm에서 떨어뜨려 안착시킨다(30개 이하).
- 등급별 지정 구역(중량A/B·경량A/B)에 적치한다.
"""
from pathlib import Path

import config as C

ASSET_DIR = Path(__file__).resolve().parent / "assets"
YARD_ASSET = ASSET_DIR / "yard.usd"
TRUCK_ASSET = ASSET_DIR / "truck_load.usd"
REBAR_ASSET = ASSET_DIR / "rebar_set.usd"
CRANE_ASSET = ASSET_DIR / "crane.usd"


def load_scene_assets(stage):
    """야드·트럭·철근·크레인 애셋을 /World 아래 reference로 로드.

    반환: {"yard": 마운트 경로, "truck": ..., "rebar": ..., "crane": ...}
    애셋 파일이 없으면 오류로 종료(씬은 반드시 USD로 구성한다).
    """
    out = {}
    for name, path, mount in (("yard", YARD_ASSET, "/World/Yard"),
                              ("truck", TRUCK_ASSET, "/World/Truck"),
                              ("rebar", REBAR_ASSET, "/World/RebarSet"),
                              ("crane", CRANE_ASSET, "/World/Crane")):
        if not path.is_file():
            raise FileNotFoundError(f"씬 애셋이 없습니다: {path} - USD를 준비해 주세요")
        prim = stage.DefinePrim(mount, "Xform")
        prim.GetReferences().AddReference(str(path))
        out[name] = mount
        print(f"[yard] 애셋 로드: {path.name} → {mount}")
    return out


# ═══════════════════════ 개별 철근/분철 컬렉션 ═══════════════════════
def rebar_count_available():
    """rebar_set.usd 에 실려 있는 강체 수(애셋 생성 시 결정)."""
    from pxr import Usd, UsdPhysics
    stage = Usd.Stage.Open(str(REBAR_ASSET))
    return sum(1 for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI))


def make_rebar_collection_cfg(num=C.REBAR_NUM):
    """자석으로 집는 철근/분철 컬렉션 설정.

    rebar_set.usd(개별 강체 ≤30, 더미 위 1cm)를 reference로 깔아둔 프림을
    그대로 래핑한다(spawn=None). 리지드바디+충돌+질량은 애셋에 구성돼 있다.
    """
    from isaaclab.assets import RigidObjectCfg, RigidObjectCollectionCfg

    entries = {}
    stage_rebar = rebar_count_available()
    for i in range(min(num, stage_rebar)):
        entries[f"rebar_{i}"] = RigidObjectCfg(
            prim_path=f"/World/RebarSet/rebar_{i}",
            spawn=None,  # 이미 애셋에 리지드바디+충돌+질량이 구성됨
            init_state=RigidObjectCfg.InitialStateCfg(),
        )
    print(f"[yard] 개별 강체 {len(entries)}개 - 애셋 프림 래핑(spawn=None)")
    return RigidObjectCollectionCfg(rigid_objects=entries)


# ═══════════════════════ 등급 구역 판정 ═══════════════════════
def zone_at(x, y):
    """(x, y)가 속한 등급 구역 인덱스(0~3). 구역 밖이면 -1."""
    zw, zd = C.ZONE_SIZE
    for i, z in enumerate(C.ZONES):
        if abs(x - z["cx"]) <= zw / 2.0 and abs(y - z["cy"]) <= zd / 2.0:
            return i
    return -1


def zone_name(i):
    return C.ZONES[i]["name"] if 0 <= i < len(C.ZONES) else "구역 밖"


def judge_drop(x, y, target):
    """낙하 지점 판정(웹 페이지의 적치 판정 메시지 재현).

    반환: (정답 여부, 메시지)
    """
    drop = zone_at(x, y)
    if drop == target:
        return True, f"{zone_name(target)} 구역에 정위치 적치했습니다"
    if drop < 0:
        return False, f"적치 구역 밖에 내려놓았습니다 (판정 {zone_name(target)})"
    return False, f"판정은 {zone_name(target)}인데 {zone_name(drop)} 구역에 놓였습니다"
