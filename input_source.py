"""입력 소스: 게임패드 / 스크립트(자동 파일럿). 스틱 필터(데드존·커브·스무딩) 포함.

명령 규약(웹 페이지 시나리오와 동일한 축 배치):
  lx  왼쪽 스틱 X  (-1~1, +1 = 동쪽)   → 크레인 X(횡행)
  ly  왼쪽 스틱 Y  (-1~1, +1 = 북쪽)   → 크레인 Y(주행)
  rz  오른쪽 스틱 Y (-1~1, +1 = 상승)  → 후크 Z(권상)
  pedal 브레이크 페달 (0/1, 1 = 크레인 정지)
  mag   자석 토글 (0/1, 상승 에지에서만 1 - 전자석 ON/OFF)
"""
import numpy as np

import config as C


class Command:
    __slots__ = ("lx", "ly", "rz", "pedal", "mag")

    def __init__(self, lx=0.0, ly=0.0, rz=0.0, pedal=0.0, mag=0.0):
        self.lx = lx
        self.ly = ly
        self.rz = rz
        self.pedal = pedal
        self.mag = mag


class StickFilter:
    """데드존 → 응답 커브 → 로우패스. (게임패드 물리 스틱용 기본값)"""

    def __init__(self, deadzone=C.INPUT_DEADZONE, curve=C.INPUT_CURVE, smoothing=C.INPUT_SMOOTHING):
        self._prev = np.zeros(3)
        self._deadzone = float(deadzone)
        self._curve = float(curve)
        self._smoothing = float(smoothing)

    def __call__(self, raw):
        x = np.clip(np.asarray(raw, dtype=float), -1.0, 1.0)
        out = np.zeros_like(x)
        if self._deadzone > 0.0:
            on = np.abs(x) > self._deadzone
            out[on] = np.sign(x[on]) * ((np.abs(x[on]) - self._deadzone) / (1.0 - self._deadzone)) ** self._curve
        elif self._curve != 1.0:
            out = np.sign(x) * np.abs(x) ** self._curve
        else:
            out = x
        out = self._prev + (out - self._prev) * self._smoothing
        self._prev = out
        return out


# ═══════════════════════ 게임패드 ═══════════════════════
class GamepadSource:
    """isaaclab Gamepad 래퍼.

    주의: get_data()의 키 이름은 Isaac Lab 버전에 따라 다를 수 있다.
    첫 폴링에서 실제 키를 콘솔에 출력하므로, 값이 안 들어오면 AXIS_KEYS의
    후보 문자열을 그 출력에 맞춰 수정한다(수정은 이곳이 유일).
    게임패드 지원 설치: pip install "isaaclab[gamepad]"
    """

    AXIS_KEYS = {
        "lx":    ("/left_stick_x", "/axis_left_x", "lx"),
        "ly":    ("/left_stick_y", "/axis_left_y", "ly"),
        "rz":    ("/right_stick_y", "/axis_right_y", "ry"),
        "pedal": ("/right_trigger", "/trigger_right", "/a", "/button_a"),
        "mag":   ("/b", "/button_b", "/x", "/button_x"),
    }

    def __init__(self):
        from isaaclab.devices.gamepad.gamepad import Gamepad  # 버전에 따라 경로 다름
        self._pad = Gamepad()
        self._filter = StickFilter()
        self._printed_keys = False
        self._mag_prev = 0.0

    def poll(self, dt, state=None):
        data = {}
        try:
            data = self._pad.get_data() or {}
        except Exception as e:  # noqa: BLE001 - 폴링 실패는 중립 명령으로 처리
            if not self._printed_keys:
                print(f"[input] 게임패드 데이터 읽기 실패({e}) - 중립 명령 유지")
                self._printed_keys = True
        if data and not self._printed_keys:
            print(f"[input] 게임패드 키 목록(참고용): {sorted(data.keys())}")
            self._printed_keys = True

        raw = [
            self._get(data, "lx"),
            self._get(data, "ly") * (-1.0 if C.INPUT_INVERT_LY else 1.0),
            self._get(data, "rz") * (-1.0 if C.INPUT_INVERT_RZ else 1.0),
        ]
        lx, ly, rz = self._filter(raw)
        pedal = 1.0 if self._get(data, "pedal") > 0.5 else 0.0
        # 자석은 토글: 버튼 상승 에지에서만 1
        mag_now = 1.0 if self._get(data, "mag") > 0.5 else 0.0
        mag = 1.0 if (mag_now > 0.5 and self._mag_prev <= 0.5) else 0.0
        self._mag_prev = mag_now
        return Command(lx, ly, rz, pedal, mag)

    def _get(self, data, name):
        for key in self.AXIS_KEYS[name]:
            if key in data:
                v = data[key]
                if isinstance(v, (list, tuple)):
                    v = 1.0 if any(bool(x) for x in v) else 0.0
                elif isinstance(v, bool):
                    v = float(v)
                try:
                    return float(v)
                except (TypeError, ValueError):
                    continue
        return 0.0


# ═══════════════════════ 스크립트(자동 파일럿) ═══════════════════════
class ScriptedSource:
    """하드웨어 없이 전체 사이클(픽업→운반→자석 해제→판정→복귀)을 반복 검증하는 자동 조종.

    웹 페이지 시나리오처럼 트럭의 분철을 집어 등급 구역(중량A/B·경량A/B)에
    차례로 하차한다. 크레인 상태(state: x, y, z)를 받아 목표 지점으로 P제어하는
    스틱 명령을 만든다.
    """

    def __init__(self):
        # 자동 파일럿은 물리 스틱이 아니므로 데드존/커브 없이 스무딩만 건다.
        # (데드존을 걸면 P제어 명령이 목표 근처에서 데드존보다 작아져
        #  정착 판정 거리 안에 영원히 도달하지 못하는 교찰이 생긴다.)
        self._filter = StickFilter(deadzone=0.0, curve=1.0)
        self.phase = "idle"
        self.zone_cycle = 0          # 몇 번째 구역에 하차 중인지(0~3 순환)
        self.zone_index = 0
        self._pick_i = 0             # 트럭 적재 다발 위치 순환(중앙→서→동...)
        self._i = 0
        self._hold = 0.0
        self._fresh_phase = True   # 페이즈 진입 첫 스텝(자석 토글 에지는 이때만)
        self._build_phases()

    def _zone_xy(self):
        z = C.ZONES[self.zone_index]
        return (z["cx"], z["cy"])

    # 트럭 적재 다발 위치(자석 필드 반경 0.45m 안에 들어오도록 사이클마다 이동)
    PICKUP_OFFSETS = ((0.0, 0.0), (-1.15, 0.0), (1.15, 0.0), (0.0, -0.55), (0.0, 0.55),
                      (-0.6, 0.35), (0.6, -0.35), (-0.6, -0.35), (0.6, 0.35))

    def _pickup_xy(self):
        ox, oy = self.PICKUP_OFFSETS[self._pick_i % len(self.PICKUP_OFFSETS)]
        return (C.TRUCK_XY[0] + ox, C.TRUCK_XY[1] + oy)

    def _build_phases(self):
        zx, zy = self._zone_xy()
        px, py = self._pickup_xy()
        # (단계명, 목표 (x, y, hook_z), 정착 유지시간, 브레이크, 자석토글에지)
        self.phases = [
            ("goto_truck", (px, py, 4.5), 0.5, 0, 0),
            ("descend",    (px, py, 1.18), 1.0, 0, 0),   # 자석 하단이 적재 분철에 접촉
            ("settle",     (px, py, 1.18), 1.4, 0, 1),   # 자석 ON → 부착
            ("lift",       (px, py, 4.5), 0.6, 0, 0),
            ("goto_zone",  (zx, zy, 3.5), 0.8, 0, 0),         # 지정 구역 상공
            ("lower",      (zx, zy, 1.0), 0.8, 0, 0),         # 구역 바닥 위로 하강
            ("release",    (zx, zy, 1.0), 1.3, 0, 1),         # 자석 OFF → 낙하·판정
            ("lift2",      (zx, zy, 4.0), 0.6, 0, 0),
            ("home",       (C.CRANE_START[0], 0.0, 4.5), 0.8, 0, 0),
        ]

    @property
    def target_zone(self):
        """현재 목표 구역 인덱스(데모의 판정 비교용)."""
        return self.zone_index

    def poll(self, dt, state=None):
        name, target, settle_t, brake, mag = self.phases[self._i]
        just_entered = self._fresh_phase
        self._fresh_phase = False
        self.phase = name
        target = np.array(target, dtype=float)

        if brake:
            return Command(0.0, 0.0, 0.0, 1.0, 0.0)
        edge = 1.0 if (mag and just_entered) else 0.0  # 토글은 페이즈 진입 시 1회

        if state:
            pos = np.array([state.get("x", target[0]), state.get("y", target[1]), state.get("z", target[2])])
        else:
            pos = target.copy()
        err = target - pos

        raw = np.array([
            np.clip(err[0] * 1.4 / max(C.CRANE_SPEED_XY, 1e-6), -1.0, 1.0),
            np.clip(err[1] * 1.4 / max(C.CRANE_SPEED_XY, 1e-6), -1.0, 1.0),
            np.clip(err[2] * 2.0 / max(C.CRANE_SPEED_Z, 1e-6), -1.0, 1.0),
        ])
        if abs(err[0]) < 0.10 and abs(err[1]) < 0.10 and abs(err[2]) < 0.06:
            self._hold += dt
        else:
            self._hold = 0.0
        if self._hold >= settle_t:
            self._i += 1
            self._hold = 0.0
            self._fresh_phase = True
            if self._i >= len(self.phases):   # 한 사이클 끝 → 다음 등급 구역으로
                self._i = 0
                self.zone_cycle += 1
                self.zone_index = self.zone_cycle % len(C.ZONES)
                self._pick_i += 1                # 다음 사이클은 다른 다발에서 픽업
                self._build_phases()

        lx, ly, rz = self._filter(raw)
        return Command(lx, ly, rz, 0.0, edge)


# ═══════════════════════ 팩토리 ═══════════════════════
def make_source(mode):
    """mode: 'auto' | 'gamepad' | 'scripted' → (소스, 이름)"""
    if mode in ("gamepad", "auto"):
        try:
            src = GamepadSource()
            src.poll(0.0)  # 초기화 조기 실패 감지
            print("[input] 게임패드 연결됨")
            return src, "gamepad"
        except Exception as e:  # noqa: BLE001 - 게임패드 없으면 자동 파일럿으로
            print(f"[input] 게임패드 사용 불가({e}) → 자동 파일럿(scripted)으로 대체")
    return ScriptedSource(), "scripted"
