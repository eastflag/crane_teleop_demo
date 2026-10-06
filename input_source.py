"""입력 소스: 게임패드 / 스크립트(자동 파일럿). 스틱 필터(데드존·커브·스무딩) 포함.

명령 규약:
  lx  왼쪽 스틱 X  (-1~1, +1 = 동쪽)   → 크레인 X
  ly  왼쪽 스틱 Y  (-1~1, +1 = 북쪽)   → 크레인 Y
  rz  오른쪽 스틱 Y (-1~1, +1 = 상승)  → 후크 Z
  pedal 패들 (0/1, 1 = 자석 해제)
"""
import numpy as np

import config as C


class Command:
    __slots__ = ("lx", "ly", "rz", "pedal")

    def __init__(self, lx=0.0, ly=0.0, rz=0.0, pedal=0.0):
        self.lx = lx
        self.ly = ly
        self.rz = rz
        self.pedal = pedal


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
    }

    def __init__(self):
        from isaaclab.devices.gamepad.gamepad import Gamepad  # 버전에 따라 경로 다름
        self._pad = Gamepad()
        self._filter = StickFilter()
        self._printed_keys = False

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
        return Command(lx, ly, rz, pedal)

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
    """하드웨어 없이 전체 사이클(픽업→운반→패들 해제→복귀)을 반복 검증하는 자동 조종.

    크레인 상태(state: x, y, z)를 받아 목표 지점으로 P제어하는 스틱 명령을 만든다.
    """

    def __init__(self):
        # 자동 파일럿은 물리 스틱이 아니므로 데드존/커브 없이 스무딩만 건다.
        # (데드존을 걸면 P제어 명령이 목표 근처에서 데드존보다 작아져
        #  정착 판정 거리 안에 영원히 도달하지 못하는 교착이 생긴다.)
        self._filter = StickFilter(deadzone=0.0, curve=1.0)
        self.phase = "idle"
        self._i = 0
        self._hold = 0.0
        # (단계명, 목표 (x, y, hook_z), 정착 유지시간, 패들)
        self.phases = [
            ("goto_truck", (*C.TRUCK_XY, 4.5), 0.5, 0),
            ("descend",    (*C.TRUCK_XY, 1.18), 1.0, 0),   # 자석 하단이 적재대 분철에 닿는 높이
            ("settle",     (*C.TRUCK_XY, 1.18), 1.4, 0),   # 이 구간에서 분철 부착
            ("lift",       (*C.TRUCK_XY, 4.5), 0.6, 0),
            ("goto_jar",   (*C.JAR_XY, 3.5), 0.8, 0),
            ("lower",      (*C.JAR_XY, 2.5), 0.8, 0),      # 2m 통 개구부 위(자석이 입구 밖)
            ("release",    (*C.JAR_XY, 2.5), 1.3, 1),      # 패들 → 자석 해제, 투입
            ("lift2",      (*C.JAR_XY, 4.0), 0.6, 0),
            ("home",       (*C.CRANE_START[:2], 4.5), 0.8, 0),
        ]

    def poll(self, dt, state=None):
        name, target, settle_t, pedal = self.phases[self._i]
        self.phase = name
        target = np.array(target, dtype=float)

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
            self._i = (self._i + 1) % len(self.phases)
            self._hold = 0.0

        lx, ly, rz = self._filter(raw)
        return Command(lx, ly, rz, float(pedal))


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
