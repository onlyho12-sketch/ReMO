"""Pi ↔ OpenCR USB 시리얼 프로토콜 (ASCII 한 줄, 115200 baud).

Pi → OpenCR (루프마다, 15Hz)
    $CMD,<v_mm_s>,<w_mrad_s>,<mode>*<XX>\n
    mode: 0 = 일반 주행, 1 = 턱 넘기 요청, 2 = 정지
OpenCR → Pi (20Hz 권장)
    $STA,<mode>,<fault_mask>,<roll_cdeg>,<pitch_cdeg>,<yaw_cdeg>,<us_l_mm>,<us_c_mm>,<us_r_mm>*<XX>\n
    mode: OpenCR가 실제로 수행 중인 모드 (1 = 턱 넘기 시퀀스 진행 중)
    fault_mask: bit0~3 = FL, FR, RL, RR 바퀴 고장
    us_*: 초음파 거리, 측정 실패는 0
XX = '$'와 '*' 사이 문자들의 XOR (16진수 2자리)

OpenCR 쪽은 cmd_timeout_s(0.5초) 동안 $CMD가 안 오면 스스로 정지해야 한다 (워치독).
"""
from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field

MODE_NORMAL, MODE_CLIMB, MODE_STOP = 0, 1, 2


def checksum(body: str) -> str:
    c = 0
    for ch in body:
        c ^= ord(ch)
    return f"{c:02X}"


def encode_cmd(v_mm_s: float, w_rad_s: float, mode: int) -> bytes:
    body = f"CMD,{int(round(v_mm_s))},{int(round(w_rad_s * 1000))},{int(mode)}"
    return f"${body}*{checksum(body)}\n".encode()


@dataclass
class OpenCRStatus:
    mode: int = MODE_NORMAL
    fault_mask: int = 0
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_rad: float = 0.0
    ultrasonic_mm: list = field(default_factory=lambda: [0, 0, 0])
    stamp: float = 0.0


def parse_status(line: str):
    line = line.strip()
    if not line.startswith("$STA") or "*" not in line:
        return None
    body, cs = line[1:].split("*", 1)
    if checksum(body) != cs.strip().upper():
        return None
    p = body.split(",")
    if len(p) != 9:
        return None
    try:
        v = [int(x) for x in p[1:]]
    except ValueError:
        return None
    return OpenCRStatus(
        mode=v[0], fault_mask=v[1], roll_deg=v[2] / 100, pitch_deg=v[3] / 100,
        yaw_rad=math.radians(v[4] / 100), ultrasonic_mm=v[5:8], stamp=time.monotonic())


class OpenCRLink:
    def __init__(self, cfg):
        import serial

        self.ser = serial.Serial(cfg.port, cfg.baud, timeout=0.05)
        self._status = OpenCRStatus()
        self._lock = threading.Lock()
        self._running = True
        self._th = threading.Thread(target=self._loop, daemon=True)
        self._th.start()

    def _loop(self):
        buf = b""
        while self._running:
            buf += self.ser.read(128)
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                st = parse_status(line.decode(errors="ignore"))
                if st:
                    with self._lock:
                        self._status = st

    def status(self) -> OpenCRStatus:
        with self._lock:
            return self._status

    def alive(self, timeout_s: float = 0.5) -> bool:
        return time.monotonic() - self.status().stamp < timeout_s

    def send(self, v_mm_s: float, w_rad_s: float, mode: int):
        self.ser.write(encode_cmd(v_mm_s, w_rad_s, mode))

    def close(self):
        try:
            self.send(0, 0, MODE_STOP)
        finally:
            self._running = False
            self._th.join(timeout=1)
            self.ser.close()
