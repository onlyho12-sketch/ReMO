"""LDROBOT D500 (LD19 계열) 2D 라이다 드라이버.

패킷 47바이트 (230400 baud):
  0x54 | 0x2C | speed(2) | start_angle(2) | 12 x [distance(2) intensity(1)] | end_angle(2) | timestamp(2) | crc8(1)
각도 단위 0.01°, 거리 단위 mm, 리틀 엔디언. CRC8 다항식 0x4D.
"""
from __future__ import annotations

import math
import struct
import threading
import time

import numpy as np

HEADER = 0x54
VER_LEN = 0x2C
PKT_LEN = 47
POINTS_PER_PKT = 12
_PKT_FMT = "<BBHH" + "HB" * POINTS_PER_PKT + "HHB"


def _make_crc_table(poly: int = 0x4D) -> list[int]:
    table = []
    for i in range(256):
        crc = i
        for _ in range(8):
            crc = ((crc << 1) ^ poly) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
        table.append(crc)
    return table


CRC_TABLE = _make_crc_table()


def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc = CRC_TABLE[(crc ^ b) & 0xFF]
    return crc


def parse_packet(pkt: bytes):
    """47바이트 패킷 → (start_deg, [(angle_deg, dist_mm, intensity), ...]) 또는 CRC 오류 시 None."""
    if len(pkt) != PKT_LEN or pkt[0] != HEADER or pkt[1] != VER_LEN:
        return None
    if crc8(pkt[:-1]) != pkt[-1]:
        return None
    f = struct.unpack(_PKT_FMT, pkt)
    start = f[3] / 100.0
    end = f[4 + 2 * POINTS_PER_PKT] / 100.0
    span = (end - start) % 360.0
    step = span / (POINTS_PER_PKT - 1)
    pts = []
    for i in range(POINTS_PER_PKT):
        dist = f[4 + 2 * i]
        inten = f[5 + 2 * i]
        pts.append(((start + step * i) % 360.0, dist, inten))
    return start, pts


class LD19Reader:
    """백그라운드 스레드에서 시리얼을 읽고, 한 바퀴(360°)가 모이면 최신 스캔으로 저장한다."""

    def __init__(self, cfg, robot_cfg):
        import serial  # 하드웨어에서만 필요

        self.cfg = cfg
        self.half_len = robot_cfg.length_mm / 2 + cfg.self_filter_mm
        self.half_w = robot_cfg.width_mm / 2 + cfg.self_filter_mm
        self.ser = serial.Serial(cfg.port, cfg.baud, timeout=0.1)
        self._lock = threading.Lock()
        self._latest = np.zeros((0, 2))
        self._stamp = 0.0
        self._running = True
        self._th = threading.Thread(target=self._loop, daemon=True)
        self._th.start()

    def _to_robot(self, angle_deg: float, dist: float):
        a = -angle_deg if self.cfg.clockwise else angle_deg
        th = math.radians(a + self.cfg.angle_offset_deg)
        return self.cfg.x_mm + dist * math.cos(th), self.cfg.y_mm + dist * math.sin(th)

    def _loop(self):
        buf = bytearray()
        scan = []
        last_start = None
        while self._running:
            buf += self.ser.read(256)
            while len(buf) >= PKT_LEN:
                i = buf.find(bytes([HEADER, VER_LEN]))
                if i < 0:
                    buf.clear()
                    break
                if len(buf) - i < PKT_LEN:
                    del buf[:i]
                    break
                pkt = bytes(buf[i:i + PKT_LEN])
                res = parse_packet(pkt)
                if res is None:
                    del buf[:i + 1]
                    continue
                del buf[:i + PKT_LEN]
                start, pts = res
                if last_start is not None and start < last_start:  # 0° 통과 → 한 바퀴 완료
                    self._publish(scan)
                    scan = []
                last_start = start
                for ang, dist, inten in pts:
                    if self.cfg.min_range_mm <= dist <= self.cfg.max_range_mm and inten > 0:
                        scan.append(self._to_robot(ang, dist))

    def _publish(self, scan):
        arr = np.asarray(scan, dtype=float).reshape(-1, 2)
        if arr.size:
            inside = (np.abs(arr[:, 0]) < self.half_len) & (np.abs(arr[:, 1]) < self.half_w)
            arr = arr[~inside]
        with self._lock:
            self._latest = arr
            self._stamp = time.monotonic()

    def points(self) -> np.ndarray:
        """로봇 좌표계(전방 +x, 좌측 +y, mm) 점 배열. 0.5초 넘게 갱신이 없으면 빈 배열."""
        with self._lock:
            if time.monotonic() - self._stamp > 0.5:
                return np.zeros((0, 2))
            return self._latest.copy()

    def close(self):
        self._running = False
        self._th.join(timeout=1)
        self.ser.close()
