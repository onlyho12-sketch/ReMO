"""앞 범퍼 ToF 거리 센서 (I2C, Pi 핀 3=SDA, 5=SCL, 3.3V).

수평으로 climb_limit 높이에 장착하는 "높이 게이트" 용도:
ToF에 잡히는 장애물 = 넘을 수 있는 높이보다 높음 → 우회.
"""
from __future__ import annotations

import threading
import time


class ToFSensor:
    def __init__(self, cfg):
        self.cfg = cfg
        self._dev = self._open(cfg.driver)
        self._value = None
        self._stamp = 0.0
        self._lock = threading.Lock()
        self._running = True
        self._th = threading.Thread(target=self._loop, daemon=True)
        self._th.start()

    @staticmethod
    def _open(driver: str):
        import board  # pip install adafruit-blinka
        import busio

        i2c = busio.I2C(board.SCL, board.SDA)
        if driver == "vl53l0x":
            import adafruit_vl53l0x

            dev = adafruit_vl53l0x.VL53L0X(i2c)
            dev.measurement_timing_budget = 33000
            return ("vl53l0x", dev)
        if driver == "vl53l1x":
            import adafruit_vl53l1x

            dev = adafruit_vl53l1x.VL53L1X(i2c)
            dev.distance_mode = 2  # long
            dev.timing_budget = 50
            dev.start_ranging()
            return ("vl53l1x", dev)
        raise ValueError(f"지원하지 않는 ToF 드라이버: {driver}")

    def _read_once(self):
        kind, dev = self._dev
        if kind == "vl53l0x":
            mm = dev.range
        else:
            if not dev.data_ready:
                return self._value
            cm = dev.distance
            dev.clear_interrupt()
            mm = None if cm is None else cm * 10.0
        if mm is None or mm <= 0 or mm >= self.cfg.max_range_mm:
            return None
        return float(mm)

    def _loop(self):
        while self._running:
            try:
                v = self._read_once()
            except OSError:
                v = None
            with self._lock:
                self._value = v
                self._stamp = time.monotonic()
            time.sleep(0.02)

    def distance_mm(self):
        """측정 거리(mm). 아무것도 없거나 측정 범위 밖이면 None."""
        with self._lock:
            if time.monotonic() - self._stamp > 0.5:
                return None
            return self._value

    def close(self):
        self._running = False
        self._th.join(timeout=1)
