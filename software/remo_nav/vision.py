"""USB 웹캠 바닥 분할 기반 장애물 감지 (초기 버전).

원리
  1. 화면 맨 아래 가운데 영역을 "바닥 색" 기준으로 잡는다 (로봇 바로 앞은 비어 있다고 가정).
  2. Lab 색공간에서 바닥 색과 많이 다른 픽셀 = 장애물 후보.
  3. 화면을 세로 띠 N개로 나눠, 띠마다 아래에서 위로 올라가며 처음 만나는 장애물 행을 찾는다.
  4. 카메라 높이·숙인 각도로 그 행을 지면 거리로 바꾼다 (지면 평면 가정).

라이다(150mm 높이)와 ToF(50mm 높이)에 안 보이는 낮은 턱까지 잡는 역할.
높이는 모르므로, 높이 판단은 perception.py에서 라이다·ToF와 비교해 한다.
"""
from __future__ import annotations

import math
import threading
import time

import numpy as np


class FloorVision:
    def __init__(self, cfg):
        import cv2

        self.cv2 = cv2
        self.cfg = cfg
        self.cap = cv2.VideoCapture(cfg.device)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg.height)
        self._frame = None
        self._stamp = 0.0
        self._lock = threading.Lock()
        self._running = True
        self._th = threading.Thread(target=self._loop, daemon=True)
        self._th.start()

    def _loop(self):
        while self._running:
            ok, frame = self.cap.read()
            if ok:
                with self._lock:
                    self._frame = frame
                    self._stamp = time.monotonic()
            else:
                time.sleep(0.05)

    def points(self) -> np.ndarray:
        with self._lock:
            if self._frame is None or time.monotonic() - self._stamp > 0.5:
                return np.zeros((0, 2))
            frame = self._frame.copy()
        return obstacle_points_from_frame(frame, self.cfg, self.cv2)

    def close(self):
        self._running = False
        self._th.join(timeout=1)
        self.cap.release()


def row_to_ground_mm(row: float, cy: float, f: float, cam_h: float, tilt_deg: float):
    """이미지 행 → 카메라 앞 지면 거리(mm). 지평선 위면 None."""
    ang = math.radians(tilt_deg) + math.atan((row - cy) / f)
    if ang <= math.radians(1.0):
        return None
    return cam_h / math.tan(ang)


def obstacle_points_from_frame(frame, cfg, cv2) -> np.ndarray:
    small = cv2.resize(frame, (160, 120))
    h, w = small.shape[:2]
    lab = cv2.cvtColor(cv2.GaussianBlur(small, (5, 5), 0), cv2.COLOR_BGR2LAB).astype(np.float32)

    ref = lab[h - 12:h, w // 3: 2 * w // 3].reshape(-1, 3)
    ref_mean = ref.mean(axis=0)
    diff = lab - ref_mean
    diff[..., 0] *= 0.5  # 밝기(그림자)에는 덜 민감하게
    dist = np.sqrt((diff ** 2).sum(axis=2))
    mask = (dist > cfg.floor_thresh).astype(np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

    f = (w / 2) / math.tan(math.radians(cfg.hfov_deg / 2))
    cx, cy = w / 2, h / 2
    band_w = w / cfg.bands
    pts = []
    for b in range(cfg.bands):
        c0, c1 = int(b * band_w), int((b + 1) * band_w)
        frac = mask[:, c0:c1].mean(axis=1)
        hit = None
        run = 0
        for r in range(h - 1, -1, -1):
            run = run + 1 if frac[r] > 0.5 else 0
            if run >= 3:
                hit = r + 2  # 연속 구간의 가장 아래 행 = 장애물 밑동
                break
        if hit is None:
            continue
        d = row_to_ground_mm(hit, cy, f, cfg.height_mm, cfg.tilt_deg)
        if d is None or not (cfg.min_range_mm <= d <= cfg.max_range_mm):
            continue
        bearing = -math.atan(((c0 + c1) / 2 - cx) / f)  # 왼쪽 +
        pts.append((cfg.x_mm + d, d * math.tan(bearing)))
    return np.asarray(pts, dtype=float).reshape(-1, 2)
