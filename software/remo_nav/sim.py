"""하드웨어 없이 판단 로직을 돌려 보는 2D 시뮬레이터.

장애물은 높이를 가진 직사각형(x0, y0, x1, y1, h). 센서마다 보이는 높이가 다르게 레이캐스트한다.
  라이다: h > lidar.height_mm,  ToF: h > tof.height_mm,  초음파: h > ultrasonic.height_mm,
  카메라: h > 10mm (지면 가시 범위 안만)
OpenCR 흉내: CLIMB 요청을 받으면 mode=1로 응답, 턱을 다 지나면 mode=0으로 복귀.
바퀴로 넘을 수 있는 높이: 일반 주행 15mm, 턱 넘기 모드 2e(60mm).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .opencr_link import MODE_CLIMB, MODE_NORMAL, OpenCRStatus


@dataclass
class Box:
    x0: float
    y0: float
    x1: float
    y1: float
    h: float
    name: str = ""


WALLS = [Box(-1000, 1200, 7000, 1250, 1000, "벽"), Box(-1000, -1250, 7000, -1200, 1000, "벽")]

SCENARIOS = {
    # 키 큰 상자 (라이다에 보임) → 우회
    "tall_box": [Box(1500, -220, 1900, 220, 300, "상자 300mm")],
    # 통로 전체를 가로지르는 40mm 턱 → 우회 불가, 편심으로 넘기
    "low_step": [Box(1500, -1200, 1650, 1200, 40, "턱 40mm")],
    # 80mm 상자: 라이다(150mm)에는 안 보이고 ToF(50mm)에 잡힘 → 넘을 수 없으니 우회
    "mid_box": [Box(1500, -200, 1800, 200, 80, "상자 80mm")],
    # 종합: 낮은 턱 → 키 큰 상자 → 중간 상자
    "course": [
        Box(1000, -1200, 1120, 1200, 35, "턱 35mm"),
        Box(2300, -150, 2700, 350, 400, "상자 400mm"),
        Box(3900, -300, 4200, 100, 90, "상자 90mm"),
    ],
}


def _ray_box(ox, oy, dx, dy, b: Box):
    """레이와 축정렬 사각형 교차 거리 (없으면 None)."""
    tmin, tmax = -math.inf, math.inf
    for o, d, lo, hi in ((ox, dx, b.x0, b.x1), (oy, dy, b.y0, b.y1)):
        if abs(d) < 1e-12:
            if o < lo or o > hi:
                return None
        else:
            t1, t2 = (lo - o) / d, (hi - o) / d
            tmin, tmax = max(tmin, min(t1, t2)), min(tmax, max(t1, t2))
    if tmax < max(tmin, 0):
        return None
    return tmin if tmin > 0 else None


class SimBackend:
    def __init__(self, cfg, scenario: str = "course", goal_x: float = 5000.0):
        self.cfg = cfg
        self.boxes = SCENARIOS[scenario] + WALLS
        self.goal_x = goal_x
        self.x, self.y, self.yaw = 0.0, 0.0, 0.0
        self.t = 0.0
        self.cmd = (0.0, 0.0, MODE_NORMAL)
        self.mode = MODE_NORMAL
        self.climb_touched = False
        self.climb_t0 = 0.0
        self.climb_left_t = None
        self.collisions = 0
        self._in_collision = False
        self.climbed = 0
        self.traj = []
        L, W = cfg.robot.length_mm / 2, cfg.robot.width_mm / 2
        xs = np.linspace(-L, L, 9)
        ys = np.linspace(-W, W, 7)
        self.body = np.array([(x, y) for x in xs for y in ys
                              if abs(x) in (L,) or abs(y) in (W,)] + [(0, 0)])

    # ---------------- 좌표 변환 ----------------
    def _world(self, x, y):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return self.x + c * x - s * y, self.y + s * x + c * y

    def _cast(self, x, y, ang, min_h, max_r):
        ox, oy = self._world(x, y)
        a = self.yaw + ang
        dx, dy = math.cos(a), math.sin(a)
        best = None
        for b in self.boxes:
            if b.h <= min_h:
                continue
            t = _ray_box(ox, oy, dx, dy, b)
            if t is not None and t <= max_r and (best is None or t < best):
                best = t
        return best

    # ---------------- 센서 ----------------
    def lidar_points(self):
        c = self.cfg.lidar
        pts = []
        for deg in range(0, 360, 1):
            a = math.radians(deg)
            d = self._cast(c.x_mm, c.y_mm, a, c.height_mm, c.max_range_mm)
            if d is not None:
                pts.append((c.x_mm + d * math.cos(a), c.y_mm + d * math.sin(a)))
        return np.asarray(pts, float).reshape(-1, 2)

    def tof_mm(self):
        c = self.cfg.tof
        return self._cast(c.x_mm, c.y_mm, 0.0, c.height_mm, c.max_range_mm) if c.enabled else None

    def vision_points(self):
        c = self.cfg.camera
        if not c.enabled:
            return np.zeros((0, 2))
        pts = []
        half = math.radians(c.hfov_deg / 2)
        for a in np.linspace(-half, half, c.bands):
            d = self._cast(c.x_mm, 0.0, a, 10.0, c.max_range_mm / max(math.cos(a), 0.3))
            if d is not None and d * math.cos(a) >= c.min_range_mm:
                pts.append((c.x_mm + d * math.cos(a), d * math.sin(a)))
        return np.asarray(pts, float).reshape(-1, 2)

    def status(self):
        us = self.cfg.opencr.ultrasonic
        vals = []
        for x, y, a in us.sensors:
            d = self._cast(x, y, math.radians(a), us.height_mm, us.max_range_mm)
            vals.append(int(d) if d is not None else 0)
        pitch = 8.0 if (self.mode == MODE_CLIMB and self._overlap_low()) else 0.0
        return OpenCRStatus(mode=self.mode, fault_mask=0, roll_deg=0.0, pitch_deg=pitch,
                            yaw_rad=self.yaw, ultrasonic_mm=vals, stamp=self.t)

    def alive(self):
        return True

    # ---------------- 구동 ----------------
    def send(self, v, w, mode):
        self.cmd = (v, w, mode)

    def _body_world(self, x, y, yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        return np.column_stack([x + c * self.body[:, 0] - s * self.body[:, 1],
                                y + s * self.body[:, 0] + c * self.body[:, 1]])

    def _hits(self, pts, max_h):
        for b in self.boxes:
            if b.h > max_h:
                inside = (pts[:, 0] >= b.x0) & (pts[:, 0] <= b.x1) & (pts[:, 1] >= b.y0) & (pts[:, 1] <= b.y1)
                if inside.any():
                    return b
        return None

    def _overlap_low(self):
        pts = self._body_world(self.x, self.y, self.yaw)
        for b in self.boxes:
            if 10 < b.h <= 2 * self.cfg.robot.eccentric_offset_mm:
                inside = (pts[:, 0] >= b.x0) & (pts[:, 0] <= b.x1) & (pts[:, 1] >= b.y0) & (pts[:, 1] <= b.y1)
                if inside.any():
                    return True
        return False

    def tick(self, dt):
        v, w, mode = self.cmd
        # OpenCR 흉내: 턱 넘기 시퀀스
        if mode == MODE_CLIMB and self.mode == MODE_NORMAL:
            self.mode, self.climb_t0, self.climb_touched, self.climb_left_t = MODE_CLIMB, self.t, False, None
        if self.mode == MODE_CLIMB:
            if self._overlap_low():
                self.climb_touched, self.climb_left_t = True, None
            elif self.climb_touched:
                self.climb_left_t = self.climb_left_t if self.climb_left_t is not None else self.t
                if self.t - self.climb_left_t > 0.3:
                    self.mode = MODE_NORMAL
                    self.climbed += 1
            if self.t - self.climb_t0 > 14.0:
                self.mode = MODE_NORMAL
        if mode == 2:
            v, w = 0.0, 0.0

        nyaw = self.yaw + w * dt
        nx = self.x + v * dt * math.cos(nyaw)
        ny = self.y + v * dt * math.sin(nyaw)
        can_h = 2 * self.cfg.robot.eccentric_offset_mm if self.mode == MODE_CLIMB else 15.0
        hit = self._hits(self._body_world(nx, ny, nyaw), can_h)
        if hit is None:
            self.x, self.y, self.yaw = nx, ny, nyaw
            self._in_collision = False
        elif not self._in_collision:
            self.collisions += 1
            self._in_collision = True
        self.t += dt
        self.traj.append((self.t, self.x, self.y, self.yaw, self.mode))

    def done(self):
        return self.x >= self.goal_x

    def close(self):
        pass

    def plot(self, path, title=""):
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Rectangle

        plt.rcParams["font.family"] = ["Noto Sans CJK KR", "NanumGothic", "DejaVu Sans"]
        fig, ax = plt.subplots(figsize=(11, 4))
        for b in self.boxes:
            if b.name == "벽":
                continue
            col = "#d32f2f" if b.h > self.cfg.robot.climb_limit_mm else "#f5b800"
            ax.add_patch(Rectangle((b.x0, b.y0), b.x1 - b.x0, b.y1 - b.y0, color=col, alpha=0.55))
            ax.text((b.x0 + b.x1) / 2, b.y1 + 60, b.name, ha="center", fontsize=9)
        tr = np.array(self.traj)
        if len(tr):
            climb = tr[:, 4] == MODE_CLIMB
            ax.plot(tr[:, 1], tr[:, 2], color="#1e88e5", lw=2, label="주행 경로")
            ax.plot(tr[climb, 1], tr[climb, 2], ".", color="#8e24aa", ms=4, label="턱 넘기 모드")
        ax.axvline(self.goal_x, color="#2e7d32", ls="--", lw=1, label="목표선")
        ax.set_xlim(-300, self.goal_x + 300)
        ax.set_ylim(-1250, 1250)
        ax.set_aspect("equal")
        ax.set_xlabel("x (mm)")
        ax.set_ylabel("y (mm)")
        ax.set_title(title)
        ax.legend(loc="lower right", fontsize=8)
        fig.tight_layout()
        fig.savefig(path, dpi=130)
        plt.close(fig)
