"""센서 융합 + 높이 판단.

센서마다 "보이는 높이"가 달라서, 같은 장애물이 어떤 센서에 잡히는지로 높이를 가른다.

    라이다 (스캔면 150mm)  : 150mm보다 높은 것만 보임 → 키 큰 장애물
    ToF   (수평, 50mm)     : 50mm(= 넘을 수 있는 한계)보다 높은 것만 보임 → 넘을 수 없음
    카메라 · 초음파         : 낮은 턱까지 다 보임 (높이는 모름)

정면 중앙(ToF가 보는 곳)에 뭔가 있는데 ToF·라이다에는 안 잡히면
→ 50mm 이하의 낮은 턱 → 편심 모드로 넘는다.
ToF가 못 보는 곳(통로 가장자리)에만 있는 장애물은 높이를 모르니 넘을 수 없다고 본다.

단기 메모리
  ToF는 한 줄만 보고 카메라는 범퍼 바로 앞(150mm 이내)이 사각지대라서,
  한 번 본 장애물은 이동 거리 추정(오도메트리)으로 몇 초 동안 기억한다.
  ToF가 잡은 점 근처의 카메라 점은 "넘을 수 없음"으로 승격해 장애물 폭 전체를 기억한다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

CLEAR, LOW, TALL = "clear", "low", "tall"
FAR = 1e9


@dataclass
class Scene:
    tall_pts: np.ndarray   # 넘을 수 없는 높이의 점 (라이다 + ToF + 기억)
    all_pts: np.ndarray    # 모든 장애물 점 (+ 카메라 + 초음파 + 기억)
    front_tall_mm: float   # 정면 통로에서 넘을 수 없는 장애물까지 거리 (범퍼 기준)
    front_any_mm: float    # 정면 통로에서 가장 가까운 장애물까지 거리
    front_class: str       # clear / low / tall


def corridor_clearance(pts: np.ndarray, heading: float, half_w: float, front_offset: float) -> float:
    """heading 방향으로 차체 폭만큼의 통로를 그었을 때, 범퍼에서 첫 장애물까지 거리."""
    if pts.size == 0:
        return FAR
    c, s = math.cos(heading), math.sin(heading)
    xr = pts[:, 0] * c + pts[:, 1] * s
    yr = -pts[:, 0] * s + pts[:, 1] * c
    m = (xr > 0) & (np.abs(yr) <= half_w)
    if not m.any():
        return FAR
    return max(0.0, float(xr[m].min()) - front_offset)


class Odometry:
    """명령 속도 + IMU yaw로 위치를 적분 (단기 메모리용, 수 초 정도만 믿을 만함)."""

    def __init__(self):
        self.x = self.y = 0.0
        self.yaw = 0.0

    def update(self, v_mm_s: float, yaw: float, dt: float):
        self.yaw = yaw
        self.x += v_mm_s * dt * math.cos(yaw)
        self.y += v_mm_s * dt * math.sin(yaw)

    def to_world(self, pts):
        if pts.size == 0:
            return pts.reshape(-1, 2)
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return np.column_stack([self.x + c * pts[:, 0] - s * pts[:, 1], self.y + s * pts[:, 0] + c * pts[:, 1]])

    def to_robot(self, pts):
        if pts.size == 0:
            return pts.reshape(-1, 2)
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        dx, dy = pts[:, 0] - self.x, pts[:, 1] - self.y
        return np.column_stack([c * dx + s * dy, -s * dx + c * dy])


class _Memory:
    def __init__(self, ttl_s: float, max_pts: int = 4000):
        self.ttl = ttl_s
        self.max_pts = max_pts
        self.pts = np.zeros((0, 2))
        self.t = np.zeros(0)

    def add(self, world_pts, now):
        if world_pts.size == 0:
            return
        self.pts = np.vstack([self.pts, world_pts])[-self.max_pts:]
        self.t = np.concatenate([self.t, np.full(len(world_pts), now)])[-self.max_pts:]

    def prune(self, now, odo: Odometry, max_dist=3000.0):
        if self.pts.size == 0:
            return
        keep = (now - self.t <= self.ttl) & (np.hypot(self.pts[:, 0] - odo.x, self.pts[:, 1] - odo.y) < max_dist)
        self.pts, self.t = self.pts[keep], self.t[keep]


class Perception:
    def __init__(self, cfg):
        self.cfg = cfg
        self.odo = Odometry()
        self.tall_mem = _Memory(ttl_s=10.0)
        self.any_mem = _Memory(ttl_s=3.0)
        self.half_w = cfg.robot.width_mm / 2 + cfg.safety.side_margin_mm
        self.half_l = cfg.robot.length_mm / 2
        self.tof_half_fov = math.radians(getattr(cfg.tof, "fov_deg", 25) / 2)
        self.promote_r = 250.0

    def update(self, lidar_pts, tof_mm, vision_pts, status, v_cmd, dt, now) -> Scene:
        cfg = self.cfg
        self.odo.update(v_cmd, status.yaw_rad if status else self.odo.yaw, dt)
        lidar_pts = np.asarray(lidar_pts, float).reshape(-1, 2)
        vision_pts = np.asarray(vision_pts, float).reshape(-1, 2)

        # ---- 초음파 → 점 ----
        us_pts = []
        us = cfg.opencr.ultrasonic
        if us.enabled and status is not None:
            for (x, y, a), d in zip(us.sensors, status.ultrasonic_mm):
                if 0 < d < us.max_range_mm:
                    r = math.radians(a)
                    us_pts.append((x + d * math.cos(r), y + d * math.sin(r)))
        us_pts = np.asarray(us_pts, float).reshape(-1, 2)

        # ---- ToF 점 + 근처 카메라 점 승격 → 넘을 수 없음 기억 ----
        tof_pt = np.zeros((0, 2))
        if tof_mm is not None:
            tof_pt = np.array([[cfg.tof.x_mm + tof_mm, cfg.tof.y_mm]])
        self.tall_mem.prune(now, self.odo)
        self.any_mem.prune(now, self.odo)
        seeds = np.vstack([tof_pt, self.odo.to_robot(self.tall_mem.pts)])
        promoted = np.zeros((0, 2))
        if seeds.size and vision_pts.size:
            d = np.hypot(vision_pts[:, None, 0] - seeds[None, :, 0], vision_pts[:, None, 1] - seeds[None, :, 1])
            promoted = vision_pts[(d < self.promote_r).any(axis=1)]
        self.tall_mem.add(self.odo.to_world(np.vstack([tof_pt, promoted])), now)
        self.any_mem.add(self.odo.to_world(np.vstack([vision_pts, us_pts])), now)

        mem_tall = self._outside_body(self.odo.to_robot(self.tall_mem.pts))
        mem_any = self._outside_body(self.odo.to_robot(self.any_mem.pts))
        tall_pts = np.vstack([lidar_pts, tof_pt, mem_tall])
        all_pts = np.vstack([tall_pts, vision_pts, us_pts, mem_any])

        ft = corridor_clearance(tall_pts, 0.0, self.half_w, self.half_l)
        fa = corridor_clearance(all_pts, 0.0, self.half_w, self.half_l)
        cls = self._classify(all_pts, ft, fa)
        return Scene(tall_pts, all_pts, ft, fa, cls)

    def _outside_body(self, pts):
        """기억 속 점 중 차체 아래에 깔린 것(이미 넘고 있는 턱)은 제외."""
        if pts.size == 0:
            return pts
        inside = (np.abs(pts[:, 0]) < self.half_l) & (np.abs(pts[:, 1]) < self.cfg.robot.width_mm / 2)
        return pts[~inside]

    def _classify(self, all_pts, ft, fa):
        c = self.cfg
        if fa >= c.safety.slow_distance_mm:
            return CLEAR
        if ft - fa <= c.decision.low_gap_mm:
            return TALL
        # 가장 가까운 장애물이 ToF 빔 안(정면 중앙)에도 있어야 높이를 확인한 것
        x_hit = fa + self.half_l
        near = all_pts[(all_pts[:, 0] > 0) & (all_pts[:, 0] <= x_hit + 80) & (np.abs(all_pts[:, 1]) <= self.half_w)]
        if near.size == 0:
            return TALL
        beam = np.maximum(near[:, 0] - c.tof.x_mm, 0) * math.tan(self.tof_half_fov) + 20
        if (np.abs(near[:, 1] - c.tof.y_mm) <= beam).any():
            return LOW
        return TALL


def build_scene(cfg, lidar_pts, tof_mm, vision_pts, status) -> Scene:
    """메모리 없이 한 프레임만으로 판단 (테스트용)."""
    p = Perception(cfg)
    return p.update(lidar_pts, tof_mm, vision_pts, status, 0.0, 0.0, 0.0)
