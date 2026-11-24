"""판단 · 경로 결정 (상태 기계 + 반응형 방향 선택).

상태
  CRUISE        목표 방향(출발 시 방향)으로 주행. 막히면 비어 있는 방향으로 틀어서 우회
  LOW_APPROACH  정면이 낮은 턱 → 정렬 유지하며 감속 접근
  CLIMB         OpenCR에 턱 넘기 모드 요청 (편심 시퀀스는 OpenCR가 수행)
  ESCAPE        앞쪽이 전부 막힘 → 제자리 회전으로 빈 방향 찾기
  STOP          기울기 초과 · 턱 넘기 실패 · OpenCR 끊김 → 정지 후 2초 뒤 재시도

우회는 매 프레임 후보 방향(-90°~+90°, 10° 간격)마다 차체 폭 통로가 lookahead만큼
비어 있는지 보고, 목표 방향에 가장 가까운 빈 방향을 고른다. 장애물을 지나면
목표 방향 통로가 다시 비므로 자연스럽게 원래 방향으로 돌아온다.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

import numpy as np

from .opencr_link import MODE_CLIMB, MODE_NORMAL, MODE_STOP
from .perception import FAR, LOW, corridor_clearance

CRUISE, LOW_APPROACH, CLIMB, ESCAPE, STOP = "CRUISE", "LOW_APPROACH", "CLIMB", "ESCAPE", "STOP"


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


@dataclass
class Command:
    v: float           # mm/s
    w: float           # rad/s (반시계 +)
    mode: int
    state: str
    heading: float = 0.0  # 선택한 상대 방향 (rad)
    note: str = ""


class Planner:
    def __init__(self, cfg):
        self.cfg = cfg
        self.state = CRUISE
        self.goal_yaw = None
        self.prev_h = 0.0
        self.cls_hist = deque(maxlen=cfg.decision.confirm_frames + 2)
        self.t_state = 0.0
        self.climb_acked = False
        self.escape_dir = 1.0
        self.half_w = cfg.robot.width_mm / 2 + cfg.safety.side_margin_mm
        self.half_l = cfg.robot.length_mm / 2
        self.cands = np.radians(np.arange(-90, 91, 10))

    # ------------------------------------------------------------------
    def _set(self, state, now):
        if state != self.state:
            self.state = state
            self.t_state = now
            if state != CLIMB:
                self.climb_acked = False

    def _turn(self, h):
        m = self.cfg.motion
        return float(np.clip(m.heading_gain * h, -m.max_turn_rate, m.max_turn_rate))

    def _speed_cap(self, st):
        m = self.cfg.motion
        return m.fault_speed if st.fault_mask else m.cruise_speed

    # ------------------------------------------------------------------
    def step(self, scene, st, now: float) -> Command:
        c = self.cfg
        if self.goal_yaw is None:
            self.goal_yaw = st.yaw_rad
        goal_rel = wrap(self.goal_yaw - st.yaw_rad)

        if self.state == STOP:
            if now - self.t_state < 2.0:
                return Command(0, 0, MODE_STOP, STOP, note="정지 대기")
            self.cls_hist.clear()
            self._set(CRUISE, now)

        if self.state == CLIMB:
            return self._climb(scene, st, now, goal_rel)

        tilt = max(abs(st.roll_deg), abs(st.pitch_deg))
        if tilt > c.safety.max_tilt_deg:
            self._set(STOP, now)
            return Command(0, 0, MODE_STOP, STOP, note=f"기울기 {tilt:.1f}°")

        # ---- 낮은 턱 판단 ----
        self.cls_hist.append(scene.front_class)
        confirmed = sum(x == LOW for x in self.cls_hist) >= c.decision.confirm_frames
        aligned = abs(goal_rel) < math.radians(c.decision.goal_align_deg)
        can_climb = c.decision.climb_enabled and st.fault_mask == 0
        if scene.front_class == LOW and aligned and can_climb and scene.front_tall_mm >= c.safety.stop_distance_mm:
            if confirmed and scene.front_any_mm <= c.decision.climb_trigger_mm:
                self._set(CLIMB, now)
                return self._climb(scene, st, now, goal_rel)
            self._set(LOW_APPROACH, now)
            return Command(c.motion.slow_speed, self._turn(goal_rel), MODE_NORMAL, LOW_APPROACH,
                           goal_rel, f"낮은 턱 {scene.front_any_mm:.0f}mm 앞")

        # ---- 우회 (반응형 방향 선택) ----
        h, clr = self._choose_heading(scene.all_pts, goal_rel)
        if h is None:
            if self.state != ESCAPE:
                self.escape_dir = self._escape_side(scene.all_pts)
            self._set(ESCAPE, now)
            return Command(0, self.escape_dir * c.motion.max_turn_rate * 0.6, MODE_NORMAL, ESCAPE,
                           note="앞이 모두 막힘 → 제자리 회전")

        self._set(CRUISE, now)
        self.prev_h = h
        s = c.safety
        v = self._speed_cap(st) * float(np.clip((clr - s.stop_distance_mm) / (s.slow_distance_mm - s.stop_distance_mm), 0, 1))
        if abs(h) > math.radians(c.motion.turn_in_place_deg):
            v = 0.0
        else:
            v *= math.cos(h)
            v = max(v, c.motion.slow_speed * 0.5) if clr > s.stop_distance_mm else 0.0
        note = "직진" if abs(h - goal_rel) < 1e-3 else f"우회 {math.degrees(h):+.0f}°"
        return Command(v, self._turn(h), MODE_NORMAL, CRUISE, h, note)

    # ------------------------------------------------------------------
    def _choose_heading(self, pts, goal_rel):
        c = self.cfg
        need = c.safety.lookahead_mm
        cands = np.append(self.cands, np.clip(goal_rel, -math.pi / 2, math.pi / 2))
        best, best_cost, best_clr = None, FAR, 0.0
        fallback, fb_clr = None, 0.0
        for h in cands:
            clr = corridor_clearance(pts, h, self.half_w, self.half_l)
            if clr > fb_clr:
                fallback, fb_clr = h, clr
            if clr < need:
                continue
            cost = abs(wrap(h - goal_rel)) + 0.3 * abs(h - self.prev_h)
            if cost < best_cost:
                best, best_cost, best_clr = h, cost, clr
        if best is not None:
            return float(best), best_clr
        if fb_clr > c.safety.stop_distance_mm * 2:
            return float(fallback), fb_clr
        return None, 0.0

    def _escape_side(self, pts):
        if pts.size == 0:
            return 1.0
        ang = np.arctan2(pts[:, 1], pts[:, 0])
        rng = np.hypot(pts[:, 0], pts[:, 1])
        left = rng[(ang > 0) & (ang < math.pi * 0.75)]
        right = rng[(ang < 0) & (ang > -math.pi * 0.75)]
        lmin = left.min() if left.size else FAR
        rmin = right.min() if right.size else FAR
        return 1.0 if lmin >= rmin else -1.0

    def _climb(self, scene, st, now, goal_rel):
        c = self.cfg
        if st.mode == MODE_CLIMB:
            self.climb_acked = True
        elapsed = now - self.t_state
        if self.climb_acked and st.mode == MODE_NORMAL:
            self.cls_hist.clear()
            self._set(CRUISE, now)
            return Command(c.motion.slow_speed, self._turn(goal_rel), MODE_NORMAL, CRUISE, note="턱 넘기 완료")
        if elapsed > c.decision.climb_max_s:
            self._set(STOP, now)
            return Command(0, 0, MODE_STOP, STOP, note="턱 넘기 시간 초과")
        if scene.front_tall_mm < c.safety.stop_distance_mm * 0.5:
            self._set(STOP, now)
            return Command(0, 0, MODE_STOP, STOP, note="턱 넘는 중 앞에 장애물")
        return Command(c.motion.climb_speed, self._turn(goal_rel) * 0.5, MODE_CLIMB, CLIMB, note="턱 넘기")
