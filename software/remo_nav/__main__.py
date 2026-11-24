"""실행: python3 -m remo_nav            (실제 로봇, Pi에서)
       python3 -m remo_nav --sim course --plot out.png   (노트북 시뮬레이션)
"""
from __future__ import annotations

import argparse
import csv
import math
import signal
import time
from pathlib import Path

import numpy as np

from .config import load_config
from .opencr_link import MODE_STOP
from .perception import Perception
from .planner import STOP, Command, Planner


class RealBackend:
    """실제 센서들을 묶는다. 꺼진 센서는 빈 값을 돌려준다."""

    def __init__(self, cfg):
        from .opencr_link import OpenCRLink

        self.cfg = cfg
        self.opencr = OpenCRLink(cfg.opencr)
        self.lidar = self.tof = self.cam = None
        if cfg.lidar.enabled:
            from .lidar_ld19 import LD19Reader
            self.lidar = LD19Reader(cfg.lidar, cfg.robot)
        if cfg.tof.enabled:
            from .tof import ToFSensor
            self.tof = ToFSensor(cfg.tof)
        if cfg.camera.enabled:
            from .vision import FloorVision
            self.cam = FloorVision(cfg.camera)
        self._next = time.monotonic()

    def lidar_points(self):
        return self.lidar.points() if self.lidar else np.zeros((0, 2))

    def tof_mm(self):
        return self.tof.distance_mm() if self.tof else None

    def vision_points(self):
        return self.cam.points() if self.cam else np.zeros((0, 2))

    def status(self):
        return self.opencr.status()

    def alive(self):
        return self.opencr.alive()

    def send(self, v, w, mode):
        self.opencr.send(v, w, mode)

    def tick(self, dt):
        self._next += dt
        time.sleep(max(0.0, self._next - time.monotonic()))

    def close(self):
        for dev in (self.lidar, self.tof, self.cam, self.opencr):
            if dev:
                dev.close()


def run(cfg, backend, max_time=None, verbose=True, log_path=None):
    planner = Planner(cfg)
    perception = Perception(cfg)
    last_v = 0.0
    dt = 1.0 / cfg.loop_hz
    t = 0.0
    last_print = -1.0
    writer = None
    if log_path:
        Path(log_path).parent.mkdir(parents=True, exist_ok=True)
        f = open(log_path, "w", newline="", encoding="utf-8")
        writer = csv.writer(f)
        writer.writerow(["t", "state", "v", "w", "mode", "heading_deg", "front_tall", "front_any", "class",
                         "roll", "pitch", "yaw_deg", "fault", "note"])
    stop = {"flag": False}
    signal.signal(signal.SIGINT, lambda *_: stop.update(flag=True))
    try:
        while not stop["flag"]:
            st = backend.status()
            scene = perception.update(backend.lidar_points(), backend.tof_mm(), backend.vision_points(), st,
                                      last_v, dt, t)
            if not backend.alive():
                cmd = Command(0, 0, MODE_STOP, STOP, note="OpenCR 응답 없음")
            else:
                cmd = planner.step(scene, st, t)
            backend.send(cmd.v, cmd.w, cmd.mode)
            last_v = cmd.v if cmd.mode != MODE_STOP else 0.0
            if writer:
                writer.writerow([f"{t:.2f}", cmd.state, f"{cmd.v:.0f}", f"{cmd.w:.3f}", cmd.mode,
                                 f"{math.degrees(cmd.heading):.0f}", f"{min(scene.front_tall_mm, 9999):.0f}",
                                 f"{min(scene.front_any_mm, 9999):.0f}", scene.front_class,
                                 f"{st.roll_deg:.1f}", f"{st.pitch_deg:.1f}", f"{math.degrees(st.yaw_rad):.1f}",
                                 st.fault_mask, cmd.note])
            if verbose and t - last_print >= 0.5:
                print(f"[{t:6.1f}s] {cmd.state:<12} v={cmd.v:4.0f} w={cmd.w:+.2f} "
                      f"정면(넘을수없음)={min(scene.front_tall_mm, 9999):4.0f} 정면(전체)={min(scene.front_any_mm, 9999):4.0f} "
                      f"{scene.front_class:<5} {cmd.note}")
                last_print = t
            backend.tick(dt)
            t += dt
            if hasattr(backend, "done") and backend.done():
                break
            if max_time is not None and t >= max_time:
                break
    finally:
        backend.send(0, 0, MODE_STOP)
        backend.close()
        if writer:
            f.close()
    return t


def main():
    ap = argparse.ArgumentParser(description="ReMO 자율주행 — 넘을 수 있으면 넘고, 못 넘으면 돌아간다")
    ap.add_argument("--config", default=None, help="설정 파일 (기본: software/config.yaml)")
    ap.add_argument("--sim", metavar="SCENARIO", help="시뮬레이션: tall_box | low_step | mid_box | course")
    ap.add_argument("--plot", help="시뮬레이션 경로 그림 저장 경로 (.png)")
    ap.add_argument("--max-time", type=float, default=None, help="최대 실행 시간 (초)")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    cfg = load_config(args.config)
    log_path = None
    if cfg.log.enabled:
        log_path = Path(cfg.log.dir) / time.strftime("run_%Y%m%d_%H%M%S.csv")

    if args.sim:
        from .sim import SimBackend
        backend = SimBackend(cfg, args.sim)
        t = run(cfg, backend, max_time=args.max_time or 90, verbose=not args.quiet, log_path=log_path)
        print(f"\n시뮬레이션 종료 {t:.1f}s · 목표 도달={backend.done()} · 충돌={backend.collisions} · 턱 넘기={backend.climbed}")
        if args.plot:
            backend.plot(args.plot, f"{args.sim} — {t:.1f}s, 충돌 {backend.collisions}, 턱 넘기 {backend.climbed}")
    else:
        backend = RealBackend(cfg)
        print("ReMO 자율주행 시작 (Ctrl+C로 정지)")
        run(cfg, backend, max_time=args.max_time, verbose=not args.quiet, log_path=log_path)


if __name__ == "__main__":
    main()
