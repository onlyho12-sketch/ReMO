"""시연용 애니메이션: 위에서 본 지도 + 센서 판단 + 옆에서 본 높이.

  python -m remo_nav --sim course --live            창으로 실시간 재생
  python -m remo_nav --sim course --video demo.mp4  동영상 저장 (.mp4는 ffmpeg 필요, 없으면 .gif)
"""
from __future__ import annotations

import math
import shutil

import numpy as np

from .opencr_link import MODE_CLIMB, MODE_STOP
from .perception import FAR, LOW, TALL, Perception, corridor_clearance
from .planner import CLIMB, ESCAPE, LOW_APPROACH, STOP, Planner
from .sim import SimBackend, setup_korean_font

SCEN_DESC = {
    "tall_box": "높이 300mm 상자 → 우회",
    "low_step": "길을 다 막은 40mm 턱 → 편심 모드로 넘기",
    "mid_box": "높이 80mm 상자 (라이다엔 안 보임) → ToF로 감지해 우회",
    "course": "35mm 턱 → 400mm 상자 → 90mm 상자",
}

STATE_LABEL = {
    LOW_APPROACH: ("낮은 턱 감지 → 감속 접근", "#f59e0b"),
    CLIMB: ("편심 모드로 턱 넘는 중", "#8e24aa"),
    ESCAPE: ("앞이 막힘 → 제자리 회전", "#d32f2f"),
    STOP: ("정지", "#d32f2f"),
}

C_LOW, C_TALL, C_ROBOT, C_PATH = "#f5b800", "#d32f2f", "#1f4fa6", "#1e88e5"


class Demo:
    def __init__(self, cfg, scenario):
        self.cfg = cfg
        self.scenario = scenario
        self.sim = SimBackend(cfg, scenario)
        self.per = Perception(cfg)
        self.plan = Planner(cfg)
        self.dt = 1.0 / cfg.loop_hz
        self.t = 0.0
        self.last_v = 0.0
        self.snap = None
        self.half_w = cfg.robot.width_mm / 2 + cfg.safety.side_margin_mm
        self.half_l = cfg.robot.length_mm / 2

    def step(self):
        b = self.sim
        st = b.status()
        lidar, tof, vis = b.lidar_points(), b.tof_mm(), b.vision_points()
        scene = self.per.update(lidar, tof, vis, st, self.last_v, self.dt, self.t)
        cmd = self.plan.step(scene, st, self.t)
        b.send(cmd.v, cmd.w, cmd.mode)
        self.last_v = cmd.v if cmd.mode != MODE_STOP else 0.0
        self.snap = dict(
            lidar=lidar, tof=tof, vis=vis, scene=scene, cmd=cmd,
            lidar_front=corridor_clearance(lidar, 0, self.half_w, self.half_l),
            cam_front=corridor_clearance(vis, 0, self.half_w, self.half_l),
            pose=(b.x, b.y, b.yaw))
        b.tick(self.dt)
        self.t += self.dt

    def done(self):
        return self.sim.done() or self.t > 90


def _to_world(pose, pts):
    if len(pts) == 0:
        return np.zeros((0, 2))
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return np.column_stack([x + c * pts[:, 0] - s * pts[:, 1], y + s * pts[:, 0] + c * pts[:, 1]])


def _dist_text(d):
    return "안 보임" if d is None or d >= FAR / 2 else f"{d:,.0f}mm 앞"


def animate(cfg, scenario="course", live=False, video=None, skip=2):
    import matplotlib

    if not live:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import animation
    from matplotlib.patches import Polygon, Rectangle

    setup_korean_font(plt)
    demo = Demo(cfg, scenario)
    sim = demo.sim
    lim = cfg.robot.climb_limit_mm

    fig = plt.figure(figsize=(13, 7.6), facecolor="white")
    gs = fig.add_gridspec(2, 2, height_ratios=[1.25, 1], width_ratios=[1.05, 1], hspace=0.28, wspace=0.12,
                          left=0.05, right=0.98, top=0.92, bottom=0.06)
    ax_map = fig.add_subplot(gs[0, :])
    ax_txt = fig.add_subplot(gs[1, 0])
    ax_side = fig.add_subplot(gs[1, 1])
    fig.suptitle(f"ReMO 자율주행 시뮬레이션 — {SCEN_DESC.get(scenario, scenario)}", fontsize=15, fontweight="bold")

    def draw_map(s):
        ax = ax_map
        ax.cla()
        for bx in sim.boxes:
            if bx.name == "벽":
                continue
            col = C_LOW if bx.h <= lim else C_TALL
            ax.add_patch(Rectangle((bx.x0, bx.y0), bx.x1 - bx.x0, bx.y1 - bx.y0, color=col, alpha=0.55, lw=0))
            ax.text((bx.x0 + bx.x1) / 2, min(bx.y1 + 70, 1020), bx.name, ha="center", fontsize=10,
                    bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="none", alpha=0.8))
        ax.axhspan(1200, 1300, color="#9e9e9e")
        ax.axhspan(-1300, -1200, color="#9e9e9e")
        ax.axvline(sim.goal_x, color="#2e7d32", ls="--", lw=1.2)
        ax.text(sim.goal_x - 40, -1120, "목표선", color="#2e7d32", ha="right", fontsize=10)
        tr = np.array(sim.traj) if sim.traj else np.zeros((0, 5))
        if len(tr):
            ax.plot(tr[:, 1], tr[:, 2], color=C_PATH, lw=2)
            cl = tr[:, 4] == MODE_CLIMB
            ax.plot(tr[cl, 1], tr[cl, 2], ".", color="#8e24aa", ms=4)
        pose = s["pose"]
        x, y, yaw = pose
        c = cfg.camera
        half = math.radians(c.hfov_deg / 2)
        wedge = np.array([[c.x_mm, 0]] + [[c.x_mm + c.max_range_mm * math.cos(a), c.max_range_mm * math.sin(a)]
                                           for a in np.linspace(-half, half, 12)])
        ax.add_patch(Polygon(_to_world(pose, wedge), color="#64b5f6", alpha=0.12, lw=0))
        lw = _to_world(pose, s["lidar"][::2])
        if len(lw):
            ax.plot(lw[:, 0], lw[:, 1], ".", color="#c62828", ms=3)
        vw = _to_world(pose, s["vis"])
        if len(vw):
            ax.plot(vw[:, 0], vw[:, 1], "x", color="#1565c0", ms=6, mew=1.6)
        tof_end = (s["tof"] if s["tof"] is not None else cfg.tof.max_range_mm)
        tw = _to_world(pose, np.array([[cfg.tof.x_mm, 0], [cfg.tof.x_mm + tof_end, 0]]))
        ax.plot(tw[:, 0], tw[:, 1], color="#ef6c00", lw=2.2 if s["tof"] is not None else 1.0,
                ls="-" if s["tof"] is not None else ":")
        L, W = cfg.robot.length_mm / 2, cfg.robot.width_mm / 2
        body = _to_world(pose, np.array([[L, W], [L, -W], [-L, -W], [-L, W]]))
        col = "#8e24aa" if s["cmd"].state == CLIMB else C_ROBOT
        ax.add_patch(Polygon(body, color=col, alpha=0.95))
        nose = _to_world(pose, np.array([[0, 0], [L + 60, 0]]))
        ax.annotate("", xy=nose[1], xytext=nose[0], arrowprops=dict(arrowstyle="->", color="white", lw=2))
        ax.set_xlim(-300, sim.goal_x + 300)
        ax.set_ylim(-1300, 1300)
        ax.set_aspect("equal")
        ax.set_xticks(range(0, int(sim.goal_x) + 1, 1000))
        ax.set_xticklabels([f"{v/1000:.0f}m" for v in range(0, int(sim.goal_x) + 1, 1000)])
        ax.set_yticks([])
        ax.set_title("위에서 본 모습", loc="left", fontsize=11)
        handles = [
            plt.Line2D([], [], color=C_LOW, lw=8, alpha=0.6, label=f"넘을 수 있음 (≤{lim}mm)"),
            plt.Line2D([], [], color=C_TALL, lw=8, alpha=0.6, label=f"넘을 수 없음 (>{lim}mm)"),
            plt.Line2D([], [], color="#c62828", marker=".", ls="", label="라이다가 본 점"),
            plt.Line2D([], [], color="#1565c0", marker="x", ls="", label="카메라가 본 물체"),
            plt.Line2D([], [], color="#ef6c00", lw=2, label="ToF 빔"),
            plt.Line2D([], [], color="#8e24aa", marker=".", ls="", label="턱 넘기 구간"),
        ]
        ax.legend(handles=handles, loc="lower center", ncol=6, fontsize=9, frameon=False,
                  bbox_to_anchor=(0.5, -0.2))

    def draw_text(s):
        ax = ax_txt
        ax.cla()
        ax.axis("off")
        cmd, sc = s["cmd"], s["scene"]
        if cmd.state in STATE_LABEL:
            label, col = STATE_LABEL[cmd.state]
        elif cmd.note.startswith("우회"):
            deg = math.degrees(cmd.heading)
            if abs(deg) < 5:
                label = "장애물 옆으로 지나가는 중"
            else:
                label = f"장애물 피해서 {'왼쪽' if deg > 0 else '오른쪽'}으로 우회"
            col = "#d32f2f"
        else:
            label, col = "직진 주행", "#2e7d32"
        ax.text(0.0, 0.95, "현재 동작", fontsize=11, color="#555", transform=ax.transAxes)
        ax.text(0.0, 0.80, label, fontsize=19, fontweight="bold", color=col, transform=ax.transAxes)
        ax.text(0.0, 0.64, "센서가 본 정면", fontsize=11, color="#555", transform=ax.transAxes)
        rows = [
            ("라이다", "높이 150mm 넘는 것만 보임", _dist_text(s["lidar_front"])),
            ("ToF", f"높이 {cfg.tof.height_mm}mm 넘는 것만 보임", _dist_text(s["tof"])),
            ("카메라", "낮은 턱까지 다 보임", _dist_text(s["cam_front"])),
        ]
        for i, (n, d, v) in enumerate(rows):
            yy = 0.52 - i * 0.11
            ax.text(0.0, yy, n, fontsize=12, fontweight="bold", transform=ax.transAxes)
            ax.text(0.14, yy, d, fontsize=10.5, color="#666", transform=ax.transAxes)
            ax.text(0.68, yy, v, fontsize=12, transform=ax.transAxes,
                    color="#111" if v != "안 보임" else "#9e9e9e")
        if cmd.state == CLIMB:
            verdict, vc = "턱 넘는 중 (OpenCR 편심 시퀀스)", "#8e24aa"
        elif sc.front_class == LOW:
            verdict, vc = f"카메라엔 보이는데 ToF엔 안 보임 → {lim}mm 이하 턱 → 넘는다", "#b26a00"
        elif sc.front_class == TALL:
            verdict, vc = f"ToF나 라이다에 보임 → {lim}mm보다 높음 → 돌아간다", "#d32f2f"
        else:
            verdict, vc = "앞이 비어 있음 → 직진", "#2e7d32"
        ax.text(0.0, 0.12, "판단", fontsize=11, color="#555", transform=ax.transAxes)
        ax.text(0.0, 0.0, verdict, fontsize=12.5, fontweight="bold", color=vc, transform=ax.transAxes)
        ax.text(1.0, 0.95, f"{demo.t:5.1f}s   턱 넘기 {sim.climbed}회 · 충돌 {sim.collisions}회",
                fontsize=10.5, ha="right", color="#444", transform=ax.transAxes)

    def draw_side(s):
        ax = ax_side
        ax.cla()
        x, y, yaw = s["pose"]
        L = cfg.robot.length_mm / 2
        d = np.arange(0, 1600, 5)
        h = np.zeros_like(d, dtype=float)
        for i, dd in enumerate(d):
            px, py = x + (L + dd) * math.cos(yaw), y + (L + dd) * math.sin(yaw)
            for bx in sim.boxes:
                if bx.name != "벽" and bx.x0 <= px <= bx.x1 and bx.y0 <= py <= bx.y1:
                    h[i] = max(h[i], bx.h)
        ax.fill_between(d, 0, h, step="mid", color="#8d6e63", alpha=0.75, lw=0)
        ax.axhline(150, color="#c62828", ls="--", lw=1.2)
        ax.text(1580, 168, "라이다 스캔 높이 150mm", color="#c62828", ha="right", va="bottom", fontsize=9.5)
        ax.axhline(lim, color="#ef6c00", ls="--", lw=1.2)
        ax.text(1580, lim + 18, f"ToF 높이 = 넘을 수 있는 한계 {lim}mm", color="#ef6c00", ha="right", va="bottom", fontsize=9.5)
        lift = 25 if s["cmd"].state == CLIMB else 0
        ax.add_patch(Rectangle((-2 * L, 45 + lift), 2 * L, 70, color="#8e24aa" if lift else C_ROBOT))
        for wx in (-2 * L + 50, -50):
            ax.add_patch(plt.Circle((wx, 50), 50, color="#424242"))
        ax.set_xlim(-2 * L - 30, 1600)
        ax.set_ylim(0, 420)
        ax.set_aspect("equal")
        ax.set_xlabel("로봇 앞 거리 (mm)", fontsize=10)
        ax.set_ylabel("높이 (mm)", fontsize=10)
        ax.set_title("옆에서 본 정면 (진행 방향 단면)", loc="left", fontsize=11)

    def update(_):
        for _ in range(skip):
            if not demo.done():
                demo.step()
        s = demo.snap
        draw_map(s)
        draw_text(s)
        draw_side(s)
        return []

    demo.step()
    frames = int(90 * cfg.loop_hz / skip)

    def gen():
        i = 0
        while not demo.done() and i < frames:
            yield i
            i += 1
        for k in range(int(cfg.loop_hz * 1.5 / skip)):  # 끝 화면 잠시 유지
            yield i + k

    fps = cfg.loop_hz / skip * 1.0
    anim = animation.FuncAnimation(fig, update, frames=gen, interval=1000 / (fps * 2), repeat=False,
                                   cache_frame_data=False, save_count=frames)
    if video:
        if video.lower().endswith(".mp4") and shutil.which("ffmpeg"):
            writer = animation.FFMpegWriter(fps=fps * 2, bitrate=2500)
        else:
            if video.lower().endswith(".mp4"):
                video = video[:-4] + ".gif"
                print("ffmpeg가 없어 GIF로 저장합니다:", video)
            writer = animation.PillowWriter(fps=fps * 2)
        anim.save(video, writer=writer, dpi=90)
        print("저장:", video)
    if live:
        plt.show()
    plt.close(fig)
    return sim
