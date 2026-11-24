import math
import struct

import numpy as np
import pytest

from remo_nav.__main__ import run
from remo_nav.config import load_config
from remo_nav.lidar_ld19 import PKT_LEN, crc8, parse_packet
from remo_nav.opencr_link import checksum, encode_cmd, parse_status
from remo_nav.sim import SimBackend


@pytest.fixture
def cfg():
    c = load_config()
    c.log.enabled = False
    return c


@pytest.mark.parametrize("scenario, climbs", [("tall_box", 0), ("low_step", 1), ("mid_box", 0), ("course", 1)])
def test_scenarios_reach_goal_without_collision(cfg, scenario, climbs):
    b = SimBackend(cfg, scenario)
    run(cfg, b, max_time=90, verbose=False)
    assert b.done(), f"{scenario}: 목표 미도달"
    assert b.collisions == 0, f"{scenario}: 충돌 {b.collisions}"
    assert b.climbed == climbs


def test_climb_disabled_when_wheel_fault(cfg):
    """바퀴 고장 시에는 턱을 넘지 않는다 (low_step은 우회도 불가 → 목표 미도달, 충돌 없음)."""
    b = SimBackend(cfg, "low_step")
    orig = b.status

    def faulty():
        st = orig()
        st.fault_mask = 0b0001
        return st

    b.status = faulty
    run(cfg, b, max_time=30, verbose=False)
    assert b.climbed == 0
    assert b.collisions == 0


def _make_packet(start_cdeg, end_cdeg, dists):
    body = struct.pack("<BBHH", 0x54, 0x2C, 3600, start_cdeg)
    for d in dists:
        body += struct.pack("<HB", d, 200)
    body += struct.pack("<HH", end_cdeg, 1234)
    return body + bytes([crc8(body)])


def test_ld19_packet_parse():
    pkt = _make_packet(35000, 900, list(range(100, 1300, 100)))
    assert len(pkt) == PKT_LEN
    start, pts = parse_packet(pkt)
    assert start == 350.0
    assert pts[0][0] == pytest.approx(350.0)
    assert pts[-1][0] == pytest.approx(9.0)  # 0° 넘어감 처리
    assert pts[3][1] == 400
    bad = bytearray(pkt)
    bad[10] ^= 0xFF
    assert parse_packet(bytes(bad)) is None


def test_opencr_protocol_roundtrip():
    assert encode_cmd(150, 0.5, 1).decode().startswith("$CMD,150,500,1*")
    body = "STA,1,2,150,-300,9000,400,0,1200"
    st = parse_status(f"${body}*{checksum(body)}\n")
    assert st.mode == 1 and st.fault_mask == 2
    assert st.pitch_deg == pytest.approx(-3.0)
    assert st.yaw_rad == pytest.approx(math.pi / 2)
    assert st.ultrasonic_mm == [400, 0, 1200]
    assert parse_status(f"${body}*00\n") is None


def test_vision_row_to_ground():
    from remo_nav.vision import row_to_ground_mm

    # 화면 중앙 행 = 광축 → 거리 = h / tan(tilt)
    assert row_to_ground_mm(60, 60, 100, 180, 30) == pytest.approx(180 / math.tan(math.radians(30)))
    assert row_to_ground_mm(0, 60, 100, 180, 30) is None  # 지평선 위
