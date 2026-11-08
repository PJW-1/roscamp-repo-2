"""ETA 1부: 레인 좌표로 잰 남은 거리와 꺾임 시간(버전 2 2번, 공식 ETA)."""
import math
from dataclasses import replace
from pathlib import Path

import pytest
from fleet_core.common.params import load_params
from fleet_core.traffic.eta import arrival_offsets, turn_s
from fleet_core.traffic.lane_map import load_lane_map

REPO = Path(__file__).resolve().parents[3]
SITE = load_lane_map(REPO / "sim/fake_robots/maps/feature_map.yaml")
PARAMS = replace(load_params(), eta_speed_mps=0.2, eta_turn_s_per_rad=1.0, eta_turn_min_rad=0.25)


def pose(waypoint):
    return SITE.waypoint_pose(waypoint)


def test_the_distance_is_the_real_straight_length_not_the_planning_cost():
    # E0 -> E1 -> E2 is 0.35 m twice along y = 1.5; lane.length is only a planning cost
    x, y = pose("E0")
    times = arrival_offsets(SITE, PARAMS, ["E0", "E1", "E2"], x, y, 0.0)
    assert times == pytest.approx([0.0, 0.35 / 0.2, 0.70 / 0.2])
    assert SITE.lane("E0", "E1").span == pytest.approx(0.35)


def test_the_first_segment_is_measured_from_where_the_robot_is():
    x, y = pose("E0")
    times = arrival_offsets(SITE, PARAMS, ["E0", "E1"], x + 0.10, y, 0.0)
    assert times[1] == pytest.approx(0.25 / 0.2)


def test_a_single_waypoint_path_has_nothing_left():
    x, y = pose("E0")
    assert arrival_offsets(SITE, PARAMS, ["E0"], x, y, 0.0) == [0.0]


def test_a_corner_over_the_minimum_angle_costs_turn_time():
    # W -> E0 heads up and east, E0 -> E1 heads east: a corner
    wx, wy = pose("W")
    ex, ey = pose("E0")
    angle = math.atan2(ey - wy, ex - wx)
    times = arrival_offsets(SITE, PARAMS, ["W", "E0", "E1"], wx, wy, angle)
    run = math.dist((wx, wy), (ex, ey)) / 0.2
    assert times[1] == pytest.approx(run)
    assert times[2] == pytest.approx(run + angle * 1.0 + 0.35 / 0.2)


def test_the_initial_heading_counts_as_a_turn():
    x, y = pose("E0")
    times = arrival_offsets(SITE, PARAMS, ["E0", "E1"], x, y, math.pi)  # facing west
    assert times[1] == pytest.approx(math.pi * 1.0 + 0.35 / 0.2)


def test_a_small_angle_is_not_a_turn():
    assert turn_s(PARAMS, 0.2) == 0.0
    assert turn_s(PARAMS, 0.3) == pytest.approx(0.3)


def test_no_turn_rate_means_no_turn_time():
    params = replace(PARAMS, eta_turn_s_per_rad=0.0)
    x, y = pose("E0")
    assert arrival_offsets(SITE, params, ["E0", "E1"], x, y, math.pi)[1] == pytest.approx(
        0.35 / 0.2)
