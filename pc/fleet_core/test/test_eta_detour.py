"""ETA 3부: 우회 비용의 스테이션 대기 어림을 ETA의 작업 남은 시간으로 바꾼다(detour_eta_wait)."""
from dataclasses import replace
from pathlib import Path

import pytest
from fleet_core.common.params import load_params
from fleet_core.manager import FleetManager
from fleet_core.orders.orchestrator import FILLING, _ArmJob, _BoundOrder
from fleet_core.traffic.lane_map import load_lane_map
from fleet_core.traffic.mission import AT_STATION

REPO = Path(__file__).resolve().parents[3]
SITE = load_lane_map(REPO / "sim/fake_robots/maps/feature_map.yaml")
PARAMS = load_params()


def make(items, **over):
    """r2는 WORK 도크에서 채우는 중(품목 items개, 팔 goal은 0 s에 보냄). r1은 F2에서 출고로."""
    now = [0.0]
    core = FleetManager(SITE, replace(PARAMS, **over), ["r1", "r2"], lambda: now[0])
    for robot, waypoint in (("r1", "F2"), ("r2", "W")):
        x, y = SITE.waypoint_pose(waypoint)
        for _ in range(PARAMS.release_confirm_samples):
            core.on_robot_state(robot, x, y, 0.0, "IDLE")
    core.on_order_board("RUN", [])
    m2 = core.traffic._missions["r2"]
    m2.phase, m2.dest, m2.hold = AT_STATION, "WORK", True
    r2 = core.orchestrator._robots["r2"]
    r2.stage = FILLING
    r2.order = _BoundOrder("O-1", {"A": items})
    r2.arm = _ArmJob("workcell", 1, "fill", sent_at=0.0)
    core.traffic.on_dispatch(5, "OUTBOUND", "r1")
    return core


def detours(core):
    return [e for e in core.tick() if type(e).__name__ == "EventOut" and e.code == "DETOUR"]


def test_the_default_keeps_the_fixed_estimate():
    # 30 s standing at a station is 6 m of penalty: go around, however short the job is
    assert PARAMS.detour_eta_wait is False
    assert detours(make(1))


def test_a_short_job_is_waited_out_when_the_eta_is_used():
    # 4 s of work + 1 s to clear the dock is 1 m of penalty; the way around is about 1.65 m longer
    core = make(1, detour_eta_wait=True)
    assert not detours(core)
    assert core.zones.waiting_for("r1") == "W"


def test_a_long_job_is_gone_around_when_the_eta_is_used():
    # 12 s + 1 s is 2.6 m of penalty
    core = make(3, detour_eta_wait=True)
    assert detours(core)


def test_a_robot_with_no_job_keeps_the_fixed_estimate_when_the_eta_is_used():
    core = make(1, detour_eta_wait=True)
    core.orchestrator._robots["r2"].stage = ""
    assert core.eta_station_left("r2") is None
    core2 = make(1, detour_eta_wait=True)
    core2.orchestrator._robots["r2"].stage = ""
    assert detours(core2)


@pytest.mark.parametrize("flag", [False, True])
def test_a_traffic_core_alone_has_no_hook(flag):
    from fleet_core.traffic.controller import TrafficController
    core = TrafficController(SITE, replace(PARAMS, detour_eta_wait=flag), ["r1"], lambda: 0.0)
    assert core.station_left is None
