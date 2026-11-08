"""자유 구역 F-1: 한 번에 한 대(버전 2 F절).

자유 구역은 구역 하나다. 들어갈 로봇은 입구에서 자유 구역과 출구 다음 구역을 한꺼번에 청하고
(ZoneLocks.request_all), 둘 다 될 때만 들어간다. 안 되면 아무것도 잡지 않고 두 구역의 줄에 서
있으므로 대기 그래프가 그 대기를 본다. 출구 다음 구역은 자유 구역을 지나는 동안 "다음 구역"과
같이 쥔다(_needed_zones).
"""
from dataclasses import replace
from pathlib import Path

from fleet_core.common.commands import PlanCmd
from fleet_core.common.params import load_params
from fleet_core.traffic.controller import TrafficController
from fleet_core.traffic.deadlock import wait_graph
from fleet_core.traffic.lane_map import load_lane_map
from fleet_core.traffic.zones import ZoneLocks

FREE_MAP = Path(__file__).resolve().parents[3] / "sim" / "fake_robots" / "maps" / \
    "feature_map_free.yaml"


# ---- request_all ---------------------------------------------------------------------------


def test_request_all_grants_every_zone_or_none():
    z = ZoneLocks(["A", "B", "C"], lambda: 0.0)
    assert z.request("r2", "B")
    assert not z.request_all("r1", ["A", "B"])
    assert z.holder("A") is None, "하나라도 안 되면 아무것도 잡지 않는다"
    assert set(z.requests_of("r1")) == {"A", "B"}, "두 구역 모두의 줄에 선다"
    assert wait_graph(z) == {"r1": {"r2"}}
    z.release("r2", "B")
    assert z.request_all("r1", ["A", "B"])
    assert z.held_by("r1") == ["A", "B"]
    assert z.requests_of("r1") == ()


def test_request_all_respects_robots_queued_earlier():
    z = ZoneLocks(["A", "B"], lambda: 0.0)
    z.request("r2", "B")
    z.request("r3", "B")  # r3이 먼저 B를 기다린다
    z.release("r2", "B")
    assert not z.request_all("r1", ["A", "B"]), "빈 구역이어도 먼저 기다리는 로봇이 있으면 안 준다"
    assert z.holder("A") is None


def test_request_all_with_zones_already_held():
    z = ZoneLocks(["A", "B"], lambda: 0.0)
    z.request("r1", "A")
    assert z.request_all("r1", ["A", "B"])
    assert z.held_by("r1") == ["A", "B"]


# ---- controller ----------------------------------------------------------------------------


class Drive:
    def __init__(self, starts, free_cells=False, **changes):
        """F-1 시험은 한 번에 한 대(free_cells 끔)로 돈다. F-2 시험은 켠다."""
        self.now = [0.0]
        self.site = load_lane_map(FREE_MAP)
        self.params = replace(load_params(), free_cells=free_cells, **changes)
        self.core = TrafficController(self.site, self.params, list(starts), lambda: self.now[0])
        self.pose = {r: self.site.waypoint_pose(w) for r, w in starts.items()}
        for _ in range(self.params.release_confirm_samples):
            self.report()
        self.core.on_run_mode("RUN")

    def report(self):
        for robot, (x, y) in self.pose.items():
            self.core.on_robot_state(robot, x, y, 0.0, "IDLE")

    def tick(self, n=1):
        out = []
        for _ in range(n):
            self.report()
            out += self.core.tick()
            self.now[0] = round(self.now[0] + 0.1, 6)
        return out

    def m(self, robot="r1"):
        return self.core._missions[robot]


def test_a_robot_takes_the_area_and_the_zone_after_its_exit_together():
    d = Drive({"r1": "F1"})
    d.tick(3)
    assert d.m().path[:4] == ["F1", "G3", "G4", "P-1"]
    assert set(d.core.zones.held_by("r1")) == {"F1", "PLAZA", "P-1"}
    assert d.core._needed_zones(d.m()) == {"F1", "PLAZA", "P-1"}


def test_a_robot_does_not_enter_while_the_exit_zone_is_taken():
    d = Drive({"r1": "F1"})
    assert d.core.zones.request("ghost", "P-1")
    d.tick(5)
    assert d.core.zones.held_by("r1") == ["F1"], "출구 다음 구역이 없으면 광장도 잡지 않는다"
    assert set(d.core.zones.requests_of("r1")) == {"PLAZA", "P-1"}
    assert d.m().goal is None or d.m().goal.target_idx == 0
    d.core.zones.release("ghost", "P-1")
    d.tick(2)
    assert set(d.core.zones.held_by("r1")) == {"F1", "PLAZA", "P-1"}


def test_a_new_route_drops_every_request_it_no_longer_needs():
    d = Drive({"r1": "F1"})
    assert d.core.zones.request("ghost", "P-1")
    d.tick(2)
    m = d.m()
    assert set(d.core.zones.requests_of("r1")) == {"PLAZA", "P-1"}
    d.core._start_leg(m, m.phase, m.dest, "F2")
    assert d.core.zones.requests_of("r1") == ()


def test_a_new_exit_must_be_held_before_the_robot_goes_into_the_area():
    """광장을 쥔 뒤 경로가 바뀌어 출구가 달라지면 새 출구 다음 구역을 쥘 때까지 들어가지 않는다."""
    d = Drive({"r1": "F1"})
    d.tick(3)
    assert set(d.core.zones.held_by("r1")) == {"F1", "PLAZA", "P-1"}
    exit_zone = d.core._zone("QW")
    assert d.core.zones.request("ghost", exit_zone)
    m = d.m()
    d.core._start_leg(m, m.phase, m.dest, "QW")
    assert m.path[:4] == ["F1", "G3", "T1", "QW"]
    out = d.tick(5)
    assert exit_zone in d.core.zones.requests_of("r1"), "새 출구 다음 구역을 청한다"
    # 경로가 바뀌기 전에 이미 낸 입구(G3)까지의 목표는 그대로 두고, 광장을 건너는 레인은 잇지 않는다
    assert m.goal is None or m.path[m.goal.target_idx] in ("F1", "G3"), "광장을 건너지 않는다"
    assert not any(isinstance(c, PlanCmd) and c.goal_x == 1.0 for c in out), \
        "새 출구 전에는 경로도 청하지 않는다"
    d.core.zones.release("ghost", exit_zone)
    out = d.tick(3)
    assert exit_zone in d.core.zones.held_by("r1")
    assert any(isinstance(c, PlanCmd) and (c.goal_x, c.goal_y) == (1.0, 0.6) for c in out), \
        "새 출구를 쥐면 G3-T1 경로를 청한다"


def test_the_status_shows_every_robot_in_the_way_of_an_all_or_nothing_request():
    d = Drive({"r1": "F1"})
    assert d.core.zones.request("ghost", "P-1")
    d.tick(3)
    assert d.core.status_view("r1")["wait_robots"] == ("ghost",)


def test_a_second_route_change_drops_the_exit_request_of_the_first():
    d = Drive({"r1": "F1"})
    d.tick(3)
    m = d.m()
    west, east = d.core._zone("QW"), d.core._zone("QO")
    assert d.core.zones.request("ghost", west) and d.core.zones.request("ghost", east)
    d.core._start_leg(m, m.phase, m.dest, "QW")
    d.tick(2)
    assert west in d.core.zones.requests_of("r1")
    d.core._start_leg(m, m.phase, m.dest, "QO")
    assert m.path[:4] == ["F1", "G3", "T2", "QO"]
    d.tick(2)
    assert west not in d.core.zones.requests_of("r1"), "앞 경로의 출구 요청은 거둔다"
    assert east in d.core.zones.requests_of("r1")
