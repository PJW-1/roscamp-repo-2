"""자유 구역 F-1: 한 번에 한 대(버전 2 F절).

자유 구역은 구역 하나다. 들어갈 로봇은 입구에서 자유 구역과 출구 다음 구역을 한꺼번에 청하고
(ZoneLocks.request_all), 둘 다 될 때만 들어간다. 안 되면 아무것도 잡지 않고 두 구역의 줄에 서
있으므로 대기 그래프가 그 대기를 본다. 출구 다음 구역은 자유 구역을 지나는 동안 "다음 구역"과
같이 쥔다(_needed_zones).
"""
from pathlib import Path

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
    def __init__(self, starts):
        self.now = [0.0]
        self.site = load_lane_map(FREE_MAP)
        self.params = load_params()
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
