"""도착 예상 시간(버전 2 2번, 공식판)을 매니저 수준에서 낸다. 관측만 하고 상태를 바꾸지 않는다.

ETA = 남은 길 ÷ 속도 + 꺾임 시간(traffic/eta.py) + 예상 대기. 스테이션 작업 남은 시간은 주문 쪽에만
있고 구역은 교통 쪽에만 있어서, 두 코어를 함께 보는 FleetManager가 이 계산을 가진다.

예상 대기: 경로가 새 구역으로 들어서는 경계마다, 그 구역(과 충돌 구역)을 쥔 로봇이 놓을 때까지와
그 구역의 줄에서 앞선 로봇이 쓸 시간을 더한다. 로봇은 경계에 닿은 시각과 구역이 비는 시각 중 늦은
쪽에 출발한다. 쥔 로봇이 놓는 때는 이렇게 어림한다.
  - 스테이션에서 일하는 중: 작업 남은 시간 + 구역을 벗어나는 시간
  - 도킹 중: 도킹 시간 + 작업 시간 + 벗어나는 시간(도킹을 시작한 시각을 몰라 줄이지 않는다)
  - 달리는 중: 경로에서 그 구역의 마지막 지점까지 가는 시간 + 벗어나는 시간. 경로가 그 구역에서
    끝나면 스테이션으로 가던 로봇은 도착 + 도킹 + 작업 + 벗어나기, 대기 지점으로 가던 로봇은 모른다
  - 대기 칸에 서 있거나 경로가 없음: 모른다(station_wait_estimate_s)
  - 고장, 통신 두절, 막힘, 레인 이탈: 무한
쥔 로봇 자신의 구역 대기는 넣지 않는다(재귀를 피한다). 그 로봇이 더 기다리면 ETA는 낙관이다.
학습한 ETA가 아니다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from fleet_core.orders.orchestrator import FILL
from fleet_core.traffic.eta import arrival_offsets
from fleet_core.traffic.lane_map import LaneMapError
from fleet_core.traffic.mission import (
    AT_STATION,
    BLOCKED,
    DOCKING,
    TO_OUTBOUND,
    TO_WORK,
)

STATION_OVERDUE_S = 1.0  # 시간이 지난 팔 작업은 곧 끝난다고 본다
ARRIVED_M = 0.02  # 지점에서 이만큼 안이면 닿은 것으로 본다
BESIDE_M = 0.05  # 레인에서 이만큼 옆이면 지점을 지났어도 거기까지 거리가 남은 것으로 본다


@dataclass(frozen=True)
class Eta:
    travel_s: float  # 남은 길 ÷ 속도 + 꺾임 시간
    wait_s: float  # 예상 대기

    @property
    def total_s(self) -> float:
        return self.travel_s + self.wait_s


class EtaEstimator:
    def __init__(self, site, params, traffic, orchestrator, clock):
        self.site = site
        self.params = params
        self._traffic = traffic
        self._orchestrator = orchestrator
        self._clock = clock
        self._work = next(s.id for s in site.stations.values() if s.action == FILL)

    # ---- public ---------------------------------------------------------------------------

    def eta(self, robot, waypoint=None) -> Eta | None:
        """robot이 waypoint(기본은 지금 경로의 끝)에 닿기까지. 모르거나 지도에 없으면 None."""
        m = self._traffic._missions.get(robot)
        if m is None or not m.placed or not m.path:
            return None
        path = self._remaining(m)
        shift = 0.0
        if m.phase == AT_STATION:
            path = path[-1:]
            if waypoint is None or waypoint == path[0]:
                return Eta(0.0, 0.0)  # it is at its station
            left = self.station_left_s(robot)  # it drives off when the work is done; the
            shift = (self._unknown_s() if left is None  # clearing time is part of that drive
                     else max(0.0, left - self._exit_s()))
        if waypoint is not None and waypoint not in path:
            try:
                path = path + self.site.plan(path[-1], waypoint)[1:]
            except LaneMapError:  # not a waypoint of the map, or no lanes lead there
                return None
        last = len(path) - 1 if waypoint is None else path.index(waypoint)
        offsets = arrival_offsets(self.site, self.params, path, m.x, m.y, m.yaw)
        held = set(self._traffic.zones.held_by(robot))
        for i in range(1, last + 1):
            zone = self.site.zone_of[path[i]]
            if zone == self.site.zone_of[path[i - 1]] or zone in held:
                continue
            free_at = self._free_at(robot, m, zone, offsets[i] + shift)
            shift = max(shift, free_at - offsets[i - 1])
            if shift == math.inf:
                break
        return Eta(offsets[last], shift)

    def station_left_s(self, robot) -> float | None:
        """스테이션에서 일하는 로봇이 도크 구역을 놓기까지. 일하는 중이 아니면 None."""
        m = self._traffic._missions.get(robot)
        if m is None or m.phase != AT_STATION:
            return None
        job = self._orchestrator.station_job(robot)
        if job is None:
            return None
        return self._job_left_s(job) + self._exit_s()

    # ---- pieces ---------------------------------------------------------------------------

    def _remaining(self, m) -> list:
        """The rest of the path, from the waypoint the robot is heading to.

        Traffic moves idx on once the robot is release_distance_m along a lane, so path[idx] can
        still be ahead of the robot: then the lane before it is kept to measure from the robot.
        """
        if m.idx > 0:
            a, b = m.path[m.idx - 1], m.path[m.idx]
            s, side = self.site.lane_progress(a, b, m.x, m.y)
            if s < self.site.lane(a, b).span - ARRIVED_M or side > BESIDE_M:
                return m.path[m.idx - 1:]  # not there yet, or beside the waypoint
        return m.path[m.idx:]

    def _exit_s(self) -> float:
        return self.params.release_distance_m / self.params.eta_speed_mps

    def _unknown_s(self) -> float:
        return self.params.station_wait_estimate_s

    def _work_s(self, station_id, items=None) -> float:
        if station_id == self._work:
            count = self.params.eta_fill_items_default if items is None else items
            return self.params.eta_fill_base_s + count * self.params.eta_fill_s_per_item
        return self.params.eta_swap_s

    def _job_left_s(self, job) -> float:
        kind, items, sent = job
        total = self._work_s(self._work if kind == FILL else "", items)
        if sent is None:
            return total
        return max(STATION_OVERDUE_S, total - (self._clock() - sent))

    def _free_at(self, robot, m, zone, arrival) -> float:
        """robot이 zone에 들어가도 되는 때(지금부터 초): 쥔 로봇들이 놓고, 줄 앞 로봇들이 쓴 뒤.

        zone이 robot이 가는 스테이션의 도크 구역이면, 같은 스테이션으로 먼저 닿는 로봇들이 차례로
        도킹하고 일하는 시간도 본다(아직 아무 구역도 못 쥐었어도).
        """
        zones = self._traffic.zones
        holders = zones.holders_around(zone, robot)
        free = max((self._release_s(holder, held) for held, holder in holders.items()),
                   default=0.0)
        if free == math.inf:
            return free
        # waiters ahead of robot in the queue of zone or of a conflicting zone (zones.blockers)
        waiters = zones.blockers(robot, zone) - set(holders.values())
        free += sum(self._hold_s(other, self._blocking_request(other, zone))
                    for other in sorted(waiters))
        station = self.site.stations.get(m.dest)
        if station is not None and self.site.zone_of[station.dock] == zone:
            free = max(free, self._served_before(robot, m, zone, arrival))
        return free

    def _blocking_request(self, other, zone) -> str:
        """The zone `other` queues for that stands in our way: zone itself, else a conflicting one
        (a robot can queue for several zones at once, e.g. a free area and the zone after it)."""
        zones = self._traffic.zones
        conflicts = zones._conflicts[zone]
        asked = zones.requests_of(other)
        return zone if zone in asked else next((z for z in asked if z in conflicts), zone)

    def _served_before(self, robot, m, zone, arrival) -> float:
        """같은 스테이션으로 robot보다 먼저 닿는 로봇들이 차례로 일을 마치는 때."""
        early = []
        for name, other in self._traffic._missions.items():
            if (name == robot or other.dest != m.dest or not other.placed
                    or other.phase not in (TO_WORK, TO_OUTBOUND, DOCKING, AT_STATION)):
                continue
            ahead = self._remaining(other)
            there = other.phase in (DOCKING, AT_STATION) or len(ahead) < 2
            reach = 0.0 if there else arrival_offsets(
                self.site, self.params, ahead, other.x, other.y, other.yaw)[-1]
            if reach < arrival:
                early.append((reach, name))
        free = 0.0
        for reach, name in sorted(early):
            other = self._traffic._missions[name]
            if self._stuck(other):
                return math.inf
            service = (self._release_s(name, zone) if other.phase == AT_STATION
                       else self._service_s(other))
            free = max(free, reach) + service
        return free

    def _stuck(self, m) -> bool:
        # BLOCKED in a line behind another robot (queue_since) is a wait, not a stop (detour.py)
        return bool(m.fault or m.comm or m.block or m.off_lane_since is not None
                    or (m.robot_state == BLOCKED and m.queue_since is None))

    def _service_s(self, m) -> float:
        """스테이션으로 가던 로봇이 도킹하고 일하고 구역을 벗어나기까지(도착 뒤)."""
        station = m.dest or ""
        return self.params.eta_dock_s + self._work_s(station) + self._exit_s()

    def _release_s(self, robot, zone) -> float:
        """robot이 zone(또는 그와 충돌하는 구역)을 놓기까지."""
        m = self._traffic._missions.get(robot)
        if m is None:
            return 0.0  # 모르는 보유자는 벌점 없음(detour.py와 같다)
        if self._stuck(m):
            return math.inf
        if m.phase == AT_STATION:
            left = self.station_left_s(robot)
            return self._unknown_s() if left is None else left
        if m.phase == DOCKING:
            return self._service_s(m)
        path = self._remaining(m)
        if len(path) < 2:
            # at the end of its path: traffic moves a station-bound robot on to DOCKING next tick
            return self._service_s(m) if m.phase in (TO_WORK, TO_OUTBOUND) else self._unknown_s()
        inside = [i for i, w in enumerate(path) if self.site.zone_of[w] == zone]
        if not inside:
            return self._exit_s()  # 이미 지난 구역: 보고가 따라올 때까지만
        offsets = arrival_offsets(self.site, self.params, path, m.x, m.y, m.yaw)
        if inside[-1] < len(path) - 1:
            return offsets[inside[-1]] + self._exit_s()
        if m.phase in (TO_WORK, TO_OUTBOUND):
            return offsets[-1] + self._service_s(m)
        return self._unknown_s()

    def _hold_s(self, robot, zone) -> float:
        """줄 앞 로봇이 zone을 얻은 뒤 쥐는 시간."""
        m = self._traffic._missions.get(robot)
        if m is None or self._stuck(m):
            return math.inf if m is not None else 0.0
        path = self._remaining(m)
        lanes = [self.site.lane(a, b) for a, b in zip(path, path[1:], strict=False)
                 if self.site.zone_of[b] == zone]
        inside = sum(lane.span for lane in lanes) / self.params.eta_speed_mps
        ends_here = path and self.site.zone_of[path[-1]] == zone
        if ends_here and m.phase in (TO_WORK, TO_OUTBOUND):
            return inside + self._service_s(m)
        return inside + self._exit_s()
