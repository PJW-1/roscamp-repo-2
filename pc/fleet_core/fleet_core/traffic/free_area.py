"""자유 구역 안 경로(버전 2 F-1, 계획서 F절 3~5번).

광장 같은 자유 구역은 지도에서 출입구 사이의 가상 레인으로 이어진다(lane_map.free_lanes).
로봇이 자유 구역과 출구 다음 구역을 쥐고 가상 레인 하나를 목표에 이으려 하면:
  - 그 레인의 경로가 아직 없으면 로봇 planner에 입구에서 출구까지 계산을 청하고(PlanCmd,
    로봇의 compute_path_to_pose) 목표는 입구까지만 둔다. 기다리는 동안 다시 청하지 않는다.
  - 답(on_plan_result)이 바닥 안(distance_to_zone == 0), 입구와 출구에서 arrival_tolerance_m
    안, 직선 길이의 free_path_max_ratio배 이하이면 그 점들을 쓴다(FREE_PATH). 끝 두 점은 출입구
    자리로 맞춘다.
  - 아니면, 또는 free_plan_wait_s 안에 답이 없으면 출입구를 잇는 직선을 쓴다(FREE_PATH_REJECTED).
받은 경로는 MoveCmd.path의 그 구간에 들어가고, 그 구간에서는 레인 대신 받은 경로로 이탈을 잰다.
경로가 바뀌면 새 경로에 없는 구간의 경로와 기다림은 버린다.

진행 추적과 구역 해제는 가상 레인(직선) 기준 그대로다. 자유 구역은 구역 하나이고 로봇은 그
바닥 안에만 있으므로, 직선에 투영한 진행으로도 해제 판단(출구 레인 s > d, 바닥에서 d)은 같다.

TrafficController에서 떼어 낸 부분이다. 제어기 상태를 쓰므로 혼자 쓰지 않는다.
"""
from __future__ import annotations

import math

from fleet_core.common.commands import PlanCmd
from fleet_core.traffic.lane_map import _point_segment
from fleet_core.traffic.mission import _Mission


def polyline_gap(x: float, y: float, points) -> float:
    """점에서 꺾은선까지 거리."""
    if len(points) == 1:
        return math.dist((x, y), points[0])
    return min(_point_segment(x, y, *a, *b) for a, b in zip(points, points[1:], strict=False))


def _length(points) -> float:
    return sum(math.dist(a, b) for a, b in zip(points, points[1:], strict=False))


class TrafficFreeArea:
    def on_plan_result(self, robot, plan_id, points) -> None:
        """경로 계산의 답. 실패면 points가 None이거나 비었다. 지금 기다리는 요청만 본다."""
        m = self._missions.get(robot)
        if m is None or m.plan_wait is None or m.plan_wait[1] != plan_id:
            return
        key = m.plan_wait[0]
        m.plan_wait = None
        problem = self._free_path_problem(key, points)
        if problem:
            self._fall_back_straight(m, key, problem)
            return
        a, b = self.site.waypoint_pose(key[0]), self.site.waypoint_pose(key[1])
        inner = [(float(x), float(y)) for x, y in points[1:-1]]
        m.free_paths[key] = (a, *inner, b)
        self._event("INFO", "FREE_PATH",
                    f"{self.site.free_area_of(key)} {key[0]} to {key[1]}: {len(inner)} points "
                    f"inside, {_length(m.free_paths[key]):.2f} m", m.name)

    def _free_path_problem(self, key: tuple, points) -> str:
        if not points or len(points) < 2:
            return "no path"
        area = self.site.free_area_of(key)
        a, b = self.site.waypoint_pose(key[0]), self.site.waypoint_pose(key[1])
        tolerance = self.params.arrival_tolerance_m
        if math.dist(points[0], a) > tolerance or math.dist(points[-1], b) > tolerance:
            return "not from gate to gate"
        if any(self.site.distance_to_zone(x, y, area) > 0.0 for x, y in points):
            return f"leaves the {area} floor"
        limit = self.params.free_path_max_ratio * math.dist(a, b)
        if _length(points) > limit:
            return f"{_length(points):.2f} m is longer than {limit:.2f} m"
        return ""

    def _fall_back_straight(self, m: _Mission, key: tuple, why: str) -> None:
        a, b = self.site.waypoint_pose(key[0]), self.site.waypoint_pose(key[1])
        m.free_paths[key] = (a, b)
        self._event("WARN", "FREE_PATH_REJECTED",
                    f"{key[0]} to {key[1]}: {why}, using the straight line", m.name)

    def _free_crossing_ready(self, m: _Mission, index: int) -> bool:
        """index에서 나가는 레인을 목표에 이어도 되나. 가상 레인이면 경로가 있어야 한다."""
        key = (m.path[index], m.path[index + 1])
        if key not in self.site.free_lanes or key in m.free_paths:
            return True
        now = self._clock()
        if m.plan_wait is None or m.plan_wait[0] != key:
            plan_id = self._next_goal_id()
            m.plan_wait = (key, plan_id, now)
            (ax, ay), (bx, by) = self.site.waypoint_pose(key[0]), self.site.waypoint_pose(key[1])
            self._out.append(PlanCmd(m.name, plan_id, self.site.free_area_of(key), ax, ay,
                                     bx, by, math.atan2(by - ay, bx - ax)))
            return False
        if now - m.plan_wait[2] >= self.params.free_plan_wait_s:
            m.plan_wait = None
            self._fall_back_straight(m, key, f"no answer in {self.params.free_plan_wait_s:.0f} s")
            return True
        return False

    def _drop_stale_plans(self, m: _Mission) -> None:
        """새 경로에 없는 구간의 경로와 기다림을 버린다."""
        keys = set(zip(m.path, m.path[1:], strict=False))
        m.free_paths = {k: v for k, v in m.free_paths.items() if k in keys}
        if m.plan_wait is not None and m.plan_wait[0] not in keys:
            m.plan_wait = None

    def _free_points_ahead(self, m: _Mission, key: tuple) -> list:
        """받은 경로에서 로봇보다 앞인 점들(로봇에 가장 가까운 선분의 끝부터)."""
        points = m.free_paths[key]
        best = min(range(len(points) - 1),
                   key=lambda i: _point_segment(m.x, m.y, *points[i], *points[i + 1]))
        return list(points[best + 1:])

    def _free_gap(self, m: _Mission, lane: tuple) -> float | None:
        """받은 경로가 있는 가상 레인이면 그 경로에서의 이탈, 아니면 None."""
        if lane not in m.free_paths:
            return None
        return polyline_gap(m.x, m.y, m.free_paths[lane])
