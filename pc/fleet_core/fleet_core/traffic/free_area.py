"""자유 구역 안 경로(버전 2 F-1, 계획서 F절 3~5번).

광장 같은 자유 구역은 지도에서 출입구 사이의 가상 레인으로 이어진다(lane_map.free_lanes).
로봇이 자유 구역과 출구 다음 구역을 쥐고 가상 레인 하나를 목표에 이으려 하면:
  - 그 레인의 경로가 아직 없으면 로봇 planner에 입구에서 출구까지 계산을 청하고(PlanCmd,
    로봇의 compute_path_to_pose) 목표는 입구까지만 둔다. 기다리는 동안 다시 청하지 않는다.
  - 답(on_plan_result)이 끝을 출입구로 맞춘 뒤 점 사이 선까지 다각형 바닥 안
    (lane_map.free_path_outside), 입구와 출구에서 arrival_tolerance_m
    안, 직선 길이의 free_path_max_ratio배 이하이면 그 점들을 쓴다(FREE_PATH). 끝 두 점은 출입구
    자리로 맞춘다.
  - 아니면, 또는 free_plan_wait_s 안에 답이 없으면 출입구를 잇는 직선을 쓴다(FREE_PATH_REJECTED).
받은 경로는 MoveCmd.path의 그 구간에 들어가고, 그 구간에서는 레인 대신 받은 경로로 이탈을 잰다.
경로가 바뀌면 새 경로에 없는 구간의 경로와 기다림은 버린다.

진행 추적과 구역 해제는 가상 레인(직선) 기준 그대로다. 자유 구역은 구역 하나이고 로봇은 그
바닥 안에만 있으므로, 직선에 투영한 진행으로도 해제 판단(출구 레인 s > d, 바닥에서 d)은 같다.

칸 모드(free_cells, 버전 2 F-2): 자유 구역 하나 대신 칸과 진입 레인(lane_map.cell_units)을
잠가 여러 대가 함께 지난다.
  - 들어가기 전에 광장 안 경로부터 받는다(path 모드. 답이 없으면 직선). point 모드는 로봇이
    알아서 지나므로 칸을 모두 잡는다.
  - 그 경로(자유 구역 앞뒤 레인 포함)에서 _cell_reach(release_distance_m와
    lane_error_warn_m + robot_radius_m 중 큰 것) 안에 드는 칸, 들어오는 출입구의 진입 레인,
    출구 다음 구역을 모두 한꺼번에 청한다(request_all, 전부 아니면 전무). 들어갈 때는 쥔 채
    다른 칸을 기다리지 않는다. 예외: 자유 구역 안에서 경로가 바뀌면 몸 둘레 칸을 쥔 채 새 묶음을
    기다린다. 그 대기는 대기 그래프에 들고, 칸 모드는 자유 구역 안으로 물러나지 않으므로
    교착이면 멈춰 기다린다(안전 쪽).
  - 로봇 몸(중심이 경로에서 e <= lane_error_warn_m 안, 반지름 r)은 쥔 칸 안에 있다. 더 벗어나면
    레인 이탈 감시가 세운다. 칸이 겹치지 않으면 몸도 겹치지 않는다.
  - 지나간 칸은 지금의 해제 규칙(칸 상자에서 release_distance_m 넘게 떨어진 보고 N번)으로
    놓는다. 칸에는 중심선이 없어 half_width를 더하지 않는다. 남은 경로에서 _cell_reach 안에 드는
    칸은 놓지 않는다. 남은 경로는 오차 한도 안의 가장 앞 선분부터 센다.
  - 칸을 다 쥐었으면 그 자유 구역을 "쥔 구역"으로 본다(_held_view). 자유 구역 단위 자체는
    잡지 않는다.

TrafficController에서 떼어 낸 부분이다. 제어기 상태를 쓰므로 혼자 쓰지 않는다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from fleet_core.common.commands import PlanCmd
from fleet_core.traffic.lane_map import _point_segment
from fleet_core.traffic.mission import _Mission


@dataclass
class _Claim:
    """칸 모드에서 자유 구역 하나를 지날 때 청하는 단위 묶음(F-2)."""
    key: tuple  # 자유 구역 앞 지점부터 뒤 지점까지의 경로 지점들. 바뀌면 다시 계산한다
    units: frozenset  # 칸과 진입 레인
    line: tuple  # 칸을 고른 꺾은선. 지나간 칸을 놓을 때 남은 부분을 본다
    granted: bool = False


def _remaining(line, x: float, y: float, near: float = 0.0) -> list:
    """꺾은선에서 로봇보다 앞인 부분: 로봇 자리, 고른 선분 위의 발, 그 뒤 점들.

    로봇에서 near 안인 가장 앞 선분을 고르고, 없으면 가장 가까운 선분이다. 되돌아오는 경로의
    뒤 선분을 골라 갈 길을 버리지 않는다(남기는 쪽으로 틀린다)."""
    if len(line) == 1:
        return [(x, y), line[0]]
    gaps = [_point_segment(x, y, *line[i], *line[i + 1]) for i in range(len(line) - 1)]
    best = next((i for i, gap in enumerate(gaps) if gap <= near),
                min(range(len(gaps)), key=gaps.__getitem__))
    (ax, ay), (bx, by) = line[best], line[best + 1]
    vx, vy = bx - ax, by - ay
    denom = vx * vx + vy * vy
    t = max(0.0, min(1.0, ((x - ax) * vx + (y - ay) * vy) / denom)) if denom else 0.0
    return [(x, y), (ax + t * vx, ay + t * vy), *line[best + 1:]]


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
        inner = [(float(x), float(y)) for x, y in points[1:-1]]
        if self.site.free_path_outside(area, [a, *inner, b]):  # 끝을 출입구로 맞춘 뒤의 선
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
        """index에서 나가는 레인을 목표에 이어도 되나.

        자유 구역 안으로 이으려면 출구 다음 구역도 쥐어야 한다(경로가 바뀌어 출구가 달라진
        경우). 가상 레인이면 경로가 있어야 한다.
        """
        key = (m.path[index], m.path[index + 1])
        area = self._zone(m.path[index + 1])
        if area in self.site.free_areas:
            if self._cells_on:
                claim = m.claims.get(area)
                if claim is None or not claim.granted:
                    return False  # 칸과 출구 다음 구역을 한꺼번에 쥐어야 들어간다(F-2)
            else:
                after = self._zone_after(m, area)
                if after is not None and after not in self.zones.held_by(m.name):
                    return False
        if not self._follows_path:
            return True  # point 모드는 MoveCmd.path를 보내지 않는다. 로봇이 알아서 지난다
        if key not in self.site.free_lanes or key in m.free_paths:
            return True
        return self._ask_free_path(m, key)

    def _ask_free_path(self, m: _Mission, key: tuple) -> bool:
        """가상 레인 key의 경로를 로봇 planner에 청한다. 쓸 경로(받았거나 직선)가 있으면 True."""
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
        """받은 경로가 있는 가상 레인이면 그 경로에서의 이탈, 아니면 None.

        blocking.py의 _gap_to도 이것을 먼저 본다. 기준 레인 고르기도 받은 경로 모양으로 한다.
        """
        if lane not in m.free_paths:
            return None
        return polyline_gap(m.x, m.y, m.free_paths[lane])

    # ---- cell locks (버전 2 F-2) ----------------------------------------------------------------

    @property
    def _cells_on(self) -> bool:
        return self.params.free_cells and bool(self.site.free_areas)

    @property
    def _cell_reach(self) -> float:
        """경로에서 이만큼 안의 칸을 청한다. 몸(중심이 경로에서 lane_error_warn_m 안, 반지름 r)이
        쥔 칸 안에 있도록 e + r보다 작지 않게 한다."""
        p = self.params
        return max(p.release_distance_m, p.lane_error_warn_m + p.robot_radius_m)

    def _held_view(self, m: _Mission) -> list:
        """경로 규칙에 쓰는 쥔 구역. 칸 모드에서는 칸 잠금 단위 대신 칸을 다 쥔 자유 구역 이름."""
        held = self.zones.held_by(m.name)
        if not self._cells_on:
            return held
        return ([zone for zone in held if self.site.unit_area(zone) is None]
                + [area for area, claim in m.claims.items() if claim.granted])

    def _area_cells(self, area: str) -> set:
        return {u for u, a in self.site.cell_units().items() if a == area and ">" not in u}

    def _spot_units(self, m: _Mission, zone: str) -> list:
        """자리 잡기와 통신 복구 때 로봇 자리에서 쥘 단위. 칸 모드의 자유 구역이면 둘레 칸."""
        if not (self._cells_on and zone in self.site.free_areas):
            return [zone]
        near = self.site.cells_near(zone, [(m.x, m.y)], self.params.release_distance_m)
        return sorted(near or self._area_cells(zone))

    def _crossing(self, m: _Mission, area: str) -> tuple | None:
        """경로에서 area 지점이 이어진 구간 (i, j). 로봇이 그 안이면 그 구간, 아니면 앞 첫 구간."""
        path = m.path
        if self._zone(path[m.idx]) == area:
            i = m.idx
            while i > 0 and self._zone(path[i - 1]) == area:
                i -= 1
        else:
            i = next((k for k in range(m.idx + 1, len(path)) if self._zone(path[k]) == area),
                     None)
            if i is None:
                return None
        j = i
        while j + 1 < len(path) and self._zone(path[j + 1]) == area:
            j += 1
        return i, j

    def _crossing_line(self, m: _Mission, i: int, j: int) -> list:
        """앞 지점, 자유 구역 지점들(받은 경로 점 포함), 뒤 지점을 잇는 꺾은선."""
        pose = self.site.waypoint_pose
        points = [pose(m.path[i - 1])] if i > 0 else []
        for k in range(i, j + 1):
            points.append(pose(m.path[k]))
            if k < j and (m.path[k], m.path[k + 1]) in m.free_paths:
                points.extend(m.free_paths[(m.path[k], m.path[k + 1])][1:-1])
        if j + 1 < len(m.path):
            points.append(pose(m.path[j + 1]))
        return points

    def _claim(self, m: _Mission, area: str) -> _Claim | None:
        """area를 지나는 데 청할 단위 묶음. 경로가 그대로면 전과 같은 것, 경로를 기다리면 None."""
        span = self._crossing(m, area)
        if span is None:
            return None
        i, j = span
        key = tuple(m.path[max(i - 1, 0):j + 2])
        old = m.claims.get(area)
        if old is not None and old.key == key:
            return old
        if self._follows_path:
            for k in range(i, j):
                lane = (m.path[k], m.path[k + 1])
                if (lane in self.site.free_lanes and lane not in m.free_paths
                        and not self._ask_free_path(m, lane)):
                    return None
        line = _remaining(self._crossing_line(m, i, j), m.x, m.y, self.params.lane_error_warn_m)
        if self._follows_path:
            units = set(self.site.cells_near(area, line, self._cell_reach))
        else:  # 로봇이 알아서 지나므로 경로를 모른다
            units = self._area_cells(area)
        if i > 0 and self._zone(m.path[i - 1]) != area:
            approach = self.site.approach_unit(area, m.path[i])
            if approach is not None:
                units.add(approach)
        m.claims[area] = _Claim(key, frozenset(units), tuple(line))
        return m.claims[area]

    def _request_cells(self, m: _Mission, area: str) -> bool:
        """칸 묶음과 출구 다음 구역을 한꺼번에 청한다. 쥐었으면 True."""
        claim = self._claim(m, area)
        if claim is None:
            return False  # 광장 안 경로를 기다린다
        if claim.granted:
            return True
        _, j = self._crossing(m, area)
        after = [self._zone(m.path[j + 1])] if j + 1 < len(m.path) else []
        if not self.zones.request_all(m.name, sorted(claim.units) + after):
            self._note_waiting(m, area)
            return False
        claim.granted = True
        m.wait_zone = None
        cells = sorted(claim.units & self._area_cells(area))
        self._event("INFO", "FREE_CELLS", f"{area}: {len(cells)} cells {cells}", m.name)
        return True

    def _cells_kept(self, m: _Mission, area: str, current: str) -> set:
        """area의 단위 중 놓으면 안 되는 것. 들어가기 전에는 청한 묶음 전부, 안에서는 남은
        경로에 닿는 칸(point 모드는 경로를 모르니 전부)."""
        claim = m.claims.get(area)
        if claim is None:
            return set()
        if not claim.granted or area != current or not self._follows_path:
            return set(claim.units)
        rest = _remaining(claim.line, m.x, m.y, self.params.lane_error_warn_m)
        return set(self.site.cells_near(area, rest, self._cell_reach)) & claim.units

    def _unit_needs(self, m: _Mission, needed: set, current: str) -> set:
        """_needed_zones의 자유 구역 이름을 놓으면 안 되는 칸 잠금 단위로 바꾼다."""
        out = set()
        for zone in needed:
            if zone in self.site.free_areas:
                out |= self._cells_kept(m, zone, current)
            else:
                out.add(zone)
        return out

    def _forget_claims(self, m: _Mission) -> None:
        """지금·다음 구역이 아닌 자유 구역의 묶음을 버린다. 그 칸은 해제 규칙으로 놓인다."""
        if not m.claims:
            return
        window = set(self._upcoming(m))
        for area in [a for a in m.claims if a not in window]:
            del m.claims[area]

    def _drop_stale_claims(self, m: _Mission) -> None:
        """경로가 바뀌어 지나는 구간이 달라진 묶음을 버린다. 다음에 새로 계산해 청한다."""
        for area, claim in list(m.claims.items()):
            span = self._crossing(m, area)
            if span is None or tuple(m.path[max(span[0] - 1, 0):span[1] + 2]) != claim.key:
                del m.claims[area]
