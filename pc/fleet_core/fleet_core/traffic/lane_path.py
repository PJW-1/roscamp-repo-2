"""레인 폴리라인 목표(지시서 7번, FollowPath).

path 모드: 잠금을 쥔 데까지 레인을 한 목표로 묶는다. 방향이 바뀌어도 잇고, 유턴만 끊는다.
MoveCmd.path에 선을 싣는다. 첫 점은 로봇 위치, 이어서 아직 지나지 않은 지점, 끝은 목표 지점.
point 모드: 같은 방향 레인만 묶고 선을 싣지 않는다. 호환과 비교용이다.
도착 판정과 끝 근처 재전송은 핑키 몫이다(docs/핑키_계약.md).

TrafficController에서 떼어 낸 부분이다. 제어기 상태를 쓰므로 혼자 쓰지 않는다.
"""
from __future__ import annotations

import math

from fleet_core.traffic.mission import _Mission

POINT = "point"
PATH = "path"
GOAL_MODES = (POINT, PATH)
ARC_STEP = math.radians(15.0)  # 원호를 이만큼마다 점으로 나눈다
MIN_TURN = math.radians(1.0)  # 이보다 덜 꺾이면 곧은 것으로 본다
MAX_TURN = math.radians(170.0)  # 이보다 더 꺾이면(거의 유턴) 그대로 둔다
MIN_RADIUS_M = 0.02  # 이보다 작은 원호는 꺾인 점과 다름없어 그대로 둔다(아주 짧은 레인)


def round_corners(points: list, roundable: list, radius: float, deviation: float,
                  accept=None) -> list:
    """꺾인 점을 반지름 radius 이하의 원호로 바꾼다(버전 2 4절, 기본 선택 13번 제안).

    원호는 꺾인 점에서 deviation보다 안쪽으로 들어가지 않는다(꺾인 점과 원호 사이 거리. 레인에서
    벗어나는 폭은 그보다 작다). 접점은 양쪽 선분의 절반 안이라 이웃 원호와 겹치지 않는다.
    roundable이 거짓인 점, 처음과 끝, 곧거나 거의 유턴인 점은 그대로 둔다.
    accept(i, 원호 점들)가 거짓이어도 그 점은 둔다.
    """
    if radius <= 0.0 or len(points) < 3:
        return list(points)
    out = [points[0]]
    for i in range(1, len(points) - 1):
        (ax, ay), (vx, vy), (bx, by) = points[i - 1], points[i], points[i + 1]
        l1, l2 = math.dist((ax, ay), (vx, vy)), math.dist((vx, vy), (bx, by))
        turn = 0.0
        if l1 > 0.0 and l2 > 0.0:
            u1 = ((vx - ax) / l1, (vy - ay) / l1)
            u2 = ((bx - vx) / l2, (by - vy) / l2)
            cross = u1[0] * u2[1] - u1[1] * u2[0]
            turn = math.atan2(abs(cross), u1[0] * u2[0] + u1[1] * u2[1])
        if not roundable[i] or not MIN_TURN <= turn <= MAX_TURN:
            out.append(points[i])
            continue
        half = turn / 2.0
        r = min(radius, deviation / (1.0 / math.cos(half) - 1.0),
                min(l1, l2) / 2.0 / math.tan(half))
        if r < MIN_RADIUS_M:
            out.append(points[i])
            continue
        t = r * math.tan(half)
        side = 1.0 if cross > 0 else -1.0  # 왼쪽으로 꺾이면 중심이 왼쪽
        sx, sy = vx - u1[0] * t, vy - u1[1] * t
        cx, cy = sx - u1[1] * r * side, sy + u1[0] * r * side
        start = math.atan2(sy - cy, sx - cx)
        steps = max(2, math.ceil(turn / ARC_STEP))
        arc = [(cx + r * math.cos(start + side * turn * k / steps),
                cy + r * math.sin(start + side * turn * k / steps)) for k in range(steps + 1)]
        arc[-1] = (vx + u2[0] * t, vy + u2[1] * t)
        if accept is not None and not accept(i, arc):
            out.append(points[i])
            continue
        out += arc
    out.append(points[-1])
    return [p for k, p in enumerate(out) if k == 0 or math.dist(p, out[k - 1]) > 1e-9]


class TrafficPath:
    def _check_goal_mode(self) -> None:
        if self.params.lane_goal_mode not in GOAL_MODES:
            raise ValueError(f"lane_goal_mode must be one of {GOAL_MODES}, "
                             f"not {self.params.lane_goal_mode!r}")

    @property
    def _follows_path(self) -> bool:
        return self.params.lane_goal_mode == PATH

    def _joins(self, m: _Mission, index: int, step) -> bool:
        """index에서 나가는 레인을 목표에 이을 수 있는가.

        point는 같은 방향일 때만. path는 되돌아가는 레인(양방향 레인의 유턴)만 끊는다.
        유턴을 이으면 경로를 바꿀 때마다 목표 끝이 앞으로 밀려 경로가 끝없이 늘어난다.
        새 목표의 첫 레인은 늘 잇는다. 들어온 레인은 지난 일이라 유턴 지점에서 다시 출발한다.
        """
        if not self._follows_path:
            return self._step_between(m, index) == step
        if index == 0 or (m.goal is None and index == m.idx):
            return True
        return m.path[index + 1] != m.path[index - 1]

    def _goal_yaw(self, m: _Mission, target_idx: int, step) -> float:
        """목표 지점에서 볼 방향. path는 마지막 레인 방향이다."""
        if self._follows_path and target_idx > 0:
            step = self._step_into(m, target_idx)
        return math.atan2(step[1], step[0]) if step else m.yaw

    def _lane_polyline(self, m: _Mission, target_idx: int, yaw: float) -> tuple:
        """(x, y, yaw)들. point 모드면 빈 값이다."""
        if not self._follows_path:
            return ()
        points = [(m.x, m.y)]
        first = self._first_ahead(m, target_idx)
        ahead, lane = [], []  # lane: 그 점이 레인 지점인가(자유 구역 경로 점·출입구가 아닌가)
        free = self.site.free_lanes
        if first > 0 and (m.path[first - 1], m.path[first]) in m.free_paths:
            # 자유 구역을 지나는 중(버전 2 F-1): 받은 경로에서 로봇 앞 점들
            ahead += self._free_points_ahead(m, (m.path[first - 1], m.path[first]))[:-1]
            lane += [False] * (len(ahead) - len(lane))
        for k in range(first, target_idx + 1):
            ahead.append(self.site.waypoint_pose(m.path[k]))
            # 자유 구역 출입구는 레인 지점으로 치지 않는다(받은 경로가 직선뿐이어도)
            lane.append((k == 0 or (m.path[k - 1], m.path[k]) not in free)
                        and (k == len(m.path) - 1 or (m.path[k], m.path[k + 1]) not in free))
            key = (m.path[k], m.path[k + 1]) if k < target_idx else None
            if key in m.free_paths:
                ahead += list(m.free_paths[key][1:-1])  # 받은 경로의 안쪽 점
                lane += [False] * (len(ahead) - len(lane))
        if len(ahead) > 1 and math.dist(points[0], ahead[0]) < 1e-9:
            ahead.pop(0)  # 지점 위에 서 있다
            lane.pop(0)
        points += ahead
        lane = [False, *lane]  # 로봇 자리
        # 앞뒤가 모두 레인 지점인 꺾인 곳만 둥글린다. 자유 구역 출입구와 그 경로 점은 둔다.
        # 로봇 자리에서 오는 첫 꺾임도 둔다: 보고 위치의 잡음으로 원호가 레인 밖에서 시작한다
        roundable = [0 < i < len(points) - 1 and lane[i - 1] and lane[i] and lane[i + 1]
                     for i in range(len(points))]
        points = round_corners(points, roundable, self.params.corner_radius_m,
                               self.site.half_width, self._arc_stays_in(points))
        out = []
        for here, there in zip(points, points[1:], strict=False):
            gap = (there[0] - here[0], there[1] - here[1])
            out.append((*here, math.atan2(gap[1], gap[0]) if any(gap) else yaw))
        out.append((*points[-1], yaw))
        return tuple(out)

    def _arc_stays_in(self, points: list):
        """원호를 써도 되나(안전 쪽). 원호 점과 점 사이 가운데가
        - 꺾인 지점이나 앞뒤 점의 구역에 속하고(nearest_waypoint, 다른 구역 로봇처럼 보이지 않게),
        - 꺾인 지점이 이미 닿은 바닥 말고는 어느 구역 바닥(자유 구역 다각형 포함)에도 들지 않는다.
        """
        site = self.site

        def zone_at(x, y):
            return self._zone(site.nearest_waypoint(x, y))

        def floors(x, y):
            return {zone for zone in site.zones if site.distance_to_zone(x, y, zone) <= 0.0}

        def accept(i, arc):
            near = {zone_at(*points[k]) for k in (i - 1, i, i + 1)}
            allowed = near | floors(*points[i])
            mids = [((a[0] + b[0]) / 2, (a[1] + b[1]) / 2) for a, b in zip(arc, arc[1:],
                                                                          strict=False)]
            return all(zone_at(x, y) in near and floors(x, y) <= allowed for x, y in arc + mids)
        return accept

    def _first_ahead(self, m: _Mission, target_idx: int) -> int:
        """선에 넣을 첫 지점. 지났으면 뺀다.

        지났다: 나가는 레인을 따라 arrival_tolerance_m 넘게 갔고, 들어오는 레인보다 나가는 레인에
        가깝다. 그보다 가까우면 넣어 위치 잡음으로 모서리를 가로지르지 않게 한다. 90도 넘게
        꺾이는 곳에서는 다가오는 로봇도 나가는 레인 방향으로 앞서 보이므로 거리도 본다.
        로봇 뒤에 남은 짧은 앞부분은 핑키가 건너뛴다(FollowPath는 가장 가까운 점부터 따른다).
        """
        i = min(m.idx, target_idx)
        if i >= target_idx:
            return i
        out = (m.path[i], m.path[i + 1])
        s, _ = self.site.lane_progress(*out, m.x, m.y)
        if s <= self.params.arrival_tolerance_m:
            return i
        before = self._lane_before(m, i)
        if before and self._gap_to(m, before) < self._gap_to(m, out):
            return i
        return i + 1

    def _round_the_corner(self, m: _Mission) -> bool:
        """경로 인덱스를 넘겨도 되나. path는 들어오는 레인보다 나가는 레인에 가까워야 한다.

        예각에서는 들어오는 레인 위의 로봇도 나가는 레인에 투영하면 d 넘게 앞서 보인다.
        point는 목표가 꺾임에서 끝나 이 경우가 없다.
        """
        return self._corner_turned(m, m.idx)

    def _corner_turned(self, m: _Mission, i: int) -> bool:
        before = self._lane_before(m, i) if self._follows_path else None
        if before is None:
            return True
        return self._gap_to(m, (m.path[i], m.path[i + 1])) <= self._gap_to(m, before)

    def _lane_before(self, m: _Mission, i: int):
        """지점 i로 들어오는 레인. 첫 지점은 경로에 없으니 실제로 있는 레인을 고른다.

        첫 지점으로 들어오는 레인 중 로봇에 가장 가까운 것(두 번째 지점에서 오는 레인은 뺀다).
        서다 만 로봇(N→S 오다 S 앞에 서고 다음이 S→L)도 지점을 지난 것으로 보지 않는다.
        """
        if i > 0:
            return (m.path[i - 1], m.path[i])
        lanes = [key for key in self.site.lanes if key[1] == m.path[0] and key[0] != m.path[1]]
        return min(lanes, key=lambda key: self._gap_to(m, key), default=None)

    def _path_error_lane(self, m: _Mission, first: int) -> tuple:
        """path 목표의 이탈 기준 레인: 진행 인덱스 앞뒤 두 레인 중 가까운 쪽(같으면 뒤).

        꺾임을 넘는 목표에서 "시작점을 지난 마지막 레인"(s >= 0)을 쓰면, 꺾임 앞에서 옆으로
        밀린 보고나 예각에 다가오는 로봇이 다음 레인에 앞서 걸린다. 진행 인덱스는 이 보고로
        _track_progress가 넘길 곳까지 셈한다(해제 거리만큼 지나야 넘고 예각도 막는다).
        """
        k = min(m.idx, m.goal.target_idx)
        while k < m.goal.target_idx:
            s, _ = self.site.lane_progress(m.path[k], m.path[k + 1], m.x, m.y)
            if s <= self.params.release_distance_m or not self._corner_turned(m, k):
                break
            k += 1
        last = min(k, m.goal.target_idx - 1)
        lanes = [(m.path[i], m.path[i + 1]) for i in range(max(first, last - 1), last + 1)]
        before = self._lane_before(m, 0) if last == 0 else None
        if before:
            lanes.insert(0, before)  # 첫 지점 전은 들어오는 레인으로 잰다
        return min(reversed(lanes), key=lambda lane: self._gap_to(m, lane))
