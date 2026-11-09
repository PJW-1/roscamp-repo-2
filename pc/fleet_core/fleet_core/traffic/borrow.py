"""옆 구역 빌리기(버전 2 1-C, 지역 대응 S2~S4). 비켜 가기(B)는 닿는 구역을 모두 이미 쥔 경우다.

막힘(blocking.py)이 열린 채, 이동 goal을 둔 BLOCKED 로봇이 obstacle_wait_s 넘게 그대로면:
  - 막힌 레인의 끝 지점부터 경로를 따라 detour_rejoin_tries개 지점을 다시 붙을 후보로 두고,
    앞 것부터 로봇 planner에 로봇 자리에서 그 지점까지 경로를 청한다(PlanCmd, area "", 로봇의
    compute_path_to_pose). free_plan_wait_s 안에 답이 없거나 답을 못 쓰면 다음 후보다.
  - 쓰는 답: 로봇 자리(lane_error_warn_m 안)에서 시작해 그 지점(arrival_tolerance_m 안)에서 끝나고,
    직선의 detour_max_ratio배 이하이며, 지금 길의 레인에서 half_width 넘게 벗어나고(아니면 planner가
    물체를 못 보는 것이라 같은 길을 다시 보내는 것과 같다), 모든 점이 어떤 구역에 닿는다
    (lane_map.off_floor_points.
    잠그지 않은 바닥으로 나가지 않는다. 임시 구역은 쓰지 않는다). 닿는 구역(zones_touched,
    _off_the_floor와 같은 기준)에 스테이션 도크와 그 앞 구역(no_detour), 자유 구역이 있으면 쓰지
    않는다(안전 쪽: 칸 모드의 자유 구역 이름은 아무도 쥐지 않는다).
  - 닿는 구역 중 쥐지 않은 것을 ZoneLocks.borrow_all로 한꺼번에 빌린다(모두 비고 기다리는 로봇도
    없을 때만, 양보형). 안 되면 매 tick 다시 보고, borrow_wait_s가 지나면 거둔다(DETOUR_REJECTED).
    그동안 막힘 표시(blocked_mark_s)는 미룬다. 거둔 뒤에는 지금의 막힘 처리 그대로다.
  - 빌리기 대기는 잠금 대기열에 없어 wait_graph에 안 든다. 그래서 그 대기를 더하면 순환이 되는지
    (빌리기를 막는 로봇이 대기 그래프에서 이 로봇에 닿는지) 매번 보고, 되면 바로 거둔다
    (물러나기보다 싸다).
  - 빌리면(닿는 구역이 모두 이미 쥔 것이면 바로) 그 경로를 DETOUR goal(MoveCmd.path)로 보내고 막힘을
    닫는다. 다른 요청은 모두 거둔다. 우회하는 동안은 닿는 구역과 시작 때 쥔 구역을 모두 쥔다
    (m.borrowed, _needed_zones). 단계도 진행하지 않는다(controller._step_phase).
  - 성공하면 경로 인덱스를 다시 붙은 지점으로 옮기고(DETOUR_DONE) 빌린 구역을 해제 규칙에 넘긴다.
    경로 위 지점에 섰으므로 지금·다음 구역이 아닌 빌린 구역은 바로 놓는다(_return_shadowed).
    놓이면 ZONE_RETURNED. 실패·거절·멈춤이면 길 밖이라 구역을 쥔 채 격리한다(DETOUR_FAILED).
  - 표시한 뒤에도(앞 구역을 놓고 선 뒤) borrow_wait_s마다 다시 시도한다. 반대 차선 로봇이 놓인
    앞 구역으로 지나간 뒤 빌릴 수 있다. 우회를 보내면 막힘 표시를 지운다(LANE_CLEARED).
  - 우회 중 다시 막히면 우회하지 않는다(DETOUR goal에는 막힘을 열지 않는다). 멈춤 감시가 잡는다.

TrafficController에서 떼어 낸 부분이다. 제어기 상태를 쓰므로 혼자 쓰지 않는다.
"""
from __future__ import annotations

import math

from fleet_core.common.commands import MoveCmd, PlanCmd
from fleet_core.traffic.deadlock import wait_graph
from fleet_core.traffic.lane_map import _point_segment
from fleet_core.traffic.mission import BLOCKED, DETOUR, MOVE, _Detour, _Goal, _Mission


def _length(points) -> float:
    return sum(math.dist(a, b) for a, b in zip(points, points[1:], strict=False))


class TrafficBorrow:
    def _no_detour_zones(self) -> set:
        """비켜 가지 않는 구역: 스테이션 도크와 도크로 들어오는 레인의 앞 구역(기본 선택 5번)."""
        out = set()
        for station in self.site.stations.values():
            out.add(self._zone(station.dock))
            out.update(self._zone(a) for a, b in station.queue if b == station.dock)
        return out

    def _detour_targets(self, m: _Mission) -> list:
        lane = m.block.lane
        if m.path[m.idx] == lane[1]:
            first = m.idx
        elif m.idx + 1 < len(m.path) and m.path[m.idx + 1] == lane[1]:
            first = m.idx + 1
        else:
            return []
        return list(range(first, min(first + self.params.detour_rejoin_tries, len(m.path))))

    def _watch_detour(self, m: _Mission, now: float) -> bool:
        """막힘이 열린 tick마다. 빌리기를 기다리는 중이면 True(그동안만 막힘 표시를 미룬다).

        경로를 묻는 동안은 미루지 않는다. 그 사이 표시하면 묻던 시도를 거두고, 앞 구역을 놓고 선 뒤
        곧 다시 묻는다.
        """
        b = m.block
        d = m.detour
        if d is not None and (not m.live or m.comm):
            self._reject_detour(m, "the robot is not available")
            return False
        if d is None:
            if not self.params.borrow_enabled or not self._follows_path:
                return False
            if b.marked:  # 앞 구역을 놓고 선 뒤 borrow_wait_s마다
                if (m.block_cancel is not None
                        or b.still_n < self.params.release_confirm_samples
                        or (b.detour_ended is not None
                            and now - b.detour_ended < self.params.borrow_wait_s)):
                    return False
            elif (b.detour_tried or b.total < self.params.obstacle_wait_s
                    or m.robot_state != BLOCKED or not (m.goal and m.goal.kind == MOVE)):
                return False
            b.detour_tried = True
            targets = self._detour_targets(m)
            if not targets:
                b.detour_ended = now
                return False
            m.detour = _Detour(targets, now)
            self._ask_detour(m, now)
            return False
        if d.plan_id is not None:
            if now - d.asked_at >= self.params.free_plan_wait_s:
                self._next_detour_target(
                    m, f"no answer in {self.params.free_plan_wait_s:.0f} s", now)
            return False
        self._try_borrow(m, now)
        return m.detour is not None

    def _ask_detour(self, m: _Mission, now: float) -> None:
        d = m.detour
        target = d.targets[0]
        gx, gy = self.site.waypoint_pose(m.path[target])
        if target + 1 < len(m.path):
            nx, ny = self.site.waypoint_pose(m.path[target + 1])
        else:
            nx, ny = gx + (gx - m.x), gy + (gy - m.y)
        d.plan_id = self._next_goal_id()
        d.asked_at = now
        self._out.append(PlanCmd(m.name, d.plan_id, "", m.x, m.y, gx, gy,
                                 math.atan2(ny - gy, nx - gx)))

    def _next_detour_target(self, m: _Mission, why: str, now: float) -> None:
        d = m.detour
        d.targets.pop(0)
        d.plan_id = None
        d.why.append(why)
        if not d.targets:
            self._reject_detour(m, "; ".join(d.why))
            return
        self._ask_detour(m, now)

    def _reject_detour(self, m: _Mission, why: str) -> None:
        self.zones.withdraw(m.name)
        m.detour = None
        if m.block is not None:
            m.block.detour_ended = self._clock()
        self._event("WARN", "DETOUR_REJECTED", why, m.name)

    def _detour_answer(self, m: _Mission, points) -> None:
        """경로 계산의 답(on_plan_result에서). 쓸 수 있으면 빌리기 대기로 넘어간다."""
        d = m.detour
        d.plan_id = None
        target = d.targets[0]
        problem, line, touched = self._detour_problem(m, points, target)
        now = self._clock()
        if problem:
            self._next_detour_target(m, f"to {self.site.waypoint_id(m.path[target])}: {problem}",
                                     now)
            return
        d.points, d.target_idx, d.touched = tuple(line), target, frozenset(touched)
        d.borrow_since = now
        self._event("INFO", "DETOUR_PLANNED",
                    f"to {self.site.waypoint_id(m.path[target])}: {len(line)} points, "
                    f"{_length(line):.2f} m, touches {sorted(touched)}", m.name)

    def _detour_problem(self, m: _Mission, points, target: int) -> tuple[str, list, set]:
        if not points or len(points) < 2:
            return "no path", [], set()
        goal = self.site.waypoint_pose(m.path[target])
        if math.dist(points[0], (m.x, m.y)) > self.params.lane_error_warn_m:
            return "does not start at the robot", [], set()
        if math.dist(points[-1], goal) > self.params.arrival_tolerance_m:
            return "does not end at the rejoin point", [], set()
        line = [(float(x), float(y)) for x, y in points[:-1]] + [goal]
        limit = self.params.detour_max_ratio * math.dist((m.x, m.y), goal)
        if _length(line) > limit:
            return f"{_length(line):.2f} m is longer than {limit:.2f} m", [], set()
        if self._stays_on_lanes(m, line, target):
            return "does not leave the lanes (the planner sees no way round)", [], set()
        sample, release = self.params.detour_sample_m, self.params.release_distance_m
        off = self.site.off_floor_points(line, sample, release, cells=self._cells_on)
        if off:
            x, y = off[0]
            return f"leaves the lane floors at ({x:.2f}, {y:.2f})", [], set()
        touched = self.site.zones_touched(line, sample, release, cells=self._cells_on)
        touched -= self._near_the_stand(line, goal, touched)
        areas = sorted(touched & set(self.site.free_areas))
        if areas:
            return f"touches the free area {areas[0]}", [], set()
        stations = sorted(touched & self._no_detour_zones())
        if stations:
            return f"touches the station zone {stations[0]}", [], set()
        return "", line, touched

    def _near_the_stand(self, line, goal, touched) -> set:
        """다시 붙을 지점에 선 로봇도 닿는 구역 중, 경로가 그 지점보다 가까이 가지 않는 것.

        그 지점은 경로 위의 평소 서는 자리이고, 거기 선 로봇은 그 이웃 구역을 쥐지 않는다(거리는
        충돌 쌍이 지킨다). 경로가 그보다 가까이 가지 않으면 빌리지 않는다. 빌리면 다시 붙은 뒤 그
        구역을 해제 거리 안에서 쥔 채 다음 구역을 청하게 되어, 그 구역을 기다리는 로봇과 순환이
        생긴다(blockers의 충돌 쌍 대기자 규칙, SimWorld two_lane_avenue_wide의 BE와 BW)."""
        samples, _ = self.site._samples(line, self.params.detour_sample_m)
        out = set()
        for zone in touched:
            if self.site.distance_to_zone(*goal, zone) <= 1e-9:
                continue  # 그 지점이 이 구역 바닥 위다(다시 붙을 지점의 구역 등). 늘 쥔다
            at_stand = self.site._touch_gap(*goal, zone)
            if at_stand < self.params.release_distance_m and all(
                    self.site._touch_gap(x, y, zone) >= at_stand - 1e-9 for x, y in samples):
                out.add(zone)
        return out

    def _stays_on_lanes(self, m: _Mission, line, target: int) -> bool:
        """경로가 지금 길의 레인에서 half_width 넘게 벗어나지 않는다(같은 길을 다시 보내는 것)."""
        lanes = [self.site.lane(m.path[k], m.path[k + 1])
                 for k in range(max(m.idx - 1, 0), target)]
        if not lanes:
            return False
        return all(min(_point_segment(x, y, ln.ax, ln.ay, ln.bx, ln.by) for ln in lanes)
                   <= self.site.half_width for x, y in line)

    def _try_borrow(self, m: _Mission, now: float) -> None:
        d = m.detour
        held = set(self.zones.held_by(m.name))
        wanted = sorted(d.touched - held)
        if wanted and self._borrow_closes_cycle(m, wanted):
            self._reject_detour(m, f"borrowing {wanted} would close a wait cycle")
            return
        if not wanted or self.zones.borrow_all(m.name, wanted):
            self._start_detour(m, wanted, held)
            return
        if now - d.borrow_since >= self.params.borrow_wait_s:
            self._reject_detour(m, f"{wanted} not free in {self.params.borrow_wait_s:.0f} s")

    def _borrow_closes_cycle(self, m: _Mission, wanted) -> bool:
        """빌리기 대기(이 로봇 -> 막는 로봇들)를 대기 그래프에 더하면 순환인가."""
        graph = wait_graph(self.zones)
        seen, todo = set(), list(self.zones.borrow_blockers(m.name, wanted))
        while todo:
            robot = todo.pop()
            if robot == m.name:
                return True
            if robot not in seen:
                seen.add(robot)
                todo.extend(graph.get(robot, ()))
        return False

    def _start_detour(self, m: _Mission, wanted: list, held: set) -> None:
        d, m.detour = m.detour, None
        self.zones.cancel_all_requests(m.name)  # 우회하는 동안은 쥔 채 기다리지 않는다
        m.wait_zone = None
        m.borrowed = set(d.touched) | held
        m.lent |= set(wanted)
        block = m.block.lane if m.block else None
        if wanted:
            where = f" round an obstacle on {block[0]} to {block[1]}" if block else ""
            self._event("INFO", "ZONE_BORROWED", f"{', '.join(wanted)}{where}", m.name)
        target = m.path[d.target_idx]
        x, y = self.site.waypoint_pose(target)
        line = d.points
        yaws = [math.atan2(b[1] - a[1], b[0] - a[0]) for a, b in zip(line, line[1:], strict=False)]
        yaws.append(yaws[-1])
        goal_id = self._next_goal_id()
        m.goal = _Goal(DETOUR, goal_id, d.target_idx)
        m.detour_from = (m.x, m.y)
        m.stall_from = None
        final = not self.params.mark_final_goal or d.target_idx == len(m.path) - 1
        self._out.append(MoveCmd(m.name, goal_id, self.site.waypoint_id(target), x, y, yaws[-1],
                                 tuple((px, py, yaw) for (px, py), yaw in zip(line, yaws,
                                                                              strict=True)),
                                 final))
        if m.block is not None:
            self._clear_block(m, "is passed by a detour")

    def _detour_result(self, m: _Mission, goal: _Goal, success: bool, reason: str) -> None:
        m.goal = None
        m.not_sent_warned = False
        m.last_result = "SUCCEEDED" if success else reason
        if not success:
            self._detour_failed(m, reason)
            return
        m.idx = max(m.idx, goal.target_idx)
        m.settle_idx = None  # 막힘 표시 때 취소한 goal의 목표는 이제 뜻이 없다
        m.attempts = 0
        m.borrowed = set()
        m.detour_from = None
        self._event("INFO", "DETOUR_DONE",
                    f"back on the path at {self.site.waypoint_id(m.path[m.idx])}", m.name)
        self._return_shadowed(m)

    def _return_shadowed(self, m: _Mission) -> None:
        """다시 붙은 뒤 지금·다음 구역이 아닌 빌린 구역을 바로 놓는다.

        로봇은 경로 위 지점(도착 허용치 안)에 섰다. 평소 그 자리의 로봇은 그 지점 구역(과 다음
        구역)만 쥐고, 이웃 구역과의 거리는 충돌 쌍(2r)이 지킨다. 빌린 구역을 해제 규칙(바닥에서
        해제 거리)까지 쥐고 있으면, 그 지점이 이웃 바닥에서 해제 거리 안일 때 다음 구역 요청이 그
        구역을 기다리는 로봇 뒤에 서서(blockers의 충돌 쌍 대기자 규칙) 순환이 생긴다(SimWorld
        two_lane_avenue에서 BE에 다시 붙은 로봇이 BW를 쥔 채 R을 기다리고, O의 로봇이 BW를
        기다림). 2r보다 가까운 바닥은 해제 규칙에 맡긴다."""
        needed = self._needed_zones(m)
        least = 2 * self.params.robot_radius_m  # 그보다 가까운 바닥은 해제 규칙에 맡긴다
        for zone in sorted(m.lent - needed):
            if (self.site._touch_gap(m.x, m.y, zone) >= least
                    and self.zones.release(m.name, zone)):
                m.outside.pop(zone, None)

    def _detour_failed(self, m: _Mission, reason: str) -> None:
        """우회가 끝나지 못했다. 로봇은 길 밖일 수 있어 빌린 구역까지 쥔 채 격리한다.

        보낸 자리에서 lane_error_warn_m 안이면 아직 제 레인 위다. 이동 실패 한 번으로 세고,
        지금·다음 구역이 아닌 빌린 구역은 해제 규칙에 넘긴다(다시 막히면 막힘 처리).
        """
        m.goal = None
        start, m.detour_from = m.detour_from, None
        if start is not None and math.dist((m.x, m.y), start) <= self.params.lane_error_warn_m:
            m.borrowed = set()
            self._event("WARN", "DETOUR_FAILED", f"{reason}, never left the lane", m.name)
            self._return_shadowed(m)
            self._goal_failed(m, reason, announce=False)
            return
        self._event("ERROR", "DETOUR_FAILED",
                    f"{reason}, holding {sorted(self.zones.held_by(m.name))}", m.name)
        self._fail(m, "DETOUR_FAILED")

    def _watch_returns(self, m: _Mission) -> None:
        if not m.lent:
            return
        held = set(self.zones.held_by(m.name))
        for zone in sorted(m.lent - held):
            m.lent.discard(zone)
            self._event("INFO", "ZONE_RETURNED", zone, m.name)
