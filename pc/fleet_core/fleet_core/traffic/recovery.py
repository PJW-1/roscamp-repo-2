"""통신 끊김, 복구, 교착 후퇴, 운영자의 confirm_clear.

파일을 짧게 하려고 TrafficController에서 떼어 낸 부분이다. 제어기 상태를 쓰므로 혼자 쓰지 않는다.
공개 메서드는 confirm_clear뿐이고 나머지는 tick에서 부른다.
"""
from __future__ import annotations

import math

from fleet_core.common.commands import CancelCmd, DispatchResult, RetreatCmd
from fleet_core.common.validate import checked, clear_problem, text
from fleet_core.traffic.deadlock import (
    choose_retreat,
    cycle_members,
    find_cycle,
    priority,
    wait_graph,
)
from fleet_core.traffic.lane_map import _point_segment
from fleet_core.traffic.mission import (
    AT_STATION,
    COMM_LOST,
    DOCKING,
    IDLE,
    LOCALIZING,
    OUT_OF_SERVICE,
    PARKED,
    RECOVERING,
    RETREAT,
    _Goal,
    _Mission,
)

# 성공한 물러나기 뒤 확인 시각에 아직 못 놓으면 recover_check_s씩 더 기다리는 횟수(1-A, 추정).
# 위치 잡음으로 한 보고가 바닥 안이어도 해제 확인(연속 3번)을 다시 모을 시간이다.
RETREAT_RECHECKS = 5
# 핑키는 달리던 이동이 끝나고 설 때까지 BackUp을 미룬다(pinky_fsm retreat_stop_wait_s와 같다).
ROBOT_STOP_WAIT_S = 3.0


class TrafficRecovery:
    def confirm_clear(self, robot, zone_ids, note="") -> list[str] | None:
        """운영자가 멈춘 로봇을 해제했다. 자세로 판단하지 않는 유일한 해제다.

        놓은 구역을 돌려주고, 거절하면 아무것도 안 바꾸고 None. 맵에 오른 적 없는 로봇은
        빈 목록이 맞다. 일부 구역만 주면 의도로 보고 경고하며, 나머지는 다시 불러 푼다.
        거부 이유는 clear_reason에 남는다(받아들이면 빈 값).
        """
        note = text(note)
        problem = checked(clear_problem, robot, zone_ids, self._missions, self.zones.exists)
        if problem:
            # 없는 이름은 잘못된 입력이다. 이 로봇에 대한 판단이 아니다.
            self.clear_reason = problem
            name = text(robot) if isinstance(robot, str) else ""
            known = name if name in self._missions else ""
            self._event("WARN", "INPUT_REJECTED", f"confirm_clear refused: {problem}: {note}",
                        known)
            return None
        refusal = self._clear_refusal(robot, zone_ids)
        self.clear_reason = refusal
        if refusal:
            self._event("WARN", "CLEAR_REFUSED", f"{refusal}: {note}", robot)
            return None
        m = self._missions[robot]
        released = [z for z in zone_ids if self.zones.release(robot, z)]
        m.outside.clear()
        self.zones.cancel_all_requests(robot)
        if m.goal:
            self._cancel_goal(m)
        m.fault = OUT_OF_SERVICE
        m.fault_reason = note
        m.comm = ""
        m.called = False
        m.hold = False
        self._event("INFO", "ZONES_CLEARED", f"released {released}: {note}", robot)
        if m.block:
            self._clear_block(m, "cleared by the operator")
        m.block_rejoin = m.block_cancel = None
        still_held = self.zones.held_by(robot)
        if still_held:
            # 이 로봇은 더는 자세로 판단하지 않으니 다른 무엇도 이 구역을 풀지 않는다.
            self._event("WARN", "ZONES_STILL_HELD", f"{still_held} stay locked", robot)
        if m.dispatch:
            self._out.append(DispatchResult(m.dispatch.goal_id, False, robot, "ROBOT_REMOVED"))
            m.dispatch = None
        return released

    def _clear_refusal(self, robot, zone_ids) -> str:
        """이름은 모두 있는데 해제할 수 없는 이유."""
        if not zone_ids and self.zones.held_by(robot):
            return f"no zones listed while it holds {self.zones.held_by(robot)}"
        return ""

    # ---- comm loss and recovery ---------------------------------------------------------

    def _check_comm(self, now: float) -> None:
        for m in self._missions.values():
            if not m.live or m.comm == COMM_LOST:
                continue
            if now - m.last_seen > self.params.comm_timeout_s:
                m.comm = COMM_LOST
                m.called = False
                m.queue_since = None  # 두절 동안 줄을 섰는지 모른다
                # 말 없는 로봇이 뒤 대기자에게 구역이 넘어가는 것을 막으면 안 된다.
                self.zones.cancel_all_requests(m.name)
                what = "pose, LOCALIZING" if m.robot_state == LOCALIZING else "state"
                self._event("WARN", "COMM_LOST", f"no {what} for {now - m.last_seen:.1f} s",
                            m.name)
                # 두절 동안 자가 복구를 이었는지 모른다. 다시 들은 RECOVERING은 새로 잰다
                self._note_self_recovery(m, COMM_LOST)

    def _begin_recovery(self, m: _Mission) -> None:
        m.comm = RECOVERING
        m.recover_at = self._clock() + self.params.recover_check_s
        if m.goal:
            self._cancel_goal(m)
        m.settle_idx = None
        m.arrival_by = None
        self._event("INFO", "COMM_RESTORED", "checking the pose before resuming", m.name)

    def _finish_recoveries(self, now: float) -> None:
        for m in self._missions.values():
            # 마지막 보고가 LOCALIZING이면 옛 자세로 끝내지 않는다.
            # 레인을 벗어난 동안은 자세를 확인할 수 없어 기다린다.
            if (m.live and m.comm == RECOVERING and now >= m.recover_at
                    and m.robot_state != LOCALIZING and not self._still_off_lane(m)):
                self._finish_recovery(m)

    def _finish_recovery(self, m: _Mission) -> None:
        m.comm = ""
        waypoint = self._resume_waypoint(m)
        zone = self._zone(waypoint)
        for held in self.zones.held_by(m.name):
            self._release_if_cleared(m, held)
        if zone not in self.zones.held_by(m.name) and not self.zones.request(m.name, zone):
            self._fail(m, "RECOVERY_CONFLICT")  # 격리도 여기서 걸린다
            self._event("ERROR", "RECOVERY_CONFLICT",
                        f"{m.name} is in {zone} held by {self.zones.holder(zone)}", m.name)
            return
        m.path = [waypoint]
        m.idx = 0
        if m.phase == AT_STATION:
            return
        if m.phase == DOCKING and waypoint == self.site.stations[m.dest].dock:
            return
        m.settle_idx = 0
        if m.dispatch:
            self._start_dispatch_leg(m)
        else:
            m.phase = PARKED if waypoint in self.site.parking else IDLE
            m.dest = None

    def _resume_waypoint(self, m: _Mission):
        """통신이 돌아온 로봇의 위치. 보고 하나는 틀릴 수 있어 쥔 구역으로 정한다.

        달리는 로봇처럼 지점에서 d 안이면 그 지점이다. d를 막 지나면 뒤 구역은 이미 놓았을 수
        있으니, d에 못 미친 보고 하나로 그리로 되돌리면 안 된다.
        """
        waypoint = self.site.nearest_waypoint(m.x, m.y, radius=self.params.release_distance_m)
        held = self.zones.held_by(m.name)
        if self._zone(waypoint) in held:
            return waypoint
        ahead = self.site.nearest_waypoint(m.x, m.y)
        return ahead if self._zone(ahead) in held else waypoint

    # ---- deadlock safety net ------------------------------------------------------------

    def _priority(self, robot: str) -> int:
        """교통이 아는 것만 본다: 스테이션에 있는지, 지정 배차가 있는지."""
        m = self._missions[robot]
        return priority(self.params, m.phase == AT_STATION, m.dispatch is not None)

    def _check_deadlock(self, now: float) -> frozenset[str]:
        """The robots on any wait cycle, empty when there is none (work order H1).

        Only these robots get no new moves while a cycle lasts; robots outside go on, and the
        zone locks keep them apart. One cycle is handled at a time: the one being handled keeps
        its clock while all its wait edges remain, even if another cycle comes and goes; the
        other cycle's robots stand still until it comes up. A cycle with other wait edges, even
        among the same robots, is a new episode. When the retreat could not break the cycle
        (DEADLOCK_UNRESOLVED) and the same cycle lasts deadlock_unresolved_s more, the robot
        choose_retreat picks is isolated (part 5.5).
        """
        graph = wait_graph(self.zones)
        cycle = list(self._deadlock_cycle)
        if not cycle or not all(holder in graph.get(waiter, ())
                                for waiter, holder in zip(cycle, cycle[1:] + cycle[:1],
                                                          strict=True)):
            cycle = find_cycle(graph) or []
            self._deadlock_cycle = tuple(cycle)
            self._deadlock_since = None
            self._deadlock_acted = False
            if not self._retreat_open():
                self._retreating = None  # 열린 물러나기는 순환이 바뀌어도 기한까지 지켜본다
            self._unresolved_at = None
        self._deadlock_members = cycle_members(graph) if cycle else frozenset()
        if not cycle:
            if self._retreat_open() and now >= self._retreating[2]:
                self._cancel_open_retreat()
            return self._deadlock_members
        if self._deadlock_since is None:
            self._deadlock_since = now
            self._event("ERROR", "DEADLOCK_DETECTED", f"wait cycle {cycle}")
        elif (now - self._deadlock_since >= self.params.deadlock_grace_s
              and not self._deadlock_acted):
            self._deadlock_acted = True
            self._retreat(choose_retreat(cycle, self._priority), cycle)
        elif self._retreating and now >= self._retreating[2]:
            self._finish_retreat(cycle)
        elif self._deadlock_acted and not self._retreating and self._unresolved_at is None:
            # The retreat gave up a zone, yet every wait edge of this cycle is still here.
            robot = choose_retreat(cycle, self._priority)
            self._event("ERROR", "DEADLOCK_UNRESOLVED",
                        f"{robot} of {cycle} gave up a zone but the cycle remains", robot)
            self._unresolved_at = now
        elif (self._unresolved_at is not None
              and now - self._unresolved_at >= self.params.deadlock_unresolved_s):
            self._unresolved_at = None
            # _fail cancels the robot's zone requests, so the cycle is gone by the next tick.
            self._fail(self._missions[choose_retreat(cycle, self._priority)],
                       "DEADLOCK_UNRESOLVED")
        return self._deadlock_members

    def _retreat(self, robot: str, cycle: list[str]) -> None:
        """아직 들어가지 않은 구역 예약을 내놓는다. 그런 구역이 없으면 진짜로 물러난다(1-A).

        양보할 로봇이 물러날 수 없으면 순환의 다른 로봇을 우선순위가 낮은 차례로 본다.
        """
        m = self._missions[robot]
        given_up = [zone for zone in self.zones.held_by(robot) if self._off_the_floor(m, zone)]
        if not given_up:
            others = sorted((r for r in cycle if r != robot),
                            key=lambda r: (self._priority(r), r))
            if any(self._back_off(self._missions[r], cycle) for r in [robot, *others]):
                return
        if not given_up:
            self._event("ERROR", "DEADLOCK_UNRESOLVED",
                        f"{robot} of {cycle} has no unentered zone to give up", robot)
            self._unresolved_at = self._clock()
            return
        if m.goal:
            self._cancel_goal(m)
        self._retreating = (robot, given_up, self._clock() + self.params.recover_check_s)
        self._retreat_rechecks = None  # 구역 포기: 바닥 밖인 동안 기다린다(전과 같다)
        self._event("ERROR", "DEADLOCK_RETREAT", f"{robot} gives up {given_up}", robot)

    def _back_off(self, m: _Mission, cycle: list[str]) -> bool:
        """버전 2 1-A(제안): 들어온 레인의 시작 지점까지(최대 retreat_max_m) 곧게 후진한다.

        조건: 경로에 지난 지점이 있고 그 구역이 지금 구역과 다르다. 그 구역을 줄 서지 않고
        바로 잠글 수 있다. 로봇이 그 레인 방향을 retreat_heading_tolerance_deg 안으로 본다
        (Retreat는 보는 방향 그대로 곧게 간다). 물러난 자리가 지금 쥔 구역 바닥 모두에서 해제
        거리만큼 떨어지고, 들어온 레인 바닥(half_width) 위에 있다. 하나라도 아니면 False이고
        아무것도 바꾸지 않는다(그 구역을 전부터 기다렸으면 그 줄도 그대로).
        """
        if not self.params.real_retreat or m.idx == 0:
            return False
        behind, here = m.path[m.idx - 1], m.path[m.idx]
        spot = self._zone(behind)
        held = self.zones.held_by(m.name)
        if spot in held or spot == self._zone(here):
            return False
        bx, by = self.site.waypoint_pose(behind)
        hx, hy = self.site.waypoint_pose(here)
        lane_yaw = math.atan2(hy - by, hx - bx)
        turn = abs(math.atan2(math.sin(m.yaw - lane_yaw), math.cos(m.yaw - lane_yaw)))
        if math.degrees(turn) > self.params.retreat_heading_tolerance_deg:
            return False
        distance = min(math.dist((m.x, m.y), (bx, by)), self.params.retreat_max_m)
        end = (m.x - math.cos(m.yaw) * distance, m.y - math.sin(m.yaw) * distance)
        clear = self.params.release_distance_m - self.site.half_width
        if distance <= 0.0 or any(self.site.distance_to_zone(*end, zone) < clear
                                  for zone in held):
            return False
        if _point_segment(*end, bx, by, hx, hy) > self.site.half_width:
            return False  # 곧게 물러나면 레인 바닥을 벗어난다(다른 구역 바닥일 수 있다)
        waiting = spot in self.zones.requests_of(m.name)
        if not self.zones.request(m.name, spot):
            if not waiting:
                self.zones.cancel_request(m.name, spot)  # 바로 못 잠그면 줄에도 서지 않는다
            return False
        if m.goal:
            self._cancel_goal(m)
        goal_id = self._next_goal_id()
        m.goal = _Goal(RETREAT, goal_id)
        self._out.append(RetreatCmd(m.name, goal_id, distance, self.params.retreat_speed_mps))
        # 로봇과 같은 셈에, 로봇이 먼저 서기를 기다리는 시간을 더한다
        allowance = distance / self.params.retreat_speed_mps * 2.0 + 2.0 + ROBOT_STOP_WAIT_S
        self._retreating = (m.name, held, self._clock() + allowance + self.params.recover_check_s)
        self._retreat_rechecks = RETREAT_RECHECKS
        self._event("ERROR", "DEADLOCK_RETREAT",
                    f"{m.name} of {cycle} backs off {distance:.2f} m to {behind} and gives up "
                    f"{held}", m.name)
        return True

    def _retreat_open(self) -> bool:
        """_retreating의 로봇이 아직 결과가 안 온 Retreat goal을 쥐고 있다."""
        if not self._retreating:
            return False
        goal = self._missions[self._retreating[0]].goal
        return goal is not None and goal.kind == RETREAT

    def _cancel_open_retreat(self) -> None:
        """기한까지 물러나기 결과가 없다. 취소를 한 번 보내고, 끝났다는 결과(또는 잃음)가 올
        때까지 goal을 열어 둔다. 그동안 뒤 구역도 쥔 구역도 놓지 않는다(1-A 재검수 3)."""
        robot, zones, _ = self._retreating
        goal = self._missions[robot].goal
        if self._retreat_cancel != goal.id:
            self._retreat_cancel = goal.id
            self._out.append(CancelCmd(robot, goal.id))
        self._retreating = (robot, zones, self._clock() + self.params.recover_check_s)

    def on_retreat_result(self, robot, goal_id, success, reason="") -> None:
        """Retreat가 끝났다. 성공이면 경로 인덱스를 하나 되돌린다(들어온 레인 위로 돌아갔다).

        구역은 여기서 놓지 않는다. 확인 시각(_finish_retreat)에 벗어난 것만 놓는다.
        """
        m = self._missions.get(robot)
        if m:
            self._cancel_finished(m, goal_id)
        if m is None or m.goal is None or m.goal.id != goal_id or m.goal.kind != RETREAT:
            return
        self._retreat_ended(m, bool(success), reason)

    def _retreat_ended(self, m: _Mission, success: bool, reason: str) -> None:
        m.goal = None
        if success:
            m.idx -= 1
            m.last_result = "SUCCEEDED"
            self._retreat_rechecks = RETREAT_RECHECKS
        else:
            m.last_result = reason
            self._retreat_rechecks = 0
            self._event("WARN", "RETREAT_FAILED", f"retreat ended with {reason or 'no reason'}",
                        m.name)
        if self._retreating and self._retreating[0] == m.name:
            robot, zones, _ = self._retreating
            self._retreating = (robot, zones, self._clock() + self.params.recover_check_s)

    def _finish_retreat(self, cycle: list[str]) -> None:
        """멈춘 로봇이 아직 밖에 있는 구역만 놓는다."""
        robot, zones, _ = self._retreating
        m = self._missions[robot]
        if self._retreat_open():
            self._cancel_open_retreat()
            return
        for zone in zones:
            self._release_if_cleared(m, zone)
        nothing_released = set(zones) <= set(self.zones.held_by(robot))
        if nothing_released:
            if self._retreat_rechecks is None:
                # 구역 포기: 밖에 있지만 보고가 아직 모자라다. 다음 보고로 확인될 수 있다.
                wait = any(self._off_the_floor(m, zone) for zone in zones)
            else:
                # 물러나기(1-A): 성공했으면 몇 번 더 본다(잡음 보고 하나 등). 끝은 있다
                wait = self._retreat_rechecks > 0
                self._retreat_rechecks = max(0, self._retreat_rechecks - 1)
            if wait:
                self._retreating = (robot, zones, self._clock() + self.params.recover_check_s)
                return
        self._retreating = None
        if nothing_released:
            self._event("ERROR", "DEADLOCK_UNRESOLVED",
                        f"{robot} of {cycle} entered {zones} before it stopped", robot)
            self._unresolved_at = self._clock()
