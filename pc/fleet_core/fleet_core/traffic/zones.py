"""구역 잠금 기록: 구역마다 소유자 하나와 FIFO 대기열. 이 클래스만 바꾼다.

충돌 쌍인 두 구역은 레인이 너무 가까워 한 번에 한 로봇만 둘 중 하나를 쥘 수 있다.
쌍에서도 먼저 온 순서를 지킨다. 아니면 교차로의 로봇이 반대 길이 바쁜 내내 기다릴 수 있다.
"""
from __future__ import annotations

import math


class ZoneLocks:
    def __init__(self, zone_ids, clock, conflicts=()):
        self._clock = clock
        self._holder: dict[str, str | None] = {z: None for z in zone_ids}
        self._queue: dict[str, list[str]] = {z: [] for z in self._holder}
        self._acquired_at: dict[str, float | None] = {z: None for z in self._holder}
        self._held: dict[str, list[str]] = {}  # 로봇 -> 잡은 순서대로의 구역
        self._conflicts: dict[str, set[str]] = {z: set() for z in self._holder}
        for pair in conflicts:
            one, other = tuple(pair)
            self._conflicts[one].add(other)
            self._conflicts[other].add(one)
        self._arrivals = 0
        self._arrived: dict[tuple[str, str], int] = {}  # (구역, 로봇) -> 요청 순번
        self._added: set[str] = set()  # 실행 중에 더한 구역(add_zone). 이것만 지울 수 있다

    def add_zone(self, zone: str, conflicts=()) -> None:
        """실행 중에 구역을 더한다(버전 2 V2-0, 임시 구역). conflicts는 함께 쥐면 안 되는 구역."""
        if zone in self._holder:
            raise ValueError(f"zone {zone} already exists")
        conflicts = tuple(conflicts)
        unknown = [other for other in conflicts if other not in self._holder]
        if unknown:
            raise ValueError(f"zone {zone}: unknown conflict zones {unknown}")
        self._holder[zone] = None
        self._queue[zone] = []
        self._acquired_at[zone] = None
        self._conflicts[zone] = set(conflicts)
        for other in conflicts:
            self._conflicts[other].add(zone)
        self._added.add(zone)

    def remove_zone(self, zone: str) -> bool:
        """add_zone으로 더한 구역을 지운다. 쥐었거나 기다리는 로봇이 있으면 거절한다."""
        if zone not in self._added:
            raise ValueError(f"zone {zone} was not added at run time")
        if self._holder[zone] is not None or self._queue[zone]:
            return False
        for other in self._conflicts.pop(zone):
            self._conflicts[other].discard(zone)
        del self._holder[zone], self._queue[zone], self._acquired_at[zone]
        self._added.discard(zone)
        return True

    def request(self, robot: str, zone: str) -> bool:
        """구역을 요청한다. 로봇과 구역 사이에 아무도 없을 때만 준다."""
        if self._holder[zone] == robot:
            return True
        queue = self._queue[zone]
        if robot not in queue:
            queue.append(robot)
            self._arrivals += 1
            self._arrived[(zone, robot)] = self._arrivals
        if self.blockers(robot, zone):
            return False
        self._grant(robot, zone)
        return True

    def request_all(self, robot: str, zones) -> bool:
        """여러 구역을 한꺼번에 요청한다(버전 2 F-1). 모두 줄 수 있을 때만 모두 준다.

        하나라도 안 되면 아무것도 주지 않고 모든 구역의 줄에 선 채로 둔다. 반쯤 잡은 채
        기다리지 않으므로 이 요청끼리는 점유 대기가 없고, 줄에 서 있으므로 FIFO와 대기
        그래프(교착 판정)는 지금과 같이 이 로봇을 본다.
        """
        zones = list(dict.fromkeys(zones))
        for zone in zones:
            if self._holder[zone] != robot and robot not in self._queue[zone]:
                self._queue[zone].append(robot)
                self._arrivals += 1
                self._arrived[(zone, robot)] = self._arrivals
        if any(self.blockers(robot, zone) for zone in zones):
            return False
        for zone in zones:
            if self._holder[zone] != robot:
                self._grant(robot, zone)
        return True

    def requests_of(self, robot: str) -> tuple[str, ...]:
        """이 로봇이 줄 서 있는 구역들, 요청 순서대로."""
        return tuple(zone for zone, who in self._arrived if who == robot)

    def blockers(self, robot: str, zone: str) -> set[str]:
        """먼저 비켜야 할 로봇: 소유자, 앞선 대기자, 충돌 쌍 구역의 같은 로봇들."""
        if self._holder[zone] == robot:
            return set()
        found = {self._holder[zone]} if self._holder[zone] else set()
        for waiter in self._queue[zone]:
            if waiter == robot:
                break
            found.add(waiter)
        mine = self._arrived.get((zone, robot), math.inf)
        for other in self._conflicts[zone]:
            if self._holder[other] not in (None, robot):
                found.add(self._holder[other])
            found.update(waiter for waiter in self._queue[other]
                         if waiter != robot and self._arrived[(other, waiter)] < mine)
        return found

    def release(self, robot: str, zone: str) -> bool:
        """구역을 놓는다. 그 로봇이 쥐고 있지 않으면 거절한다."""
        if self._holder[zone] != robot:
            return False
        self._holder[zone] = None
        self._acquired_at[zone] = None
        self._held[robot].remove(zone)
        return True

    def force_acquire(self, robot: str, zone: str) -> bool:
        """초기 배치용. 대기열을 건너뛰고 빈 구역을 잡는다.

        그 구역이나 충돌 쌍 구역을 누가 쥐고 있으면 거절한다.
        """
        if self._holder[zone] == robot:
            return True
        if self.holders_around(zone, robot):
            return False
        self._grant(robot, zone)
        return True

    def holders_around(self, zone: str, robot: str) -> dict[str, str]:
        """이 구역이나 충돌 쌍 구역을 쥔 다른 로봇: 구역 -> 소유자."""
        return {z: self._holder[z] for z in (zone, *sorted(self._conflicts[zone]))
                if self._holder[z] not in (None, robot)}

    def cancel_request(self, robot: str, zone: str) -> bool:
        queue = self._queue[zone]
        if robot not in queue:
            return False
        queue.remove(robot)
        del self._arrived[(zone, robot)]
        return True

    def cancel_all_requests(self, robot: str) -> None:
        for zone in self._queue:
            self.cancel_request(robot, zone)

    def exists(self, zone: str) -> bool:
        return zone in self._holder

    def holder(self, zone: str) -> str | None:
        return self._holder[zone]

    def queue(self, zone: str) -> tuple[str, ...]:
        return tuple(self._queue[zone])

    def held_by(self, robot: str) -> list[str]:
        return list(self._held.get(robot, ()))

    def waiting_for(self, robot: str) -> str | None:
        """기다리는 구역. 로봇은 다른 구역을 쥐지 않을 때만 다음 구역을 요청한다."""
        return next((zone for zone, who in self._arrived if who == robot), None)

    def snapshot(self) -> dict[str, dict]:
        return {
            zone: {
                "holder": self._holder[zone],
                "queue": tuple(self._queue[zone]),
                "acquired_at": self._acquired_at[zone],
            }
            for zone in self._holder
        }

    def _grant(self, robot: str, zone: str) -> None:
        self.cancel_request(robot, zone)
        self._holder[zone] = robot
        self._acquired_at[zone] = self._clock()
        self._held.setdefault(robot, []).append(zone)
