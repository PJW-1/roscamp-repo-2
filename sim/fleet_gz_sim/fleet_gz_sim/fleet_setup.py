"""gz.launch.py가 시나리오에서 정하는 것. ROS 없이 돈다(fleet_core, fake_robots의 순수 파이썬만).

- 핑키를 놓는 자리: 시나리오의 시작 지점. 방향은 그 지점으로 들어오는 레인 방향
  (대기 줄을 따라 들어와 섰다).
- 돌릴 수 없는 시나리오: 로봇 고장이 있는 것(가짜 핑키가 일으키는 고장이라 진짜 pinky_fsm에는
  일으킬 곳이 없다).
- 경기장: 시나리오의 `map`이 없으면 lane_arena(기본 레인 지도). 있으면 sites/<map>.yaml이
  있는 지도는 그 이름의 월드·지도·레인 지도(arena_for), 없는 지도는 lane_arena 월드에 그
  레인 지도만(버전 2 F-2: 그 지도가 벽 안에 들어야 한다. feature_map_crossing은 벽에서
  0.25 m 떨어져 있다).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import yaml
from fake_robots.scenario import ROBOT_FAULTS, load_scenario, scenario_map_file
from fleet_core.traffic.lane_map import load_lane_map


class ScenarioUnfit(RuntimeError):
    """Gazebo 경기장에서 돌릴 수 없는 시나리오."""


def scenario_path(value: str, *shares: str | Path) -> Path:
    """파일 경로이거나, 주어진 패키지 share의 scenarios 폴더에 있는 이름(앞 share가 먼저)."""
    path = Path(value)
    if path.is_file():
        return path
    name = value if value.endswith(".yaml") else value + ".yaml"
    folders = [Path(share) / "scenarios" for share in shares]
    for folder in folders:
        if (folder / name).is_file():
            return folder / name
    where = ", ".join(str(folder) for folder in folders)
    raise ScenarioUnfit(f"scenario {value!r} is nowhere: not a file and not in {where}")


def obstacles_path(value: str, share: str | Path) -> str:
    """gz.launch.py의 obstacles 인자. 비면 장애물 없음, 아니면 파일이나 share/obstacles의 이름."""
    from fleet_gz_sim.obstacles import ScheduleError
    if not value:
        return ""
    if Path(value).is_file():
        return value
    name = value if value.endswith(".yaml") else value + ".yaml"
    path = Path(share) / "obstacles" / name
    if not path.is_file():
        raise ScheduleError(f"obstacle schedule {value!r} is nowhere: not a file and not in "
                            f"{path.parent}")
    return str(path)


@dataclass(frozen=True)
class Arena:
    """시나리오가 돌 경기장: Gazebo 월드, Nav2 지도 둘, 관제 레인 지도(""는 기본 지도)."""
    name: str
    world: str
    nav_map: str
    nav_map_fine: str
    lane_map: str


def arena_for(scenario_file, gz_share: str | Path, fake_share: str | Path) -> Arena:
    """시나리오의 `map`이 없으면 lane_arena와 기본 레인 지도.

    sites/<map>.yaml이 있는 지도는 arena.py가 그것으로 만든 월드와 지도, 그리고 그 site의
    lane_map(월드의 레인 그림과 상자 여유가 그 지도로 만들어졌다). 없는 지도는 lane_arena의 월드와
    지도에 레인 지도만 바꾼다(버전 2 F-2. 그 레인 지도가 lane_arena 벽 안에 들어야 한다). 이때
    레인 지도는 버전 2 scenario_map_file처럼 시나리오 파일 옆 maps 폴더를 먼저, 다음에
    fake_robots/maps를 본다."""
    data = yaml.safe_load(Path(scenario_file).read_text(encoding="utf-8")) or {}
    name = data.get("map") or ""
    gz = Path(gz_share)
    site_file = gz / "sites" / f"{name}.yaml"
    if name and site_file.is_file():
        lane = (yaml.safe_load(site_file.read_text(encoding="utf-8")) or {}).get("lane_map") or ""
        found = Arena(name, str(gz / "worlds" / f"{name}.sdf"), str(gz / "maps" / f"{name}.yaml"),
                      str(gz / "maps" / f"{name}_fine.yaml"),
                      str(Path(fake_share) / "maps" / f"{lane}.yaml") if lane else "")
    else:
        found = Arena("lane_arena", str(gz / "worlds" / "lane_arena.sdf"),
                      str(gz / "maps" / "lane_arena.yaml"),
                      str(gz / "maps" / "lane_arena_fine.yaml"),
                      _lane_map_file(scenario_file, name, gz, fake_share))
    missing = [p for p in (found.world, found.nav_map, found.nav_map_fine, found.lane_map)
               if p and not Path(p).is_file()]
    if missing:
        raise ScenarioUnfit(f"{Path(scenario_file).name} wants map {name!r}, but there is no "
                            f"{', '.join(missing)} (python3 -m fleet_gz_sim.arena --site {name})")
    return found


def _lane_map_file(scenario_file, name: str, gz: Path, fake_share: str | Path) -> str:
    """site가 없는 지도 이름의 레인 지도 파일. 시나리오 옆 maps 폴더(Gazebo 지도 폴더는 빼고), 다음
    fake_robots/maps. 이름이 없으면 ""(기본 지도)."""
    if not name:
        return ""
    beside = Path(scenario_file).resolve().parent.parent / "maps" / f"{name}.yaml"
    if beside.is_file() and beside.parent.resolve() != (gz / "maps").resolve():
        return str(beside)
    return str(Path(fake_share) / "maps" / f"{name}.yaml")


def load(path, lane_map_file: str = ""):
    """(레인 지도, 시나리오). 레인 지도는 lane_map_file, 비면 시나리오의 map(fake_robots의
    지도), 그것도 없으면 기본 지도. 경기장에서 돌릴 수 없으면 ScenarioUnfit."""
    name = Path(path).name
    site = load_lane_map(lane_map_file or scenario_map_file(path) or None)
    scenario = load_scenario(path, site)
    faults = sorted({f.type for f in scenario.faults if f.type in ROBOT_FAULTS})
    if faults:
        raise ScenarioUnfit(f"{name} has robot faults {faults}; only fake pinkys can make them")
    return site, scenario


def start_poses(site, scenario) -> dict[str, tuple[float, float, float]]:
    """로봇마다 (x, y, yaw). 들어오는 레인이 없으면 나가는 레인 방향, 둘 다 없으면 0."""
    poses = {}
    for robot, cfg in scenario.robots.items():
        x, y = site.waypoint_pose(cfg.start)
        lanes = ([lane for lane in site.lanes.values() if lane.b == cfg.start]
                 or [lane for lane in site.lanes.values() if lane.a == cfg.start])
        yaw = math.atan2(lanes[0].uy, lanes[0].ux) if lanes else 0.0
        poses[robot] = (x, y, yaw)
    return poses
