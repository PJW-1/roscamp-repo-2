"""시나리오의 상자(버전 2 F-1). 가짜 경로 계산이 비켜 가는 물체다. 시각과 로봇이 없다."""
from pathlib import Path

import pytest
import yaml
from fake_robots.scenario import ScenarioError, parse_scenario
from fleet_core.traffic.lane_map import load_lane_map

SCENARIOS = Path(__file__).resolve().parents[1] / "scenarios"
MAPS = Path(__file__).resolve().parents[1] / "maps"


def base():
    return yaml.safe_load((SCENARIOS / "free_area_basic.yaml").read_text(encoding="utf-8"))


def site():
    return load_lane_map(MAPS / "feature_map_free.yaml")


def test_boxes_are_read():
    data = base()
    data["boxes"] = [{"x": 1.5, "y": 0.6, "size_m": 0.1}]
    assert parse_scenario(data, site()).boxes == ((1.5, 0.6, 0.1),)


def test_no_boxes_by_default():
    assert parse_scenario(base(), site()).boxes == ()


@pytest.mark.parametrize("box", [{"x": 1.5, "y": 0.6}, {"x": 1.5, "y": 0.6, "size_m": 0},
                                 {"x": 1.5, "y": 0.6, "size_m": 0.1, "at_s": 3}])
def test_bad_boxes_are_refused(box):
    data = base()
    data["boxes"] = [box]
    with pytest.raises(ScenarioError):
        parse_scenario(data, site())


def test_the_box_scenario_parses():
    data = yaml.safe_load((SCENARIOS / "free_area_box.yaml").read_text(encoding="utf-8"))
    scenario = parse_scenario(data, site())
    assert len(scenario.boxes) == 1
