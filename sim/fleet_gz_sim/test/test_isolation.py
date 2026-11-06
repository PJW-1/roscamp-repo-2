"""격리 검사 시험. Gazebo 시뮬은 실물과 같은 /pinkyN/... 이름을 쓰므로
로봇 네트워크에 붙으면 안 된다.
"""
import pytest
from fleet_gz_sim.isolation import problems, require_isolated

SAFE = {"ROS_DOMAIN_ID": "43", "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST"}


def with_(**changes):
    env = dict(SAFE)
    for key, value in changes.items():
        if value is None:
            env.pop(key, None)
        else:
            env[key] = value
    return env


def test_isolated_setting_passes():
    assert problems(SAFE) == []
    assert problems(with_(RMW_IMPLEMENTATION="rmw_fastrtps_cpp", ROS_STATIC_PEERS="")) == []


@pytest.mark.parametrize("domain", [None, "", "abc", "6", "42", "233"])
def test_domain_must_be_set_and_away_from_the_robot_network(domain):
    found = problems(with_(ROS_DOMAIN_ID=domain))
    assert len(found) == 1 and "ROS_DOMAIN_ID" in found[0]


@pytest.mark.parametrize("scope", [None, "SUBNET", "SYSTEM_DEFAULT", "localhost"])
def test_discovery_must_stay_on_this_computer(scope):
    found = problems(with_(ROS_AUTOMATIC_DISCOVERY_RANGE=scope))
    assert len(found) == 1 and "ROS_AUTOMATIC_DISCOVERY_RANGE" in found[0]


def test_static_peers_reach_other_computers():
    found = problems(with_(ROS_STATIC_PEERS="192.168.0.10"))
    assert len(found) == 1 and "ROS_STATIC_PEERS" in found[0]


def test_only_the_repo_dds_is_accepted():
    found = problems(with_(RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"))
    assert len(found) == 1 and "RMW_IMPLEMENTATION" in found[0]


def test_every_problem_is_reported_at_once():
    assert len(problems({"ROS_STATIC_PEERS": "10.0.0.1"})) == 3


def test_require_isolated_refuses_with_the_reasons():
    with pytest.raises(RuntimeError, match="ROS_DOMAIN_ID"):
        require_isolated(with_(ROS_DOMAIN_ID="6"))
    require_isolated(SAFE)
