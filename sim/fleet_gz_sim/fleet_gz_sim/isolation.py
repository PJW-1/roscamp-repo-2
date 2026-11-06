"""Gazebo 시뮬을 띄워도 되는 네트워크 설정인지 본다.

시뮬은 실물과 같은 이름(/pinkyN/cmd_vel 등)을 쓰므로 로봇 네트워크(기본 도메인 6)에 붙으면
실물 핑키가 움직일 수 있다. 저장소 규칙(README "실행")대로 도메인 43 이상, 이 컴퓨터 안에서만
찾기, 고정 피어 없음, Fast DDS일 때만 통과시킨다.
"""
from collections.abc import Mapping

MIN_DOMAIN = 43
MAX_DOMAIN = 232  # Fast DDS가 받는 가장 큰 도메인
DDS = "rmw_fastrtps_cpp"


def problems(env: Mapping[str, str]) -> list[str]:
    """격리가 안 된 이유 목록. 비어 있으면 띄워도 된다."""
    found = []
    domain = env.get("ROS_DOMAIN_ID", "")
    if not domain.isdigit() or not MIN_DOMAIN <= int(domain) <= MAX_DOMAIN:
        found.append(f"ROS_DOMAIN_ID={domain!r}: {MIN_DOMAIN}~{MAX_DOMAIN} 사이로 정해야 한다")
    scope = env.get("ROS_AUTOMATIC_DISCOVERY_RANGE", "")
    if scope != "LOCALHOST":
        found.append(f"ROS_AUTOMATIC_DISCOVERY_RANGE={scope!r}: LOCALHOST여야 한다")
    if env.get("ROS_STATIC_PEERS", ""):
        found.append("ROS_STATIC_PEERS가 있다: 다른 컴퓨터와 이어지므로 비워야 한다")
    rmw = env.get("RMW_IMPLEMENTATION", DDS)
    if rmw != DDS:
        found.append(f"RMW_IMPLEMENTATION={rmw!r}: 저장소 기준인 {DDS}만 시험했다")
    return found


def require_isolated(env: Mapping[str, str]) -> None:
    """격리가 안 됐으면 이유를 모두 담아 RuntimeError를 낸다."""
    found = problems(env)
    if found:
        raise RuntimeError("Gazebo 시뮬을 띄우지 않는다:\n  " + "\n  ".join(found))
