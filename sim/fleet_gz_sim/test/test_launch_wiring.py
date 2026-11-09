"""gz.launch.py: 격리 검사를 먼저 하고, 핑키 3대와 진짜 관제, 가짜 central·팔을 잇는 것을 본다."""
import importlib.util
import math
from pathlib import Path

import pytest

launch = pytest.importorskip("launch")
rclpy = pytest.importorskip("rclpy")

LAUNCH = Path(__file__).parents[1] / "launch" / "gz.launch.py"
FAST_DDS = ("ROS_STATIC_PEERS", "RMW_IMPLEMENTATION", "ROS_DISCOVERY_SERVER",
            "FASTRTPS_DEFAULT_PROFILES_FILE", "FASTDDS_DEFAULT_PROFILES_FILE",
            "FASTDDS_ENVIRONMENT_FILE", "FASTDDS_BUILTIN_TRANSPORTS")
ROBOTS = ["pinky1", "pinky2", "pinky3"]
STARTS = {"pinky1": (1.20, 2.05, 0.0), "pinky2": (0.80, 2.05, math.atan2(0.10, 0.45)),
          "pinky3": (0.35, 1.95, math.pi / 2)}  # basic_orders: P-6, P-5, P-4


@pytest.fixture
def gz(monkeypatch, tmp_path):
    spec = importlib.util.spec_from_file_location("gz_launch", LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setenv("ROS_DOMAIN_ID", "44")
    monkeypatch.setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
    for name in FAST_DDS:
        monkeypatch.delenv(name, raising=False)
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(module, "get_rmw_implementation_identifier", lambda: "rmw_fastrtps_cpp")
    return module


def test_the_rmw_comes_from_rclpy():
    spec = importlib.util.spec_from_file_location("gz_launch_plain", LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert module.get_rmw_implementation_identifier is rclpy.get_rmw_implementation_identifier


def run(gz, **overrides):
    """ros2 launch처럼 인자 기본값을 넣고(overrides로 바꾸고) OpaqueFunction을 실행한다."""
    context = launch.LaunchContext()
    *args, start = gz.generate_launch_description().entities
    for action in args:
        action.visit(context)
    context.launch_configurations.update(overrides)
    return start.execute(context)


def test_launch_reads_the_current_environment(gz, monkeypatch):
    monkeypatch.setenv("ROS_DOMAIN_ID", "6")
    with pytest.raises(RuntimeError, match="ROS_DOMAIN_ID"):
        run(gz)


def test_launch_checks_the_loaded_rmw(gz, monkeypatch):
    monkeypatch.setattr(gz, "get_rmw_implementation_identifier", lambda: "rmw_cyclonedds_cpp")
    with pytest.raises(RuntimeError, match="rmw_cyclonedds_cpp"):
        run(gz)


def test_launch_checks_the_working_directory(gz, tmp_path):
    (tmp_path / "DEFAULT_FASTRTPS_PROFILES.xml").write_text("<dds/>")
    with pytest.raises(RuntimeError, match="DEFAULT_FASTRTPS_PROFILES.xml"):
        run(gz)


# -- 배선: launch 동작 대신 만든 인자를 기록해 본다 ------------------------------------------

KINDS = ("Node", "IncludeLaunchDescription", "PythonLaunchDescriptionSource", "ExecuteProcess",
         "GroupAction", "SetRemap", "RegisterEventHandler", "OnProcessExit")


class Made:
    def __init__(self, kind, args, kwargs):
        self.kind, self.args, self.kw = kind, args, kwargs

    def flat(self):
        yield self
        if self.kind == "GroupAction":
            for child in self.args[0]:
                yield from (child.flat() if isinstance(child, Made) else [child])


def need(*packages):
    from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
    for package in packages:
        try:
            get_package_share_directory(package)
        except PackageNotFoundError:
            pytest.skip(f"{package} is not installed")


@pytest.fixture
def made(gz, monkeypatch):
    need("pinky_description", "pinky_nav_config", "pinky_fsm", "fake_robots", "fleet_manager")
    from fleet_gz_sim import launch_parts
    for module in (gz, launch_parts):
        for kind in KINDS:
            if hasattr(module, kind):
                monkeypatch.setattr(module, kind, lambda *a, _k=kind, **kw: Made(_k, a, kw))
    return gz


def flat(actions):
    for a in actions:
        yield from (a.flat() if isinstance(a, Made) else [a])


def nodes(actions, executable=None):
    return [a for a in flat(actions) if isinstance(a, Made) and a.kind == "Node"
            and (executable is None or a.kw["executable"] == executable)]


def params(node) -> dict:
    out = {}
    for p in node.kw.get("parameters", []):
        out.update(p)
    return out


def includes(actions, launch_file):
    return [a for a in flat(actions) if isinstance(a, Made) and a.kind == "IncludeLaunchDescription"
            and str(a.args[0].args[0]).endswith(launch_file)]


def launch_args(include) -> dict:
    return dict(include.kw["launch_arguments"])


def test_three_pinkys_start_where_the_scenario_puts_them(made):
    actions = run(made)
    spawned = {}
    for create in nodes(actions, "create"):
        a = create.kw["arguments"]
        spawned[a[a.index("-name") + 1]] = tuple(float(a[a.index(k) + 1])
                                                 for k in ("-x", "-y", "-Y"))
    assert spawned.keys() == STARTS.keys()
    for robot, pose in STARTS.items():
        assert spawned[robot] == pytest.approx(pose)
    navs = {launch_args(i)["namespace"]: launch_args(i)
            for i in includes(actions, "nav2.launch.py")}
    assert navs.keys() == STARTS.keys()
    for robot, nav in navs.items():  # AMCL 시작 자세가 놓은 자리와 같다
        assert tuple(float(nav[k]) for k in ("initial_x", "initial_y", "initial_yaw")) == \
            pytest.approx(spawned[robot])


def test_gazebo_time_and_every_pinky_share_one_bridge(made):
    """/clock을 브리지 여럿이 같이 내면 표본 순서가 섞여 시간이 뒤로 갈 수 있다.
    브리지를 로봇마다 두면 DDS 참여자도 그만큼 는다(test_ros_process_budget_...)."""
    [bridge] = nodes(run(made), "parameter_bridge")
    topics = [str(x).split("@")[0] for x in bridge.kw["arguments"]]
    assert topics.count("/clock") == 1
    for robot in ROBOTS:
        for name in ("scan", "odom", "tf", "joint_states", "cmd_vel", "truth"):
            assert topics.count(f"/model/{robot}/{name}") == 1
    remaps = dict(bridge.kw["remappings"])
    assert remaps["/model/pinky2/cmd_vel"] == "/pinky2/cmd_vel"
    assert remaps["/model/pinky3/truth"] == "/pinky3/sim/truth"


def declared(launch_file) -> set:
    from ament_index_python.packages import get_package_share_directory
    package = {"nav2.launch.py": "pinky_nav_config", "pinky_fsm.launch.py": "pinky_fsm"}
    path = Path(get_package_share_directory(package[launch_file])) / "launch" / launch_file
    spec = importlib.util.spec_from_file_location(launch_file.replace(".", "_"), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return {a.name for a in module.generate_launch_description().entities
            if type(a).__name__ == "DeclareLaunchArgument"}


def test_arguments_do_not_leak_into_the_robot_launches(gz):
    """include는 인자 범위를 나누지 않는다. gz.launch.py의 params_file(빈 값)이 Nav2의
    params_file 기본값을 덮어 launch가 바로 끝났다. 같은 뜻으로 넘기는 값만 겹쳐도 된다."""
    need("pinky_nav_config", "pinky_fsm")
    shared = {"heartbeat_timeout_s"}  # pinky_fsm에 같은 값을 그대로 넘긴다
    for launch_file in ("nav2.launch.py", "pinky_fsm.launch.py"):
        assert set(gz.ARGS) & declared(launch_file) <= shared, launch_file


def test_nav2_runs_composed_unless_asked_not_to(made):
    """한 대의 Nav2 노드 9개를 프로세스 하나에 담는다(pinky_nav_config use_composition)."""
    composed = [launch_args(i)["use_composition"] for i in includes(run(made), "nav2.launch.py")]
    assert composed == ["true"] * 3
    separate = [launch_args(i)["use_composition"]
                for i in includes(run(made, nav_composition="false"), "nav2.launch.py")]
    assert separate == ["false"] * 3


NAV2_NODES = 9  # pinky_nav_config: map_server 둘, amcl, 서버 넷, lifecycle manager 둘
LOCALHOST_PARTICIPANTS = 32  # Fast DDS LOCALHOST 탐색에서 서로 보이는 참여자 수(실측)
SPARE = 6  # launch 프로세스, RViz, 사람이 띄우는 ros2 명령 몇 개


def ros_processes(actions, fleet_actions) -> int:
    """ROS 프로세스 하나가 DDS 참여자 하나다. Gazebo 서버와 화면은 ROS가 아니다."""
    count = len(nodes(actions)) + len(nodes(fleet_actions))
    for nav in includes(actions, "nav2.launch.py"):
        count += 1 if launch_args(nav)["use_composition"] == "true" else NAV2_NODES
    return count + len(includes(actions, "pinky_fsm.launch.py"))


def test_ros_process_budget_leaves_room_under_the_localhost_limit(made):
    """3대를 처음 띄웠을 때 참여자가 32개를 넘어 pinky3의 토픽이 보이지 않았다.

    launch가 만드는 프로세스를 세는 예산 계산이다. 실제로 서로 보이는지는 Gazebo를 띄워
    확인해야 한다(README). 잠깐 돌고 끝나는 create와 wait_ready도 넉넉하게 같이 센다."""
    from ament_index_python.packages import get_package_share_directory
    path = Path(get_package_share_directory("fake_robots")) / "scenarios" / "basic_orders.yaml"
    fleet = made.fleet(str(path), ROBOTS, result_dir="out", monitor=True,
                       obstacles_file="roam.yaml")  # 장애물 노드까지 띄운 가장 많은 경우
    assert ros_processes(run(made), fleet) + SPARE <= LOCALHOST_PARTICIPANTS
    assert ros_processes(run(made, nav_composition="false"), fleet) > LOCALHOST_PARTICIPANTS


def dock_aside_groups(actions, robot):
    return [a for a in flat(actions) if isinstance(a, Made) and a.kind == "GroupAction"
            and any(isinstance(c, Made) and c.kind == "SetRemap"
                    and c.args == (f"/{robot}/dock", f"/{robot}/dock_fsm") for c in a.flat())]


def test_the_real_pinky_fsm_answers_dock_by_default(made):
    """Work order H8: the real pinky_fsm fails docking with DOCK_UNAVAILABLE unless the sim-only
    stub is asked for. The stand-in still relays the true pose for the scenario monitor."""
    actions = run(made)
    fsms = {launch_args(i)["namespace"] for i in includes(actions, "pinky_fsm.launch.py")}
    assert fsms == set(ROBOTS)
    for robot in ROBOTS:
        assert dock_aside_groups(actions, robot) == []
    assert not [a for a in flat(actions) if isinstance(a, Made) and a.kind == "SetRemap"]
    [stand_in] = nodes(actions, "sim_stand_in")
    assert params(stand_in)["dock_stub"] is False
    assert params(stand_in)["robots"] == ROBOTS and params(stand_in)["use_sim_time"] is True


def test_sim_dock_stub_moves_the_pinky_fsm_dock_aside_and_turns_the_stub_on(made):
    actions = run(made, sim_dock_stub="true")
    for robot in ROBOTS:
        [group] = dock_aside_groups(actions, robot)
        [fsm] = includes([group], "pinky_fsm.launch.py")
        assert launch_args(fsm)["namespace"] == robot
    [stand_in] = nodes(actions, "sim_stand_in")
    assert params(stand_in)["dock_stub"] is True and params(stand_in)["use_sim_time"] is True


def test_sim_dock_stub_still_needs_an_isolated_network(made, monkeypatch):
    monkeypatch.setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "SUBNET")
    with pytest.raises(RuntimeError, match="LOCALHOST"):
        run(made, sim_dock_stub="true")


@pytest.mark.parametrize("name", ["sim_dock_stub", "fleet_check", "gui", "monitor",
                                  "nav_composition"])
def test_a_misspelt_switch_stops_the_launch(made, name):
    with pytest.raises(ValueError, match=name):
        run(made, **{name: "ture"})


def test_fleet_check_records_into_the_result_dir_on_gazebo_time(made):
    from ament_index_python.packages import get_package_share_directory
    path = Path(get_package_share_directory("fake_robots")) / "scenarios" / "basic_orders.yaml"
    [check] = nodes(made.fleet(str(path), ROBOTS, result_dir="out", monitor=True),
                    "fleet_check")
    assert params(check).items() >= {"use_sim_time": True, "robots": ROBOTS,
                                     "output": str(Path("out") / "fleet_check.json")}.items()
    assert not nodes(made.fleet(str(path), ROBOTS, result_dir="out", monitor=True,
                                check=False), "fleet_check")


def test_fleet_check_follows_the_launch_switch(made, monkeypatch):
    seen = {}
    real = made.fleet

    def spy(*args, **kw):
        seen["check"] = args[6] if len(args) > 6 else kw.get("check", True)
        return real(*args, **kw)

    monkeypatch.setattr(made, "fleet", spy)
    for value, expected in (("true", True), ("false", False)):
        actions = run(made, fleet_check=value)
        [on_exit] = [a.args[0].kw["on_exit"] for a in flat(actions) if isinstance(a, Made)
                     and a.kind == "RegisterEventHandler"
                     and callable(a.args[0].kw["on_exit"])]  # the one after wait_ready
        on_exit(type("Exit", (), {"returncode": 0})(), None)
        assert seen["check"] is expected


def test_the_fleet_waits_until_every_pinky_is_ready(made):
    """주문은 시나리오 시간(Gazebo 시계)으로 나간다. 로봇이 서기 전에 관제가 명령하지 않게 한다."""
    actions = run(made)
    [wait] = nodes(actions, "wait_ready")
    assert params(wait)["robots"] == ROBOTS
    assert not nodes(actions, "fleet_manager_node")  # 기다린 뒤에 띄운다


def test_the_real_fleet_manager_runs_with_fake_central_and_arms_on_gazebo_time(made):
    from ament_index_python.packages import get_package_share_directory
    path = Path(get_package_share_directory("fake_robots")) / "scenarios" / "basic_orders.yaml"
    actions = made.fleet(str(path), ROBOTS, result_dir="out", monitor=True)
    [fleet] = nodes(actions, "fleet_manager_node")
    assert params(fleet)["robots"] == ROBOTS and params(fleet)["use_sim_time"] is True
    fakes = [n for n in nodes(actions) if n.kw["package"] == "fake_robots"]
    assert sorted(n.kw["executable"] for n in fakes) == sorted(
        ["fake_workcell_node", "fake_vision_node", "fake_vision_node", "fake_arm_node",
         "fake_central_node", "scenario_monitor_node"])  # 가짜 핑키와 sim_clock은 없다
    for n in fakes:
        assert params(n)["use_sim_time"] is True and params(n)["scenario_file"] == str(path)
    assert params(fleet)["map_file"] == "", "지도가 없는 시나리오는 기본 지도"


def test_a_scenario_with_its_own_map_gives_that_map_to_the_fleet(made):
    """버전 2 F-2 Gazebo 확인: 시나리오의 `map:`(광장 지도)을 관제, 가짜들, 주행 지표가 함께 쓴다.
    월드와 Nav2 지도는 그대로 lane_arena다(벽과 상자만 있고 지도는 그 안에 든다)."""
    from ament_index_python.packages import get_package_share_directory
    from fleet_gz_sim.fleet_setup import load
    path = (Path(get_package_share_directory("fake_robots")) / "scenarios"
            / "free_area_crossing.yaml")
    actions = made.fleet(str(path), ROBOTS, result_dir="out", monitor=True)
    [fleet] = nodes(actions, "fleet_manager_node")
    assert params(fleet)["map_file"].endswith("feature_map_crossing.yaml")
    for n in nodes(actions):
        if n.kw["package"] == "fake_robots":
            assert params(n)["map_file"] == params(fleet)["map_file"]
    [check] = nodes(actions, "fleet_check")
    assert params(check)["lane_map"] == params(fleet)["map_file"]
    site, _ = load(str(path))
    assert "GNE" in site.zone_of, "출발 자세도 시나리오 지도에서 잰다"


def test_the_stand_in_and_the_fleet_view_get_the_scenario_map_too(made):
    actions = run(made, scenario="free_area_crossing", sim_dock_stub="true")
    [stand_in] = nodes(actions, "sim_stand_in")
    assert params(stand_in)["map_file"].endswith("feature_map_crossing.yaml")
    [view] = nodes(actions, "fleet_view")
    assert params(view)["lane_map"] == params(stand_in)["map_file"]
    default = run(made)
    assert params(nodes(default, "sim_stand_in")[0])["map_file"] == ""


def test_scenarios_the_arena_cannot_run_start_nothing(made):
    from fleet_gz_sim.fleet_setup import ScenarioUnfit
    with pytest.raises(ScenarioUnfit):
        run(made, scenario="stuck_robot")


def test_gazebo_transport_stays_on_this_computer(made, monkeypatch):
    monkeypatch.setenv("GZ_IP", "192.168.0.10")
    monkeypatch.delenv("GZ_PARTITION", raising=False)
    env ={"".join(s.perform(None) for s in a.name): "".join(s.perform(None) for s in a.value)
           for a in run(made) if type(a).__name__ == "SetEnvironmentVariable"}
    assert env["GZ_IP"] == "127.0.0.1" and env["GZ_PARTITION"] == "fleet_gz_44"


def test_obstacles_are_off_unless_a_schedule_is_given(made):
    from ament_index_python.packages import get_package_share_directory
    path = Path(get_package_share_directory("fake_robots")) / "scenarios" / "basic_orders.yaml"
    assert not nodes(made.fleet(str(path), ROBOTS, result_dir="out", monitor=True),
                     "sim_obstacles")


def test_the_obstacle_node_places_boxes_on_gazebo_time_away_from_every_pinky(made):
    from ament_index_python.packages import get_package_share_directory
    path = Path(get_package_share_directory("fake_robots")) / "scenarios" / "basic_orders.yaml"
    actions = made.fleet(str(path), ROBOTS, result_dir="out", monitor=True,
                         obstacles_file="/some/roam.yaml")
    [node] = nodes(actions, "sim_obstacles")
    assert params(node) == {"use_sim_time": True, "robots": ROBOTS,
                            "obstacles_file": "/some/roam.yaml", "lane_map": ""}
    crossing = path.parent / "free_area_crossing.yaml"
    actions = made.fleet(str(crossing), ROBOTS, result_dir="out", monitor=True,
                         obstacles_file="/some/roam.yaml")
    [node] = nodes(actions, "sim_obstacles")
    assert params(node)["lane_map"].endswith("feature_map_crossing.yaml"), "장애물도 시나리오 지도"


def test_an_unknown_obstacle_schedule_stops_the_launch_before_anything_starts(made):
    from fleet_gz_sim.obstacles import ScheduleError
    with pytest.raises(ScheduleError, match="no_such_schedule"):
        run(made, obstacles="no_such_schedule")


def test_one_rviz_shows_the_whole_fleet_by_default(made):
    """로봇마다 TF가 /pinkyN/tf로 나뉘어 RViz 하나는 한 대만 그린다. 기본은 3대 표시 노드 + RViz."""
    actions = run(made)
    [view] = nodes(actions, "fleet_view")
    assert params(view)["robots"] == ROBOTS and params(view)["use_sim_time"] is True
    [rviz] = nodes(actions, "rviz2")
    assert rviz.kw.get("namespace") in (None, "")
    assert rviz.kw["arguments"][-1].endswith("fleet_view.rviz")
    assert params(rviz)["use_sim_time"] is True


def test_rviz_can_follow_one_pinky_or_stay_closed(made):
    actions = run(made, rviz="pinky2")
    assert not nodes(actions, "fleet_view")
    [rviz] = nodes(actions, "rviz2")
    assert rviz.kw["namespace"] == "pinky2"
    closed = run(made, rviz="none")
    assert not nodes(closed, "fleet_view") and not nodes(closed, "rviz2")


# -- 시나리오의 map이 경기장을 고른다 ---------------------------------------------------------

def after_ready(actions):
    """wait_ready가 0으로 끝났을 때 띄우는 관제 쪽 노드들."""
    exits = [a.args[0] for a in flat(actions)
             if isinstance(a, Made) and a.kind == "RegisterEventHandler"]
    [handler] = [e for e in exits if callable(e.kw.get("on_exit"))]

    class Exit:
        returncode = 0
    return handler.kw["on_exit"](Exit(), None)


def world_file(actions) -> str:
    [server] = [a for a in flat(actions) if isinstance(a, Made) and a.kind == "ExecuteProcess"
                and a.kw.get("name") == "gz_server"]
    return server.kw["cmd"][-1]


def test_without_a_map_the_lane_arena_is_used_everywhere(made):
    actions = run(made)
    assert world_file(actions).endswith("/worlds/lane_arena.sdf")
    for nav in includes(actions, "nav2.launch.py"):
        assert launch_args(nav)["map"].endswith("/maps/lane_arena.yaml")
        assert launch_args(nav)["map_fine"].endswith("/maps/lane_arena_fine.yaml")
    fleet = after_ready(actions)
    for node in [*nodes(actions), *nodes(fleet)]:
        assert not params(node).get("map_file") and not params(node).get("lane_map"), node.kw


def test_a_warehouse_scenario_runs_on_the_warehouse_world_maps_and_lane_map(made):
    from ament_index_python.packages import get_package_share_directory
    lane_map = str(Path(get_package_share_directory("fake_robots")) / "maps" / "warehouse.yaml")
    actions = run(made, scenario="gz_warehouse", obstacles="warehouse")
    assert world_file(actions).endswith("/worlds/warehouse.sdf")
    navs = includes(actions, "nav2.launch.py")
    assert len(navs) == 3
    for nav in navs:
        assert launch_args(nav)["map"].endswith("/maps/warehouse.yaml")
        assert launch_args(nav)["map_fine"].endswith("/maps/warehouse_fine.yaml")
    spawned = sorted((float(c.kw["arguments"][c.kw["arguments"].index("-x") + 1]),
                      float(c.kw["arguments"][c.kw["arguments"].index("-y") + 1]))
                     for c in nodes(actions, "create"))
    assert spawned == pytest.approx([(2.65, 3.15), (3.0, 3.15), (3.35, 3.15)])  # P-3..P-1
    [view] = nodes(actions, "fleet_view")
    assert params(view)["lane_map"] == lane_map
    [stand_in] = nodes(actions, "sim_stand_in")
    assert params(stand_in)["map_file"] == lane_map
    fleet = after_ready(actions)
    [manager] = nodes(fleet, "fleet_manager_node")
    assert params(manager)["map_file"] == lane_map
    for node in nodes(fleet):
        if node.kw["package"] == "fake_robots":
            assert params(node)["map_file"] == lane_map, node.kw["executable"]
    for executable in ("sim_obstacles", "fleet_check"):
        [node] = nodes(fleet, executable)
        assert params(node)["lane_map"] == lane_map, executable


def test_an_obstacle_schedule_is_checked_against_the_scenario_map(made):
    from fleet_gz_sim.obstacles import ScheduleError
    with pytest.raises(ScheduleError):
        run(made, scenario="gz_warehouse", obstacles="roam")  # roam의 레인은 창고 지도에 없다
    with pytest.raises(ScheduleError):
        run(made, obstacles="warehouse")


def test_the_recovery_probe_shares_the_fleet_check_process(made):
    """A process of its own would cost a DDS participant (test_ros_process_budget_...)."""
    from ament_index_python.packages import get_package_share_directory
    path = Path(get_package_share_directory("fake_robots")) / "scenarios" / "basic_orders.yaml"
    fleet = made.fleet(str(path), ROBOTS, result_dir="out", monitor=True)
    assert nodes(fleet, "recovery_probe") == []
    [check] = nodes(fleet, "fleet_check")
    assert params(check).items() >= {"use_sim_time": True, "robots": ROBOTS,
                                     "recovery_probe": True, "result_dir": "out",
                                     "settle_s": 2.0}.items()


def test_without_fleet_check_the_recovery_probe_runs_on_its_own(made):
    from ament_index_python.packages import get_package_share_directory
    path = Path(get_package_share_directory("fake_robots")) / "scenarios" / "basic_orders.yaml"
    fleet = made.fleet(str(path), ROBOTS, result_dir="out", monitor=True, check=False)
    [probe] = nodes(fleet, "recovery_probe")
    assert params(probe) == {"use_sim_time": True, "robots": ROBOTS, "result_dir": "out",
                             "settle_s": 2.0}


# -- 잠금 경계 이동의 둘째 RPP, 속도 상한, 충돌 검사, 관제 파라미터 이름 ----------------------

THROUGH = "FollowPathThrough"


@pytest.fixture
def nav_tmp(monkeypatch, tmp_path):
    """The sim Nav2 params copies go to tmp_path, not the system temp folder."""
    from fleet_gz_sim import launch_parts
    monkeypatch.setattr(launch_parts.tempfile, "gettempdir", lambda: str(tmp_path))
    return tmp_path


def nav2_source() -> dict:
    import yaml
    from ament_index_python.packages import get_package_share_directory
    path = (Path(get_package_share_directory("pinky_nav_config")) / "params"
            / "nav2_params.yaml")
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def nav2_files(actions) -> dict:
    """{robot namespace: the Nav2 params that robot's Nav2 gets}, read now (the files are
    per robot and process, so a later launch in this test writes over them)."""
    import yaml
    out = {}
    for nav in includes(actions, "nav2.launch.py"):
        args = launch_args(nav)
        out[args["namespace"]] = yaml.safe_load(
            Path(args["params_file"]).read_text(encoding="utf-8"))
    return out


def fsm_args(actions) -> dict:
    """{robot namespace: pinky_fsm.launch.py arguments}."""
    return {launch_args(i)["namespace"]: launch_args(i)
            for i in includes(actions, "pinky_fsm.launch.py")}


def test_lock_boundary_moves_run_on_the_second_rpp_by_default(made, nav_tmp):
    """The robot Nav2 params already load FollowPathThrough; the sim copy changes only AMCL and
    pinky_fsm sends paths that end at a lock boundary (final_goal false) there."""
    actions = run(made)
    assert {ns: a["through_controller_id"] for ns, a in fsm_args(actions).items()} == {
        ns: THROUGH for ns in ROBOTS}
    source = nav2_source()
    assert source["controller_server"]["ros__parameters"]["controller_plugins"] == [
        "FollowPath", THROUGH]
    used = nav2_files(actions)
    assert set(used) == set(ROBOTS)
    for robot, data in used.items():
        assert data["controller_server"] == source["controller_server"], robot
        assert {k: v for k, v in data.items() if k != "amcl"} == \
            {k: v for k, v in source.items() if k != "amcl"}, robot


def test_without_pass_through_both_rpps_stay_loaded_but_every_move_uses_followpath(
        made, nav_tmp):
    """nav_pass_through:=false compares against the speed limit alone on the same build."""
    on = nav2_files(run(made))
    actions = run(made, nav_pass_through="false")
    assert nav2_files(actions) == on
    assert {a["through_controller_id"] for a in fsm_args(actions).values()} == {""}


def test_nav_max_speed_caps_both_rpps_and_the_fsm_speed_limit_of_every_robot(made, nav_tmp):
    """RPP takes the FSM's lock-boundary limit as its speed, so the cap goes to both."""
    capped_actions = run(made, nav_max_speed="0.12")
    capped = nav2_files(capped_actions)
    plain_actions = run(made)
    plain = nav2_files(plain_actions)
    assert set(capped) == set(plain) == set(ROBOTS)
    for ns in ROBOTS:
        for name in ("FollowPath", THROUGH):
            assert capped[ns]["controller_server"]["ros__parameters"][name][
                "desired_linear_vel"] == 0.12, (ns, name)
            assert plain[ns]["controller_server"]["ros__parameters"][name][
                "desired_linear_vel"] == 0.25, (ns, name)
    assert {a["horizon_cap_mps"] for a in fsm_args(capped_actions).values()} == {"0.12"}
    assert all("horizon_cap_mps" not in a for a in fsm_args(plain_actions).values())


@pytest.mark.parametrize("bad", ["fast", "0", "-1", "nan", "inf", "0.05", "0.079", "0.3"])
def test_a_bad_nav_max_speed_stops_the_launch(made, nav_tmp, bad):
    """Below the FSM's own speeds (backup 0.08, slow-down floor 0.05 m/s) the cap would not hold;
    above the robot's Nav2 speed it would speed the robot up."""
    with pytest.raises(ValueError, match="max_speed"):
        run(made, nav_max_speed=bad)


def test_the_slowest_nav_max_speed_is_the_fsm_backup_speed(made, nav_tmp):
    from pinky_fsm.core import FsmParams
    slowest = FsmParams().backup_speed_mps
    assert slowest >= FsmParams().horizon_min_mps
    actions = run(made, nav_max_speed=str(slowest))
    assert {a["horizon_cap_mps"] for a in fsm_args(actions).values()} == {str(slowest)}


def test_the_fsm_slow_down_floor_also_bounds_nav_max_speed(made, nav_tmp, monkeypatch):
    """Either FSM speed may be the larger one."""
    import types

    import pinky_fsm.core
    monkeypatch.setattr(pinky_fsm.core, "FsmParams", lambda: types.SimpleNamespace(
        backup_speed_mps=0.03, horizon_min_mps=0.06))
    with pytest.raises(ValueError, match="nav_max_speed"):
        run(made, nav_max_speed="0.059")
    actions = run(made, nav_max_speed="0.06")
    assert {a["horizon_cap_mps"] for a in fsm_args(actions).values()} == {"0.06"}


def test_nav_collision_check_false_turns_the_rpp_check_off_for_every_robot(made, nav_tmp):
    off = nav2_files(run(made, nav_collision_check="false"))
    plain = nav2_files(run(made))
    assert set(off) == set(plain) == set(ROBOTS)
    for ns in ROBOTS:
        for name in ("FollowPath", THROUGH):
            assert off[ns]["controller_server"]["ros__parameters"][name][
                "use_collision_detection"] is False, (ns, name)
            assert plain[ns]["controller_server"]["ros__parameters"][name][
                "use_collision_detection"] is True, (ns, name)


@pytest.mark.parametrize("name", ["nav_pass_through", "nav_collision_check"])
def test_a_misspelt_nav_switch_stops_the_launch(made, nav_tmp, name):
    with pytest.raises(ValueError, match=name):
        run(made, **{name: "ture"})


def fleet_params_files(actions) -> set:
    return {params(n)["params_file"] for n in nodes(after_ready(actions))
            if "params_file" in params(n)}


def test_a_bare_fleet_params_name_is_a_fleet_core_data_file(made, nav_tmp):
    from fleet_core.common.params import DEFAULT_PARAMS
    assert fleet_params_files(run(made, fleet_params_file="fleet_params")) == {
        str(DEFAULT_PARAMS)}
    mine = nav_tmp / "mine.yaml"
    mine.write_bytes(DEFAULT_PARAMS.read_bytes())
    assert fleet_params_files(run(made, fleet_params_file=str(mine))) == {str(mine)}, \
        "a path stays as given"
    assert fleet_params_files(run(made)) == {""}, "empty: the installed fleet_params.yaml"


def test_the_real_arena_command_line_resolves_every_argument(made, nav_tmp):
    """The command in scenarios/gz_real_arena.yaml: its fleet params by data file name, a
    slower speed for both RPPs and the FSM cap, and no RPP collision check."""
    from fleet_core.common.params import DEFAULT_PARAMS
    actions = run(made, scenario="gz_real_arena", sim_dock_stub="true",
                  fleet_params_file="fleet_params_real_arena", nav_max_speed="0.12",
                  nav_collision_check="false")
    assert fleet_params_files(actions) == {
        str(DEFAULT_PARAMS.parent / "fleet_params_real_arena.yaml")}
    used = nav2_files(actions)
    assert set(used) == {"pinky1", "pinky2"}
    for ns, data in used.items():
        for name in ("FollowPath", THROUGH):
            rpp = data["controller_server"]["ros__parameters"][name]
            assert rpp["desired_linear_vel"] == 0.12, (ns, name)
            assert rpp["use_collision_detection"] is False, (ns, name)
    assert {(a["horizon_cap_mps"], a["through_controller_id"])
            for a in fsm_args(actions).values()} == {("0.12", THROUGH)}


def test_an_unknown_fleet_params_name_stops_the_launch(made, nav_tmp):
    with pytest.raises(ValueError, match="fleet_params_file"):
        run(made, fleet_params_file="no_such_params")
