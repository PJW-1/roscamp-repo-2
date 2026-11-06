"""핑키 Pi 한 대를 띄우는 자리(04 구성안 4절). 아직 아무것도 띄우지 않는다.

ros2 launch iponic_bringup pinky.launch.py namespace:=$ROBOT_NS

채울 것: 제조사 pinky_bringup, Nav2(pinky_nav_config), pinky_fsm, pinky_dock을 모두
namespace 아래에 띄운다. TF 프레임은 '{namespace}/' 접두어(Gazebo 시뮬 방식, 실물 미시험).
이 패키지는 다른 패키지를 의존성으로 적지 않는다(Pi에 관제 패키지가 빌드되지 않게).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import EnvironmentVariable, LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "namespace", default_value=EnvironmentVariable("ROBOT_NS", default_value=""),
            description="로봇 이름(pinky1~pinky3). 기본값은 ~/.iponic_env의 ROBOT_NS"),
        LogInfo(msg=["pinky.launch.py는 아직 비어 있다. namespace=",
                     LaunchConfiguration("namespace")]),
    ])
