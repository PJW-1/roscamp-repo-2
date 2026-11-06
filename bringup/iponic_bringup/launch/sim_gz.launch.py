"""Gazebo 3대 시뮬을 띄운다. 격리 설정이 아니면 시작하지 않는다(fleet_gz_sim).

export ROS_DOMAIN_ID=43 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 launch iponic_bringup sim_gz.launch.py

지금은 fleet_gz_sim이 자리만 있어 격리 검사만 한다.
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gz = PathJoinSubstitution([FindPackageShare("fleet_gz_sim"), "launch", "gz.launch.py"])
    return LaunchDescription([IncludeLaunchDescription(PythonLaunchDescriptionSource(gz))])
