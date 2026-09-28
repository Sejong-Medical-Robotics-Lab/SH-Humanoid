"""sh_gen2_teleop 실행 파일.

기본: udp_pose_bridge + moveit_ik_bridge
    ros2 launch sh_gen2_teleop quest_ik_demo.launch.py

Quest 없이 데모 목표로 돌리기 (udp 대신 demo 발행기 사용):
    ros2 launch sh_gen2_teleop quest_ik_demo.launch.py use_demo:=true

IK 없이 수신만 확인하기:
    ros2 launch sh_gen2_teleop quest_ik_demo.launch.py use_ik:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('sh_gen2_teleop'), 'config', 'quest_ik.yaml')

    use_demo = LaunchConfiguration('use_demo')
    use_ik = LaunchConfiguration('use_ik')

    return LaunchDescription([
        DeclareLaunchArgument('use_demo', default_value='false',
                              description='true면 UDP 대신 demo_pose_publisher 사용'),
        DeclareLaunchArgument('use_ik', default_value='true',
                              description='false면 moveit_ik_bridge 미실행 (수신 확인용)'),

        Node(package='sh_gen2_teleop', executable='udp_pose_bridge',
             parameters=[config], output='screen',
             condition=UnlessCondition(use_demo)),

        Node(package='sh_gen2_teleop', executable='demo_pose_publisher',
             parameters=[config], output='screen',
             condition=IfCondition(use_demo)),

        Node(package='sh_gen2_teleop', executable='moveit_ik_bridge',
             parameters=[config], output='screen',
             condition=IfCondition(use_ik)),
    ])
