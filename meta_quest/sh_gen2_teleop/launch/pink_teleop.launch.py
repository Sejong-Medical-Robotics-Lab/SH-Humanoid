import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

PKG = 'sh_gen2_teleop'


def generate_launch_description():
    mc = MoveItConfigsBuilder('SH_Humanoid_v2',
                              package_name='SH_Humanoid_v2_moveit_config').to_moveit_configs()
    params = os.path.join(get_package_share_directory(PKG), 'config', 'quest_ik.yaml')
    return LaunchDescription([
        Node(package=PKG, executable='udp_pose_bridge',
             name='udp_pose_bridge', output='screen', parameters=[params]),
        # servo_teleop과 같은 이유로 노드 이름을 moveit_ik_bridge로 둬서 quest_ik.yaml 블록 재사용
        Node(package=PKG, executable='pink_ik_bridge',
             name='moveit_ik_bridge', output='screen',
             parameters=[params, mc.robot_description]),
    ])
