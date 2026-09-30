import os, yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

PKG = 'sh_gen2_teleop'

def load(side):
    p = os.path.join(get_package_share_directory(PKG), 'config', f'servo_{side}.yaml')
    with open(p) as f:
        return {'moveit_servo': yaml.safe_load(f)}

def generate_launch_description():
    mc = MoveItConfigsBuilder('SH_Humanoid_v2',
            package_name='SH_Humanoid_v2_moveit_config').to_moveit_configs()
    params = os.path.join(get_package_share_directory(PKG), 'config', 'quest_ik.yaml')
    nodes = []
    for side in ('left', 'right'):
        nodes.append(Node(
            package='moveit_servo', executable='servo_node',
            namespace=side, name='servo_node', output='screen',
            parameters=[load(side), mc.robot_description, mc.robot_description_semantic,
                        mc.robot_description_kinematics, mc.joint_limits]))
    nodes.append(Node(package=PKG, executable='udp_pose_bridge',
                      name='udp_pose_bridge', output='screen', parameters=[params]))
    # 이름을 moveit_ik_bridge로 두어 quest_ik.yaml의 delta_scale/neutral 등을 그대로 사용
    nodes.append(Node(package=PKG, executable='servo_pose_bridge',
                      name='moveit_ik_bridge', output='screen', parameters=[params]))
    return LaunchDescription(nodes)
