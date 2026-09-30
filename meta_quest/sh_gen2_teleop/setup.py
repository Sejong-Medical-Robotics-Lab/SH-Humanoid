from glob import glob
from setuptools import setup

package_name = 'sh_gen2_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sejong MR Lab',
    maintainer_email='sejongmrlab2@gmail.com',
    description='Meta Quest 3 → Unity → UDP → ROS2 → MoveIt 텔레오퍼레이션 브릿지',
    license='MIT',
    entry_points={
        'console_scripts': [
            'udp_pose_bridge = sh_gen2_teleop.udp_pose_bridge:main',
            'demo_pose_publisher = sh_gen2_teleop.demo_pose_publisher:main',
            'moveit_ik_bridge = sh_gen2_teleop.moveit_ik_bridge:main',
            'servo_pose_bridge = sh_gen2_teleop.servo_pose_bridge:main',
        ],
    },
)
