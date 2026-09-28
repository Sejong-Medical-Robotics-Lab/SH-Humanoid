#!/usr/bin/env python3
"""Quest/Unity 없이 연결 흐름을 테스트하기 위한 가짜 손목 목표 발행기.

로봇 중립 손 위치 주변에서 사인파로 천천히 움직이는 PoseStamped를
/teleop/left_wrist_pose, /teleop/right_wrist_pose 로 발행한다.
udp_pose_bridge 대신 이 노드를 켜면 뒷단(IK, RViz)만 따로 검증할 수 있다.

실행:
    python3 demo_pose_publisher.py
    # 또는 colcon 빌드 후: ros2 run sh_gen2_teleop demo_pose_publisher
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class DemoPosePublisher(Node):

    def __init__(self):
        super().__init__('demo_pose_publisher')

        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('rate_hz', 20.0)
        # 중립 손 위치 (base_link, m) — SH_Humanoid_v2 홈 포즈 FK 계산값
        self.declare_parameter('left_neutral', [-0.1658, 0.0027, 0.5824])
        self.declare_parameter('right_neutral', [0.1698, 0.0011, 0.5844])
        # 사인파 진폭(m)과 주기(s)
        self.declare_parameter('amplitude', 0.05)
        self.declare_parameter('period_sec', 6.0)

        self.frame_id = str(self.get_parameter('frame_id').value)
        rate = float(self.get_parameter('rate_hz').value)
        self.left_neutral = list(self.get_parameter('left_neutral').value)
        self.right_neutral = list(self.get_parameter('right_neutral').value)
        self.amp = float(self.get_parameter('amplitude').value)
        self.period = float(self.get_parameter('period_sec').value)

        self.pub_left = self.create_publisher(PoseStamped, '/teleop/left_wrist_pose', 10)
        self.pub_right = self.create_publisher(PoseStamped, '/teleop/right_wrist_pose', 10)

        self._t0 = self.get_clock().now()
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f'가짜 손목 목표 발행 시작 ({rate:.0f} Hz, 진폭 {self.amp} m, 주기 {self.period} s)')

    def _tick(self):
        t = (self.get_clock().now() - self._t0).nanoseconds * 1e-9
        phase = 2.0 * math.pi * t / self.period
        # x(좌우)로 사인, z(상하)로 코사인 — 작은 원을 그리는 목표
        dx = self.amp * math.sin(phase)
        dz = self.amp * 0.5 * (1.0 - math.cos(phase))

        now = self.get_clock().now().to_msg()
        for pub, neutral, mirror in (
                (self.pub_left, self.left_neutral, -1.0),
                (self.pub_right, self.right_neutral, 1.0)):
            msg = PoseStamped()
            msg.header.stamp = now
            msg.header.frame_id = self.frame_id
            msg.pose.position.x = neutral[0] + mirror * dx
            msg.pose.position.y = neutral[1]
            msg.pose.position.z = neutral[2] + dz
            msg.pose.orientation.w = 1.0  # 회전 없음 (position-only 단계)
            pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DemoPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
