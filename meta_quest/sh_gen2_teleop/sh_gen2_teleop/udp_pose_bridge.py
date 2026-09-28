#!/usr/bin/env python3
"""Unity에서 오는 SHPOSE UDP 패킷을 ROS2 PoseStamped 토픽으로 발행한다.

패킷 형식 (ASCII, Unity ControllerPoseLogger.cs와 동일):
    SHPOSE,1,{L|R},{timestamp},{x},{y},{z},{qx},{qy},{qz},{qw}

좌표는 이미 Unity 쪽에서 로봇 base_link 기준으로 변환된 값이다 (단위: m).
발행 토픽 (팀 약속):
    /teleop/left_wrist_pose   (geometry_msgs/PoseStamped)
    /teleop/right_wrist_pose  (geometry_msgs/PoseStamped)

실행:
    python3 udp_pose_bridge.py
    # 또는 colcon 빌드 후: ros2 run sh_gen2_teleop udp_pose_bridge
"""

import socket
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class UdpPoseBridge(Node):

    def __init__(self):
        super().__init__('udp_pose_bridge')

        self.declare_parameter('udp_port', 15000)
        self.declare_parameter('frame_id', 'base_link')
        # 이 시간(초) 동안 패킷이 없으면 경고 로그 (수신 끊김 감지)
        self.declare_parameter('stale_warn_sec', 1.0)

        self.udp_port = int(self.get_parameter('udp_port').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.stale_warn_sec = float(self.get_parameter('stale_warn_sec').value)

        self.pub = {
            'L': self.create_publisher(PoseStamped, '/teleop/left_wrist_pose', 10),
            'R': self.create_publisher(PoseStamped, '/teleop/right_wrist_pose', 10),
        }

        self._count = {'L': 0, 'R': 0}
        self._bad = 0
        self._last_rx_time = None
        self._lock = threading.Lock()

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', self.udp_port))
        self.sock.settimeout(0.5)

        self._running = True
        self._thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._thread.start()

        # 2초마다 수신 상태 출력
        self.create_timer(2.0, self._report)

        self.get_logger().info(
            f'UDP {self.udp_port} 포트 수신 대기 중 → '
            f'/teleop/left_wrist_pose, /teleop/right_wrist_pose (frame_id={self.frame_id})')

    def _recv_loop(self):
        while self._running:
            try:
                data, _addr = self.sock.recvfrom(1024)
            except socket.timeout:
                continue
            except OSError:
                break
            self._handle_packet(data)

    def _handle_packet(self, data: bytes):
        try:
            parts = data.decode('ascii').strip().split(',')
            # SHPOSE,1,side,t,x,y,z,qx,qy,qz,qw  → 11개 필드
            if len(parts) != 11 or parts[0] != 'SHPOSE' or parts[1] != '1':
                raise ValueError('bad header')
            side = parts[2]
            if side not in ('L', 'R'):
                raise ValueError('bad side')
            x, y, z, qx, qy, qz, qw = (float(v) for v in parts[4:11])
        except (UnicodeDecodeError, ValueError):
            with self._lock:
                self._bad += 1
            return

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.pub[side].publish(msg)

        with self._lock:
            self._count[side] += 1
            self._last_rx_time = self.get_clock().now()

    def _report(self):
        with self._lock:
            left, right, bad = self._count['L'], self._count['R'], self._bad
            self._count = {'L': 0, 'R': 0}
            self._bad = 0
            last = self._last_rx_time

        if left == 0 and right == 0:
            if last is None:
                self.get_logger().warn('아직 수신된 패킷 없음 (Unity IP/보정(C키)/네트워크 확인)')
            else:
                self.get_logger().warn('수신 끊김 — 마지막 수신 이후 패킷 없음')
        else:
            self.get_logger().info(
                f'수신: L {left / 2.0:.1f} Hz, R {right / 2.0:.1f} Hz'
                + (f', 형식 오류 {bad}건' if bad else ''))

    def destroy_node(self):
        self._running = False
        try:
            self.sock.close()
        except OSError:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UdpPoseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
