#!/usr/bin/env python3
"""손목 목표 Pose → MoveIt Servo(pose tracking), Servo 출력 관절각 → /teleop/joint_targets 중계.

orientation_mode:
  hold_current (기본): 목표 방향 = 로봇 손의 현재 방향 → 사실상 위치만 추종, 손목 억지 비틀림 방지
  fixed:               목표 방향 = neutral_quat 고정
  controller:          목표 방향 = 입력(컨트롤러) 방향 그대로 (좌표 변환 검증 후 사용)
최신 목표를 republish_hz(기본 200Hz)로 Servo에 재발행. 입력이 input_timeout 넘게 끊기면 발행 중지.
"""

import time
import math
from collections import deque
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from moveit_msgs.srv import ServoCommandType
import tf2_ros

ARM_JOINTS = ['shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow',
              'wrist_yaw', 'wrist_pitch', 'wrist_roll']


class ServoPoseBridge(Node):

    def __init__(self):
        super().__init__('servo_pose_bridge')

        self.declare_parameter('planning_frame', 'base_link')
        self.declare_parameter('left_servo_ns', '/left/servo_node')
        self.declare_parameter('right_servo_ns', '/right/servo_node')
        self.declare_parameter('left_command_topic', '/left_arm_forward_controller/commands')
        self.declare_parameter('right_command_topic', '/right_arm_forward_controller/commands')
        self.declare_parameter('left_ee_link', 'left_hand_link')
        self.declare_parameter('right_ee_link', 'right_hand_link')
        self.declare_parameter('orientation_mode', 'hold_current')
        self.declare_parameter('position_only', True)   # 호환용 (orientation_mode 우선)
        self.declare_parameter('left_neutral', [-0.1658, 0.0027, 0.5824])
        self.declare_parameter('right_neutral', [0.1698, 0.0011, 0.5844])
        self.declare_parameter('left_neutral_quat', [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter('right_neutral_quat', [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter('delta_scale', 1.0)
        self.declare_parameter('max_delta', 0.20)
        self.declare_parameter('republish_hz', 200.0)
        self.declare_parameter('input_timeout', 0.5)
        self.declare_parameter('arm_min_packets', 3)     # 최근 1초에 이만큼 들어와야 동작 (튀는 패킷 무시)
        self.declare_parameter('left_shoulder_link', '')
        self.declare_parameter('right_shoulder_link', '')
        self.declare_parameter('left_reach', 0.0)       # 0이면 구 제한 끔
        self.declare_parameter('right_reach', 0.0)
        self.declare_parameter('reach_ratio', 0.93)     # 완전히 펴지는 특이점 회피
        self.declare_parameter('keepout_radius', 0.12)   # 기둥 중심(base_link x=0,y=0)에서 이 반경 안 금지
        self.declare_parameter('keepout_top_z', 1.10)    # 이 높이 아래에서만 기둥 금지 구역 적용
        self.declare_parameter('midline_margin', 0.03)   # 좌우 팔이 몸 중심선을 넘지 않게
        self.declare_parameter('max_step', 0.03)         # 목표를 현재 손 위치에서 최대 3cm 이내로 제한

        gp = lambda n: self.get_parameter(n).value
        self.frame = str(gp('planning_frame'))
        self.ori_mode = str(gp('orientation_mode'))
        self.delta_scale = float(gp('delta_scale'))
        self.max_delta = float(gp('max_delta'))
        self.input_timeout = float(gp('input_timeout'))
        self.arm_min_packets = int(gp('arm_min_packets'))
        self.max_step = float(gp('max_step'))
        self.keepout_r = float(gp('keepout_radius'))
        self.keepout_top = float(gp('keepout_top_z'))
        self.midline = float(gp('midline_margin'))
        self.shoulder = {'L': str(gp('left_shoulder_link')), 'R': str(gp('right_shoulder_link'))}
        self.reach = {'L': float(gp('left_reach')) * float(gp('reach_ratio')),
                      'R': float(gp('right_reach')) * float(gp('reach_ratio'))}
        self.in_times = {'L': deque(maxlen=50), 'R': deque(maxlen=50)}
        self.neutral = {'L': list(gp('left_neutral')), 'R': list(gp('right_neutral'))}
        self.neutral_quat = {'L': list(gp('left_neutral_quat')),
                             'R': list(gp('right_neutral_quat'))}
        self.ee = {'L': str(gp('left_ee_link')), 'R': str(gp('right_ee_link'))}
        self.joint_names = {'L': [f'left_{j}_joint' for j in ARM_JOINTS],
                            'R': [f'right_{j}_joint' for j in ARM_JOINTS]}

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        ns = {'L': str(gp('left_servo_ns')), 'R': str(gp('right_servo_ns'))}
        self.servo_pub = {s: self.create_publisher(PoseStamped, f'{ns[s]}/pose_target_cmds', 10)
                          for s in ns}
        self.mode_cli = {s: self.create_client(ServoCommandType, f'{ns[s]}/switch_command_type')
                         for s in ns}
        self.mode_ok = {'L': False, 'R': False}
        self.mode_pending = {'L': False, 'R': False}

        self.pub_joints = self.create_publisher(JointState, '/teleop/joint_targets', 10)

        self.latest = {'L': None, 'R': None}       # (pos[3], quat[4], 수신시각)
        self.create_subscription(PoseStamped, '/teleop/left_wrist_pose',
                                 lambda m: self._on_target('L', m), 10)
        self.create_subscription(PoseStamped, '/teleop/right_wrist_pose',
                                 lambda m: self._on_target('R', m), 10)
        self.create_subscription(Float64MultiArray, str(gp('left_command_topic')),
                                 lambda m: self._on_servo_out('L', m), 10)
        self.create_subscription(Float64MultiArray, str(gp('right_command_topic')),
                                 lambda m: self._on_servo_out('R', m), 10)

        self.cnt_in = {'L': 0, 'R': 0}
        self.cnt_cmd = {'L': 0, 'R': 0}
        self.cnt_out = {'L': 0, 'R': 0}
        self.create_timer(1.0 / float(gp('republish_hz')), self._tick)
        self.create_timer(0.5, self._ensure_pose_mode)
        self.create_timer(2.0, self._report)

        self.get_logger().info(
            f'Servo 브릿지 시작 | 방향모드: {self.ori_mode} | delta_scale={self.delta_scale} | '
            f'max_delta={self.max_delta} | L중립={self.neutral["L"]} R중립={self.neutral["R"]}')

    def _ensure_pose_mode(self):
        for s, cli in self.mode_cli.items():
            if self.mode_ok[s] or self.mode_pending[s] or not cli.service_is_ready():
                continue
            self.mode_pending[s] = True
            fut = cli.call_async(ServoCommandType.Request(command_type=2))
            fut.add_done_callback(lambda f, side=s: self._on_mode(side, f))

    def _on_mode(self, side, fut):
        self.mode_pending[side] = False
        try:
            ok = fut.result().success
        except Exception as exc:  # noqa: BLE001
            ok = False
            self.get_logger().warn(f'[{side}] Servo 모드 전환 오류: {exc}')
        self.mode_ok[side] = ok
        if ok:
            self.get_logger().info(f'[{side}] Servo POSE 모드 전환 완료')

    # 입력 수신: 위치만 계산해서 저장 (발행은 _tick에서 200Hz로)
    def _on_target(self, side, msg: PoseStamped):
        n = self.neutral[side]
        p = msg.pose.position
        pos = []
        for cur, base in ((p.x, n[0]), (p.y, n[1]), (p.z, n[2])):
            d = (cur - base) * self.delta_scale
            d = max(-self.max_delta, min(self.max_delta, d))
            pos.append(base + d)
        o = msg.pose.orientation
        self.latest[side] = (pos, [o.x, o.y, o.z, o.w], time.monotonic())
        self.in_times[side].append(time.monotonic())
        self.cnt_in[side] += 1

    def _target_quat(self, side, ctrl_quat):
        if self.ori_mode == 'controller':
            return ctrl_quat
        if self.ori_mode == 'hold_current':
            try:
                t = self.tf_buffer.lookup_transform(self.frame, self.ee[side], Time())
                r = t.transform.rotation
                return [r.x, r.y, r.z, r.w]
            except Exception:  # noqa: BLE001
                pass
        return self.neutral_quat[side]

    def _limit_reach(self, side, pos):
        if self.reach[side] <= 0.0 or not self.shoulder[side]:
            return pos
        try:
            t = self.tf_buffer.lookup_transform(self.frame, self.shoulder[side], Time())
        except Exception:  # noqa: BLE001
            return pos
        c = t.transform.translation
        d = [pos[0] - c.x, pos[1] - c.y, pos[2] - c.z]
        n = math.sqrt(sum(v * v for v in d))
        if n <= self.reach[side] or n < 1e-9:
            return pos
        k = self.reach[side] / n
        return [c.x + d[0] * k, c.y + d[1] * k, c.z + d[2] * k]

    def _keep_out(self, side, pos):
        x, y, z = pos
        # 1) 팔 교차 금지 (왼팔 x<0, 오른팔 x>0 영역만)
        if side == 'L':
            x = min(x, -self.midline)
        else:
            x = max(x, self.midline)
        # 2) 기둥 주변 원기둥 금지 구역 → 바깥으로 밀어냄
        if z < self.keepout_top:
            r = math.hypot(x, y)
            if r < self.keepout_r:
                if r < 1e-6:
                    x, y = 0.0, self.keepout_r
                else:
                    k = self.keepout_r / r
                    x, y = x * k, y * k
        return [x, y, z]

    def _limit_step(self, side, pos):
        try:
            t = self.tf_buffer.lookup_transform(self.frame, self.ee[side], Time())
        except Exception:  # noqa: BLE001
            return pos
        c = t.transform.translation
        d = [pos[0] - c.x, pos[1] - c.y, pos[2] - c.z]
        n = math.sqrt(sum(v * v for v in d))
        if n <= self.max_step or n < 1e-9:
            return pos
        k = self.max_step / n
        return [c.x + d[0] * k, c.y + d[1] * k, c.z + d[2] * k]

    def _tick(self):
        now = time.monotonic()
        for side, item in self.latest.items():
            if item is None or not self.mode_ok[side]:
                continue
            pos, ctrl_quat, t_in = item
            if now - t_in > self.input_timeout:
                continue   # 입력 끊김 → 발행 중지 (Servo가 스스로 정지)
            recent = sum(1 for t in self.in_times[side] if now - t < 1.0)
            if recent < self.arm_min_packets:
                continue   # 입력이 꾸준하지 않음 → 움직이지 않음
            pos = self._limit_reach(side, pos)
            pos = self._keep_out(side, pos)
            pos = self._limit_step(side, pos)
            q = self._target_quat(side, ctrl_quat)
            out = PoseStamped()
            out.header.frame_id = self.frame
            out.header.stamp = self.get_clock().now().to_msg()
            out.pose.position.x, out.pose.position.y, out.pose.position.z = pos
            (out.pose.orientation.x, out.pose.orientation.y,
             out.pose.orientation.z, out.pose.orientation.w) = q
            self.servo_pub[side].publish(out)
            self.cnt_cmd[side] += 1

    def _on_servo_out(self, side, msg: Float64MultiArray):
        names = self.joint_names[side]
        if len(msg.data) != len(names):
            return
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = names
        js.position = list(msg.data)
        self.pub_joints.publish(js)
        self.cnt_out[side] += 1

    def _report(self):
        parts = []
        for s in ('L', 'R'):
            parts.append(f'{s}: 입력 {self.cnt_in[s] / 2:.0f}Hz → Servo목표 {self.cnt_cmd[s] / 2:.0f}Hz'
                         f' → 관절출력 {self.cnt_out[s] / 2:.0f}Hz'
                         f'{"" if self.mode_ok[s] else " (Servo 대기)"}')
            self.cnt_in[s] = self.cnt_cmd[s] = self.cnt_out[s] = 0
        self.get_logger().info(' | '.join(parts))


def main(args=None):
    rclpy.init(args=args)
    node = ServoPoseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
