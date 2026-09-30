#!/usr/bin/env python3
"""손목 목표 Pose를 MoveIt /compute_ik 로 보내 관절각을 얻고 로봇을 움직이는 브릿지.

구독 (팀 약속):
    /teleop/left_wrist_pose   → MoveIt 그룹 left_arm
    /teleop/right_wrist_pose  → MoveIt 그룹 right_arm
    /joint_states             → IK 시드(현재 자세) 용
발행:
    /teleop/joint_targets                    (sensor_msgs/JointState, rad)
    /left_arm_controller/joint_trajectory    (send_to_controllers=true 일 때)
    /right_arm_controller/joint_trajectory   (   〃   ) → RViz/FakeSystem 팔이 실제로 움직임

파라미터:
    position_only (기본 true): 목표의 회전을 무시하고 중립 방향(w=1)으로 고정.
        컨트롤러의 실제 회전을 쓰려면 false (회전 변환 검증 후 권장).
    send_to_controllers (기본 true): IK 해를 JointTrajectory로 컨트롤러에 전송.
    move_duration_sec (기본 0.3): 각 목표 지점까지 도달 시간 (작을수록 민첩, 클수록 부드러움).

전제: URDF + MoveIt(move_group) + ros2_control 컨트롤러 실행 중 (demo.launch.py면 전부 포함).
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.srv import GetPositionIK
from builtin_interfaces.msg import Duration


class ArmIkClient:
    """한쪽 팔의 IK 요청 상태를 관리한다."""

    def __init__(self, group_name: str, ik_link: str, joint_prefix: str,
                 traj_pub):
        self.group_name = group_name
        self.ik_link = ik_link
        self.joint_prefix = joint_prefix
        self.traj_pub = traj_pub
        self.latest_target = None   # 가장 최근에 받은 PoseStamped
        self.in_flight = False      # IK 요청이 진행 중인지
        self.ok_count = 0
        self.fail_count = 0


class MoveItIkBridge(Node):

    def __init__(self):
        super().__init__('moveit_ik_bridge')

        self.declare_parameter('left_group', 'left_arm')
        self.declare_parameter('right_group', 'right_arm')
        # IK 목표 링크 = SRDF의 tip_link (SH_Humanoid_v2 확인 완료)
        self.declare_parameter('left_ik_link', 'left_hand_link')
        self.declare_parameter('right_ik_link', 'right_hand_link')
        self.declare_parameter('ik_timeout_sec', 0.05)
        self.declare_parameter('position_only', True)
        self.declare_parameter('send_to_controllers', True)
        self.declare_parameter('move_duration_sec', 0.3)
        self.declare_parameter('left_controller_topic',
                               '/left_arm_controller/joint_trajectory')
        self.declare_parameter('right_controller_topic',
                               '/right_arm_controller/joint_trajectory')
        # 중립 손 위치(base_link) — 스케일/클램프의 기준점 (URDF 홈 포즈 FK 값)
        self.declare_parameter('left_neutral', [-0.1658, 0.0027, 0.5824])
        self.declare_parameter('right_neutral', [0.1698, 0.0011, 0.5844])
        # 사람 이동량 → 로봇 이동량 배율 (1.0 = 1:1)
        self.declare_parameter('delta_scale', 1.0)
        # 중립점에서 각 축으로 허용하는 최대 이동량(m) — 작업범위 밖 목표 차단
        self.declare_parameter('max_delta', 0.20)

        self.ik_timeout = float(self.get_parameter('ik_timeout_sec').value)
        self.position_only = bool(self.get_parameter('position_only').value)
        self.send_to_controllers = bool(self.get_parameter('send_to_controllers').value)
        self.move_duration = float(self.get_parameter('move_duration_sec').value)
        self.delta_scale = float(self.get_parameter('delta_scale').value)
        self.max_delta = float(self.get_parameter('max_delta').value)
        self.neutral = {
            'L': list(self.get_parameter('left_neutral').value),
            'R': list(self.get_parameter('right_neutral').value),
        }

        cb = ReentrantCallbackGroup()
        self.client = self.create_client(GetPositionIK, '/compute_ik', callback_group=cb)

        left_traj_pub = self.create_publisher(
            JointTrajectory, str(self.get_parameter('left_controller_topic').value), 10)
        right_traj_pub = self.create_publisher(
            JointTrajectory, str(self.get_parameter('right_controller_topic').value), 10)

        self.arms = {
            'L': ArmIkClient(str(self.get_parameter('left_group').value),
                             str(self.get_parameter('left_ik_link').value),
                             'left_', left_traj_pub),
            'R': ArmIkClient(str(self.get_parameter('right_group').value),
                             str(self.get_parameter('right_ik_link').value),
                             'right_', right_traj_pub),
        }

        self.pub_joints = self.create_publisher(JointState, '/teleop/joint_targets', 10)

        # IK 시드용 현재 관절 상태
        self.current_joint_state = None
        self.create_subscription(JointState, '/joint_states',
                                 self._on_joint_states, 10, callback_group=cb)

        self.create_subscription(
            PoseStamped, '/teleop/left_wrist_pose',
            lambda m: self._on_target('L', m), 10, callback_group=cb)
        self.create_subscription(
            PoseStamped, '/teleop/right_wrist_pose',
            lambda m: self._on_target('R', m), 10, callback_group=cb)

        self.create_timer(0.02, self._pump, callback_group=cb)   # 50 Hz로 대기 목표 처리
        self.create_timer(2.0, self._report)

        mode = 'position-only(방향 고정)' if self.position_only else '6D(위치+방향)'
        dest = '컨트롤러 전송 ON' if self.send_to_controllers else '토픽 발행만'
        self.get_logger().info(
            f'/compute_ik 서비스 대기 중... (MoveIt 실행 필요) | IK 모드: {mode} | {dest}')
        self._service_ready = False

    def _on_joint_states(self, msg: JointState):
        self.current_joint_state = msg

    def _on_target(self, side: str, msg: PoseStamped):
        # 최신 목표만 보관 (오래된 목표에 IK를 낭비하지 않음)
        self.arms[side].latest_target = msg

    def _pump(self):
        if not self._service_ready:
            if self.client.service_is_ready():
                self._service_ready = True
                self.get_logger().info('/compute_ik 연결됨 — IK 시작')
            else:
                return
        for side, arm in self.arms.items():
            if arm.in_flight or arm.latest_target is None:
                continue
            target = arm.latest_target
            arm.latest_target = None
            self._request_ik(side, arm, target)

    def _request_ik(self, side: str, arm: ArmIkClient, target: PoseStamped):
        # 중립점 기준 이동량에 스케일을 곱하고, 작업범위(max_delta) 안으로 제한
        n = self.neutral[side]
        p = target.pose.position
        for axis, (cur, base) in enumerate(((p.x, n[0]), (p.y, n[1]), (p.z, n[2]))):
            d = (cur - base) * self.delta_scale
            d = max(-self.max_delta, min(self.max_delta, d))
            v = base + d
            if axis == 0:
                p.x = v
            elif axis == 1:
                p.y = v
            else:
                p.z = v

        if self.position_only:
            # 회전은 무시하고 중립 방향으로 고정 (홈 포즈의 손 방향 = identity)
            target.pose.orientation.x = 0.0
            target.pose.orientation.y = 0.0
            target.pose.orientation.z = 0.0
            target.pose.orientation.w = 1.0

        req = GetPositionIK.Request()
        req.ik_request.group_name = arm.group_name
        req.ik_request.ik_link_name = arm.ik_link
        req.ik_request.pose_stamped = target
        req.ik_request.avoid_collisions = True
        # 현재 자세를 시드로 → 연속적인 해 (팔 튐 방지)
        if self.current_joint_state is not None:
            req.ik_request.robot_state.joint_state = self.current_joint_state
        req.ik_request.timeout.sec = 0
        req.ik_request.timeout.nanosec = int(self.ik_timeout * 1e9)

        arm.in_flight = True
        future = self.client.call_async(req)
        future.add_done_callback(lambda f, s=side, a=arm: self._on_ik_done(s, a, f))

    def _on_ik_done(self, side: str, arm: ArmIkClient, future):
        arm.in_flight = False
        try:
            res = future.result()
        except Exception as exc:  # noqa: BLE001
            arm.fail_count += 1
            self.get_logger().warn(f'[{side}] IK 서비스 오류: {exc}')
            return

        if res.error_code.val != 1:  # 1 == SUCCESS
            arm.fail_count += 1
            return

        arm.ok_count += 1
        js = res.solution.joint_state

        # 해당 팔의 관절만 추출
        names, positions = [], []
        for name, pos in zip(js.name, js.position):
            if name.startswith(arm.joint_prefix):
                names.append(name)
                positions.append(pos)
        if not names:
            return

        # 1) 관절 목표 토픽 발행 (팀 약속)
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = names
        out.position = positions
        self.pub_joints.publish(out)

        # 2) 로봇 컨트롤러로 전송 → RViz/실기 팔이 실제로 움직임
        if self.send_to_controllers:
            traj = JointTrajectory()
            traj.joint_names = names
            pt = JointTrajectoryPoint()
            pt.positions = positions
            pt.time_from_start = Duration(
                sec=int(self.move_duration),
                nanosec=int((self.move_duration % 1.0) * 1e9))
            traj.points.append(pt)
            arm.traj_pub.publish(traj)

    def _report(self):
        if not self._service_ready:
            return
        parts = []
        for side, arm in self.arms.items():
            parts.append(f'{side}: 성공 {arm.ok_count} / 실패 {arm.fail_count}')
            arm.ok_count = 0
            arm.fail_count = 0
        self.get_logger().info('IK ' + ' | '.join(parts))


def main(args=None):
    rclpy.init(args=args)
    node = MoveItIkBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
