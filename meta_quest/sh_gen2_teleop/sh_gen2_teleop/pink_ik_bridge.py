#!/usr/bin/env python3
"""손목 목표 Pose → pink(Pinocchio) 미분 IK → forward 컨트롤러 관절각 (Servo 대체).

ServoPoseBridge 상속: 목표 가공은 그대로, Servo 전송 부분만 pink QP IK로 교체.
  최소화  |손 위치 오차|² + orientation_cost·|방향 오차|² + posture_cost·Σ w_j (q_j - 준비자세_j)²
  제약    URDF 관절 limit(− limit_margin), 관절 속도 ≤ teleop_max_vel
  후처리  캡슐 충돌 필터 (여유가 collision_margin 아래로 줄어드는 스텝은 버림)

유니트리 xr_teleoperate에서 가져온 것:
  위치:방향 50:1 가중치, 출력 가중 이동평균 필터 [0.4,0.3,0.2,0.1], 출력 속도 클립,
  실제 관절각 재동기화, 중력 보상 토크(/teleop/joint_ff), IK 실패 시 자세 유지
"""
import time
from collections import deque

import numpy as np
import pinocchio as pin
import qpsolvers
import rclpy
from pink import Configuration, solve_ik
from pink.exceptions import NoSolutionFound
from pink.tasks import FrameTask, PostureTask
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from sh_gen2_teleop.servo_pose_bridge import ServoPoseBridge
from sh_gen2_teleop.abs_input import AbsoluteInput
from std_srvs.srv import Trigger

SIDES = {'L': 'left', 'R': 'right'}


class WeightedPostureTask(PostureTask):
    """관절별 가중치 posture (오차·야코비안 행에 sqrt(w)를 곱함)."""

    def __init__(self, cost, weights):
        super().__init__(cost=cost)
        self.sqrt_w = np.sqrt(np.asarray(weights, dtype=float))

    def compute_error(self, configuration):
        return self.sqrt_w * super().compute_error(configuration)

    def compute_jacobian(self, configuration):
        return self.sqrt_w[:, None] * super().compute_jacobian(configuration)


class CapsuleCollision:
    """팔(팔꿈치·팔뚝·손)을 구 체인으로 근사해서 몸통 형상과의 최소 여유(m) 계산."""

    def __init__(self, model, ee, p):
        self.model = model
        self.data = model.createData()
        self.p = p
        self.jid = {s: {k: model.getJointId(f'{SIDES[s]}_{k}_joint')
                        for k in ('elbow', 'wrist_pitch')} for s in SIDES}
        self.fid = {s: model.getFrameId(ee[s]) for s in SIDES}

    def _spheres(self, q):
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        out = {}
        n = int(self.p['samples'])
        for s in SIDES:
            el = self.data.oMi[self.jid[s]['elbow']].translation.copy()
            wr = self.data.oMi[self.jid[s]['wrist_pitch']].translation.copy()
            Mh = self.data.oMf[self.fid[s]]
            hd = Mh.translation.copy()
            tip = Mh.act(np.array(self.p['finger_tip'][s], dtype=float))
            pts, rad = [el], [self.p['r_elbow']]
            for a in np.linspace(0.0, 1.0, n)[1:]:
                pts.append(el + a * (wr - el)); rad.append(self.p['r_forearm'])
            for a in np.linspace(0.0, 1.0, n)[1:]:
                pts.append(wr + a * (hd - wr)); rad.append(self.p['r_hand'])
            for a in np.linspace(0.0, 1.0, n + 1)[1:]:
                pts.append(hd + a * (tip - hd)); rad.append(self.p['r_hand'])
            out[s] = (np.array(pts), np.array(rad))
        return out

    def clearance(self, q):
        return min(self.clearance_sides(q).values())

    def clearance_sides(self, q):
        """팔별 최소 여유 {'L': m, 'R': m} (양팔 사이 거리는 두 팔 모두에 반영)"""
        p = self.p
        sph = self._spheres(q)
        res = {}
        for s, (P, R) in sph.items():
            best = np.inf
            x, y, z = P[:, 0], P[:, 1], P[:, 2]
            on = (z > p['col_z'][0]) & (z < p['col_z'][1])
            if on.any():
                d = np.hypot(x - p['col_xy'][0], y - p['col_xy'][1]) - p['col_r'] - R
                best = min(best, d[on].min())
            lo, hi = np.array(p['bar_min']), np.array(p['bar_max'])
            dv = np.maximum(np.maximum(lo - P, P - hi), 0.0)
            best = min(best, (np.linalg.norm(dv, axis=1) - R).min())
            inside = (np.abs(x) < p['plat_xy'][0]) & (np.abs(y) < p['plat_xy'][1])
            if inside.any():
                best = min(best, (z - p['plat_z'] - R)[inside].min())
            res[s] = best
        (PL, RL), (PR, RR) = sph['L'], sph['R']
        dd = float((np.linalg.norm(PL[:, None, :] - PR[None, :, :], axis=2) - RL[:, None] - RR[None, :]).min())
        return {s: float(min(v, dd)) for s, v in res.items()}

class PinkIKBridge(ServoPoseBridge):

    def __init__(self):
        super().__init__()
        dp = self.declare_parameter
        dp('robot_description', '')
        # relative = 기존 (그립 순간 기준 상대 이동, UDP 15000) / absolute = 유니트리 방식 (UDP 15001)
        dp('input_mode', 'relative')
        dp('abs_udp_port', 15001)
        dp('human_arm_length', 0.62)        # 사람 어깨~손 길이(m). 로봇 팔 길이(left_reach)와의 비율로 스케일
        dp('abs_use_orientation', True)     # False면 손 방향은 준비 자세로 고정
        dp('posture_cost', 0.02)
        # 순서: shoulder_pitch, roll, yaw, elbow, wrist_yaw, wrist_pitch, wrist_roll
        dp('posture_weights', [0.1, 1.0, 1.0, 0.3, 0.5, 0.5, 0.5])
        dp('orientation_cost', 0.15)
        dp('orientation_compose', 'local')   # controller 모드: local = 중립×변화량, world = 변화량×중립
        dp('neutral_from_fk', True)          # 중립 손 방향을 준비 자세 FK로 계산 (yaml 값 무시)
        dp('damping', 1e-4)
        dp('limit_margin', 0.05)
        dp('teleop_max_vel', 1.5)
        dp('ready_pose', [0.0, 0.0, 0.0, -1.2, 0.0, 0.3, 0.0])
        dp('qp_solver', '')
        # 충돌 (base_link.stl 실측: 기둥 (0,0) r=0.057, z 0.14~0.98 / 어깨 바 z 0.98~1.10)
        dp('collision_check', True)
        dp('collision_margin', 0.03)
        dp('col_xy', [0.0, 0.0])
        dp('col_r', 0.06)
        dp('col_z', [0.14, 0.99])
        dp('bar_min', [-0.135, -0.068, 0.975])
        dp('bar_max', [0.135, 0.068, 1.105])
        dp('plat_xy', [0.34, 0.44])
        dp('plat_z', 0.15)
        dp('r_elbow', 0.05)
        dp('r_forearm', 0.04)
        dp('r_hand', 0.055)
        dp('collision_samples', 4)
        # 손 메시 끝(hand_link 기준). L_hand/R_hand.stl 실측: 손바닥 중심 z≈-0.10, 손끝 z≈-0.20
        dp('left_finger_tip', [0.006, 0.011, -0.19])
        dp('right_finger_tip', [-0.006, -0.024, -0.18])
        # 유니트리식 출력 처리
        dp('output_filter_weights', [0.4, 0.3, 0.2, 0.1])   # 빈 리스트면 필터 끔
        dp('output_vel_limit', 1.5)        # rad/s, 발행 직전 관절 변화량 클립
        dp('resync_threshold', 0.3)        # rad, 내부 자세와 /joint_states 차이가 이보다 크고
        dp('resync_ticks', 40)             #   이 주기 수만큼 지속되면 실제 각도로 재동기화
        dp('publish_gravity_ff', True)     # /teleop/joint_ff (position=명령각, effort=중력 토크)

        gp = lambda n: self.get_parameter(n).value
        urdf = str(gp('robot_description'))
        if not urdf:
            raise RuntimeError('robot_description 파라미터가 비어 있음 (launch에서 전달 필요)')

        full = pin.buildModelFromXML(urdf)
        keep = self.joint_names['L'] + self.joint_names['R']
        missing = [n for n in keep if not full.existJointName(n)]
        if missing:
            raise RuntimeError(f'URDF에 없는 관절: {missing}')
        lock = [full.getJointId(n) for n in list(full.names)[1:] if n not in keep]
        self.model = pin.buildReducedModel(full, lock, pin.neutral(full))
        self.qidx = {n: self.model.joints[self.model.getJointId(n)].idx_q for n in keep}

        m = float(gp('limit_margin'))
        self.model.lowerPositionLimit = self.model.lowerPositionLimit + m
        self.model.upperPositionLimit = self.model.upperPositionLimit - m
        self.model.velocityLimit = np.minimum(self.model.velocityLimit, float(gp('teleop_max_vel')))
        self.lo = self.model.lowerPositionLimit + 1e-4
        self.hi = self.model.upperPositionLimit - 1e-4

        ready = list(gp('ready_pose'))
        wts = list(gp('posture_weights'))
        self.q_ready = pin.neutral(self.model)
        w = np.ones(self.model.nv)
        for s in SIDES:
            for n, v, wi in zip(self.joint_names[s], ready, wts):
                self.q_ready[self.qidx[n]] = v
                w[self.model.joints[self.model.getJointId(n)].idx_v] = wi

        if bool(gp('neutral_from_fk')):
            d = self.model.createData()
            pin.framesForwardKinematics(self.model, d, self.q_ready)
            for s in SIDES:
                qq = pin.Quaternion(d.oMf[self.model.getFrameId(self.ee[s])].rotation)
                self.neutral_quat[s] = [qq.x, qq.y, qq.z, qq.w]
                self.get_logger().info(f'[{s}] 중립 손 방향(FK): '
                                       f'{[round(v, 4) for v in self.neutral_quat[s]]}')
        self.compose = str(gp('orientation_compose'))

        ori = float(gp('orientation_cost'))
        self.hand = {s: FrameTask(self.ee[s], position_cost=1.0, orientation_cost=ori) for s in SIDES}
        self.posture = WeightedPostureTask(float(gp('posture_cost')), w)
        self.posture.set_target(self.q_ready)
        self.damping = float(gp('damping'))
        pref = str(gp('qp_solver'))
        order = ([pref] if pref else []) + ['proxqp', 'quadprog', 'daqp', 'osqp']
        self.solvers = [x for i, x in enumerate(order)
                        if x in qpsolvers.available_solvers and x not in order[:i]]
        if not self.solvers:
            raise RuntimeError(f'QP 솔버 없음: {qpsolvers.available_solvers}')
        self.dt = 1.0 / float(gp('republish_hz'))

        self.col = None
        if bool(gp('collision_check')):
            cp = {k: gp(k) for k in ('col_xy', 'col_r', 'col_z', 'bar_min', 'bar_max',
                                     'plat_xy', 'plat_z', 'r_elbow', 'r_forearm', 'r_hand')}
            cp['samples'] = int(gp('collision_samples'))
            cp['finger_tip'] = {'L': list(gp('left_finger_tip')), 'R': list(gp('right_finger_tip'))}
            self.col = CapsuleCollision(self.model, self.ee, cp)
        self.col_margin = float(gp('collision_margin'))
        self.clear = np.inf
        self.clear_s = {'L': np.inf, 'R': np.inf}
        if self.col:
            need = float(gp('col_r')) + float(gp('r_hand')) + self.col_margin + 0.01
            if self.keepout_r < need:
                self.get_logger().info(f'keepout_radius {self.keepout_r} -> {need:.3f} (충돌 필터와 맞춤)')
                self.keepout_r = need

        fw = list(gp('output_filter_weights'))
        self.filt_w = np.asarray(fw, dtype=float) if fw else None
        self.q_hist = deque(maxlen=len(fw) if fw else 1)
        self.out_step = float(gp('output_vel_limit')) * self.dt
        self.q_out = None
        self.q_meas = None
        self.resync_thr = float(gp('resync_threshold'))
        self.resync_ticks = int(gp('resync_ticks'))
        self.resync_cnt = 0
        self.resyncs = 0
        self.grav = bool(gp('publish_gravity_ff'))
        self.data_g = self.model.createData()
        self.pub_ff = self.create_publisher(JointState, '/teleop/joint_ff', 10) if self.grav else None
        self.all_joints = self.joint_names['L'] + self.joint_names['R']

        self.abs_in = None
        if str(gp('input_mode')) == 'absolute':
            d = self.model.createData()
            pin.framesForwardKinematics(self.model, d, self.q_ready)
            ref = {}
            for s in SIDES:
                sh = d.oMi[self.model.getJointId(f'{SIDES[s]}_shoulder_pitch_joint')].translation.copy()
                M = d.oMf[self.model.getFrameId(self.ee[s])]
                ref[s] = dict(sh=sh, p0=M.translation.copy(), R0=M.rotation.copy())
            scale = float(gp('left_reach')) / float(gp('human_arm_length'))
            self.abs_in = AbsoluteInput(int(gp('abs_udp_port')), ref, scale, self.get_logger().info)
            self.abs_ori = bool(gp('abs_use_orientation'))
            self.create_service(Trigger, '~/recalibrate', self._on_recal)
            self.get_logger().info(
                f'입력: absolute (UDP {gp("abs_udp_port")}, 팔 길이 비율 {scale:.2f}). '
                f'준비 자세로 서서 그립을 처음 누르면 팔별로 캘리브레이션됩니다.')
        else:
            self.get_logger().info('입력: relative (기존 방식)')

        self.config = None
        self.goal = {}
        self.cmd_pub = {s: self.create_publisher(
            Float64MultiArray, str(gp(f'{SIDES[s]}_command_topic')), 10) for s in SIDES}
        self.create_subscription(JointState, '/joint_states', self._on_joint_states, 10)

        self.mode_ok = {'L': True, 'R': True}
        self.qp_fails = 0
        self.col_blocks = 0
        self.solve_ms = []
        self.get_logger().info(
            f'pink IK 브릿지 | 솔버 {self.solvers[0]} | posture {gp("posture_cost")} {wts} | '
            f'orientation {ori} ({self.ori_mode}/{self.compose}) | vel<={gp("teleop_max_vel")} | '
            f'충돌 {"ON" if self.col else "OFF"} (여유 {self.col_margin} m) | {1 / self.dt:.0f} Hz')

    def _ensure_pose_mode(self):
        pass

    def _on_recal(self, req, res):
        self.abs_in.recalibrate()
        res.success = True
        res.message = '다음에 그립을 누를 때 다시 캘리브레이션'
        self.get_logger().info(res.message)
        return res

    def _abs_update(self, now):
        for s, (pos, R) in self.abs_in.targets(now, self.input_timeout).items():
            pos = self._limit_reach(s, list(pos))
            pos = self._keep_out(s, pos)
            pos = self._limit_step(s, pos)
            if not self.abs_ori:
                R = self._quat_to_R(self.neutral_quat[s])
            self.goal[s] = pin.SE3(np.asarray(R, dtype=float), np.asarray(pos, dtype=float))
            self.hand[s].set_target(self.goal[s])
            self.cnt_cmd[s] += 1

    def _on_joint_states(self, msg: JointState):
        pos = dict(zip(msg.name, msg.position))
        if not all(n in pos for n in self.qidx):
            return
        q = pin.neutral(self.model)
        for n, i in self.qidx.items():
            q[i] = pos[n]
        self.q_meas = q
        if self.config is not None:
            return
        q = np.clip(q, self.lo, self.hi)
        self.q_out = q.copy()
        self.config = Configuration(self.model, self.model.createData(), q)
        for s in SIDES:
            self.goal[s] = self.config.get_transform_frame_to_world(self.ee[s]).copy()
            self.hand[s].set_target(self.goal[s])
        if self.col:
            self.clear_s = self.col.clearance_sides(q)
            self.clear = min(self.clear_s.values())
        self.get_logger().info(f'초기 자세 수신 → IK 시작 (충돌 여유 {self.clear:.3f} m)')

    def _target_quat(self, side, ctrl_quat):
        if self.ori_mode == 'controller' and self.compose == 'world':
            return self._qmul(ctrl_quat, self.neutral_quat[side])
        return super()._target_quat(side, ctrl_quat)

    @staticmethod
    def _quat_to_R(q):
        x, y, z, w = q
        return pin.Quaternion(w, x, y, z).normalized().toRotationMatrix()

    def _tick(self):
        if self.config is None:
            return
        now = time.monotonic()
        if self.abs_in is not None:
            self._abs_update(now)
        for s, item in ([] if self.abs_in is not None else self.latest.items()):
            if item is None:
                continue
            pos, ctrl_quat, t_in = item
            if now - t_in > self.input_timeout:
                continue
            if sum(1 for t in self.in_times[s] if now - t < 1.0) < self.arm_min_packets:
                continue
            pos = self._limit_reach(s, pos)
            pos = self._keep_out(s, pos)
            pos = self._limit_step(s, pos)
            R = self._quat_to_R(self._target_quat(s, ctrl_quat))
            self.goal[s] = pin.SE3(R, np.asarray(pos, dtype=float))
            self.hand[s].set_target(self.goal[s])
            self.cnt_cmd[s] += 1

        if self.q_meas is not None and self.resync_thr > 0:
            if np.max(np.abs(self.config.q - self.q_meas)) > self.resync_thr:
                self.resync_cnt += 1
                if self.resync_cnt >= self.resync_ticks:
                    self.config.update(np.clip(self.q_meas, self.lo, self.hi))
                    self.q_hist.clear()
                    self.resync_cnt = 0
                    self.resyncs += 1
                    if self.col:
                        self.clear_s = self.col.clearance_sides(self.config.q)
                        self.clear = min(self.clear_s.values())
                    self.get_logger().warn('내부 자세와 /joint_states 차이 큼 → 실제 각도로 재동기화')
            else:
                self.resync_cnt = 0

        t0 = time.perf_counter()
        v = None
        for sv in self.solvers:
            try:
                v = solve_ik(self.config, [*self.hand.values(), self.posture], self.dt,
                             solver=sv, damping=self.damping, safety_break=False)
                break
            except NoSolutionFound:
                continue
        if v is None:   # 실패 시 감쇠를 크게 해서 한 번 더
            try:
                v = solve_ik(self.config, [*self.hand.values(), self.posture], self.dt,
                             solver=self.solvers[0], damping=max(self.damping * 100, 1e-2),
                             safety_break=False)
            except NoSolutionFound:
                v = None
        if v is None:
            self.qp_fails += 1
        else:
            q_new = np.clip(pin.integrate(self.model, self.config.q, v * self.dt), self.lo, self.hi)
            if self.col:
                c_new = self.col.clearance_sides(q_new)
                for s in SIDES:
                    if c_new[s] < self.col_margin and c_new[s] < self.clear_s[s] - 1e-6:
                        for n in self.joint_names[s]:
                            q_new[self.qidx[n]] = self.config.q[self.qidx[n]]
                        self.col_blocks += 1
                self.clear_s = self.col.clearance_sides(q_new)
                self.clear = min(self.clear_s.values())
            self.config.update(q_new)
        self.solve_ms.append((time.perf_counter() - t0) * 1000)

        self.q_hist.appendleft(self.config.q.copy())
        if self.filt_w is not None:
            w = self.filt_w[:len(self.q_hist)]
            q_f = np.tensordot(w / w.sum(), np.array(self.q_hist), axes=1)
        else:
            q_f = self.config.q.copy()
        if self.q_out is not None and self.out_step > 0:
            q_f = self.q_out + np.clip(q_f - self.q_out, -self.out_step, self.out_step)
        self.q_out = q_f

        for s in SIDES:
            msg = Float64MultiArray()
            msg.data = [float(q_f[self.qidx[n]]) for n in self.joint_names[s]]
            self.cmd_pub[s].publish(msg)
        if self.grav:
            tau = pin.computeGeneralizedGravity(self.model, self.data_g, q_f)
            ff = JointState()
            ff.header.stamp = self.get_clock().now().to_msg()
            ff.name = self.all_joints
            ff.position = [float(q_f[self.qidx[n]]) for n in self.all_joints]
            ff.effort = [float(tau[self.model.joints[self.model.getJointId(n)].idx_v])
                         for n in self.all_joints]
            self.pub_ff.publish(ff)

    def _report(self):
        super()._report()
        if self.abs_in is not None:
            cal = ''.join(k if self.abs_in.cal[k] is not None else '-' for k in ('L', 'R'))
            self.get_logger().info(f'절대 입력 {self.abs_in.rate(time.monotonic())} Hz | 캘리브레이션 {cal} '
                                   f'| 잘못된 패킷 {self.abs_in.bad}')
        if self.solve_ms:
            t = np.array(self.solve_ms)
            self.get_logger().info(
                f'IK+충돌 평균 {t.mean():.2f} ms / 최대 {t.max():.2f} ms | QP 실패 {self.qp_fails} | '
                f'충돌 차단 {self.col_blocks} | 여유 L {self.clear_s["L"]:.3f} R {self.clear_s["R"]:.3f} m | 재동기화 {self.resyncs}')
        self.solve_ms = []
        self.qp_fails = 0
        self.col_blocks = 0


def main(args=None):
    rclpy.init(args=args)
    node = PinkIKBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
