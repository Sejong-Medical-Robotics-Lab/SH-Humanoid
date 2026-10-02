#!/usr/bin/env python3
"""유니트리 xr_teleoperate 방식의 '절대 자세' 입력 (Unity AbsolutePoseSender.cs → UDP 15001).

패킷 (ASCII, 필드 26개, 모든 값은 이미 로봇 base_link 축 기준으로 변환됨):
  SHABS,1,t, hx,hy,hz,hqx,hqy,hqz,hqw, gl, lx,ly,lz,lqx,lqy,lqz,lqw, gr, rx,ry,rz,rqx,rqy,rqz,rqw

변환 (televuer tv_wrapper.py의 arm_reference_mode="head_yaw"와 같은 방식):
  1) 머리 yaw 프레임 R_yaw (머리 정면 방향을 수평으로 투영, 피치·롤은 무시)
  2) 손을 머리 기준으로:  p_rel = R_yawᵀ (p_hand - p_head),  R_rel = R_yawᵀ R_hand
  3) 사람 어깨 기준 → 로봇 어깨 기준으로 옮기고 팔 길이 비율 s = 로봇 팔 길이 / 사람 팔 길이 적용
       p_target = sh_robot + s (p_rel - sh_human)
  4) 방향: R_target = R_rel · R_align   (컨트롤러 축 ↔ 로봇 손 축 정렬 회전)

캘리브레이션 (팔별, 그립을 처음 누를 때 한 번):
  사람이 로봇 준비 자세와 같은 자세로 서서 그립을 누르면
    sh_human = p_rel0 - (p_hand_ready - sh_robot) / s     ← 사람 어깨 위치 추정
    R_align  = R_rel0ᵀ · R_hand_ready                      ← 유니트리 T_TO_UNITREE_HUMANOID_ARM 역할
  이후 그립을 떼었다 다시 눌러도 기준은 유지 (절대 매핑). 다시 잡으려면 recalibrate().
"""
import socket
import threading
import time
from collections import deque

import numpy as np


def quat_to_R(q):
    x, y, z, w = q
    n = np.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def yaw_frame(R_head):
    """머리 정면(+y)을 수평면에 투영한 yaw 프레임 [오른쪽, 정면, 위]."""
    f = R_head @ np.array([0.0, 1.0, 0.0])
    f[2] = 0.0
    n = np.linalg.norm(f)
    if n < 1e-6:
        return np.eye(3)
    f /= n
    up = np.array([0.0, 0.0, 1.0])
    return np.column_stack([np.cross(f, up), f, up])


def parse_packet(text):
    p = text.strip().split(',')
    if len(p) != 26 or p[0] != 'SHABS' or p[1] != '1':
        return None
    v = [float(x) for x in p[2:]]
    t = v[0]
    head = (np.array(v[1:4]), quat_to_R(v[4:8]))
    hands = {'L': (v[8] > 0.5, np.array(v[9:12]), quat_to_R(v[12:16])),
             'R': (v[16] > 0.5, np.array(v[17:20]), quat_to_R(v[20:24]))}
    return t, head, hands


class AbsoluteInput:
    def __init__(self, port, robot_ref, scale, logger=None):
        """robot_ref[s] = dict(sh=어깨 위치, p0=준비 자세 손 위치, R0=준비 자세 손 방향)"""
        self.robot = robot_ref
        self.scale = float(scale)
        self.log = logger
        self.lock = threading.Lock()
        self.latest = None            # (수신 시각, head, hands)
        self.rx_times = deque(maxlen=400)
        self.bad = 0
        self.cal = {'L': None, 'R': None}
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(('0.0.0.0', int(port)))
        self.sock.settimeout(0.5)
        self.running = True
        self.th = threading.Thread(target=self._loop, daemon=True)
        self.th.start()

    def _loop(self):
        while self.running:
            try:
                data, _ = self.sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                pkt = parse_packet(data.decode('ascii', 'ignore'))
            except ValueError:
                pkt = None
            if pkt is None:
                self.bad += 1
                continue
            now = time.monotonic()
            with self.lock:
                self.latest = (now, pkt[1], pkt[2])
                self.rx_times.append(now)

    def close(self):
        self.running = False
        try:
            self.sock.close()
        except OSError:
            pass

    def recalibrate(self):
        self.cal = {'L': None, 'R': None}

    def rate(self, now):
        with self.lock:
            return sum(1 for t in self.rx_times if now - t < 1.0)

    def targets(self, now, timeout):
        """그립을 누르고 있는 팔만 {side: (pos[3], R[3x3])} 반환."""
        with self.lock:
            item = self.latest
        if item is None or now - item[0] > timeout:
            return {}
        _, (ph, Rh), hands = item
        Ry = yaw_frame(Rh)
        out = {}
        for s, (grip, p, R) in hands.items():
            if not grip:
                continue
            p_rel = Ry.T @ (p - ph)
            R_rel = Ry.T @ R
            ref = self.robot[s]
            if self.cal[s] is None:
                sh_h = p_rel - (ref['p0'] - ref['sh']) / self.scale
                R_align = R_rel.T @ ref['R0']
                self.cal[s] = (sh_h, R_align)
                if self.log:
                    self.log(f'[{s}] 절대 모드 캘리브레이션: 사람 어깨(머리 기준) {np.round(sh_h, 3).tolist()}')
            sh_h, R_align = self.cal[s]
            out[s] = (ref['sh'] + self.scale * (p_rel - sh_h), R_rel @ R_align)
        return out
