# sh_gen2_teleop

Meta Quest 3 → Unity → UDP → ROS2 → MoveIt 텔레오퍼레이션 브릿지 (퀘스트 팀 패키지).

팀 약속 준수: 단위 m/rad, 기준 좌표 `base_link`, 토픽 `/teleop/left_wrist_pose`,
`/teleop/right_wrist_pose`, `/teleop/joint_targets`, MoveIt 그룹 `left_arm`/`right_arm`.

## 구성

| 파일 | 역할 |
|---|---|
| `sh_gen2_teleop/udp_pose_bridge.py` | Unity SHPOSE UDP(15000) → PoseStamped 토픽 |
| `sh_gen2_teleop/demo_pose_publisher.py` | Quest 없이 가짜 손목 목표 발행 (사인파) |
| `sh_gen2_teleop/moveit_ik_bridge.py` | Pose → MoveIt `/compute_ik` → `/teleop/joint_targets` |
| `launch/quest_ik_demo.launch.py` | 실행 파일 |
| `config/quest_ik.yaml` | 설정값 |

## 빠른 시작 (빌드 없이, Jetson에서)

```bash
# 1) 수신 브릿지 실행
python3 ~/sh_gen2_teleop/sh_gen2_teleop/udp_pose_bridge.py

# 2) 다른 터미널에서 수신 확인
ros2 topic echo /teleop/right_wrist_pose
ros2 topic hz /teleop/right_wrist_pose
```

Unity 쪽: PoseLogger의 `Ros2 Computer Address`에 Jetson IP 입력 → Play → C키 보정.
(보정 전에는 UDP가 나가지 않음)

## 정식 설치 (colcon 빌드)

```bash
mkdir -p ~/sh_ws/src
cp -r ~/sh_gen2_teleop ~/sh_ws/src/
cd ~/sh_ws
colcon build --symlink-install
source install/setup.bash    # 매 터미널마다, 또는 ~/.bashrc에 추가

ros2 launch sh_gen2_teleop quest_ik_demo.launch.py use_ik:=false   # 수신만
ros2 launch sh_gen2_teleop quest_ik_demo.launch.py use_demo:=true  # 가짜 목표 + IK
ros2 launch sh_gen2_teleop quest_ik_demo.launch.py                 # 전체 (UDP + IK)
```

## URDF 확정 시 교체할 값 (config/quest_ik.yaml)

- `left_neutral` / `right_neutral`: SH_Humanoid_v2 홈 포즈 FK 계산값 적용됨 (URDF 개정 시 재계산)
- `left_ik_link` / `right_ik_link`: `left_hand_link` / `right_hand_link` (SRDF tip_link) 적용됨
- Unity `ControllerPoseLogger.cs`의 `leftNeutralPosition` / `rightNeutralPosition`도 같은 값으로 교체

## 참고

- `moveit_ik_bridge`는 URDF + MoveIt(move_group)이 실행 중이어야 동작한다.
  그 전에는 "`/compute_ik` 서비스 대기 중" 로그만 나오는 것이 정상.
- IK는 현재 6D(위치+회전). position-only가 필요하면 moveit_ik_bridge.py의 TODO 참고.
