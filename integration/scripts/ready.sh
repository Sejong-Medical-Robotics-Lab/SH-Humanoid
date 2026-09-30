#!/bin/bash
# 준비 자세로 이동 → Servo용 forward 컨트롤러로 전환
# 전제: demo.launch.py 실행 중, ROS 환경 source 완료
set -e
source "$(dirname "$0")/pose.env"

JOINTS() { echo "[${1}_shoulder_pitch_joint, ${1}_shoulder_roll_joint, ${1}_shoulder_yaw_joint, ${1}_elbow_joint, ${1}_wrist_yaw_joint, ${1}_wrist_pitch_joint, ${1}_wrist_roll_joint]"; }

echo "[1/3] 궤적 컨트롤러 활성화"
ros2 control switch_controllers \
  --deactivate left_arm_forward_controller right_arm_forward_controller \
  --activate left_arm_controller right_arm_controller 2>/dev/null || true

echo "[2/3] 준비 자세로 이동 (3초)"
for side in left right; do
  ros2 topic pub --times 3 /${side}_arm_controller/joint_trajectory \
    trajectory_msgs/msg/JointTrajectory \
    "{joint_names: $(JOINTS $side), points: [{positions: [$READY_POSE], time_from_start: {sec: 2}}]}" >/dev/null
done
sleep 3

echo "[3/3] forward 컨트롤러로 전환"
ros2 control load_controller --set-state inactive left_arm_forward_controller 2>/dev/null || true
ros2 control load_controller --set-state inactive right_arm_forward_controller 2>/dev/null || true
ros2 control switch_controllers \
  --deactivate left_arm_controller right_arm_controller \
  --activate left_arm_forward_controller right_arm_forward_controller

echo "완료. 활성 컨트롤러:"
ros2 control list_controllers | grep -E "forward|arm_controller"
