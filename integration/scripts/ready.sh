#!/bin/bash
# 준비 자세로 이동 → forward 컨트롤러로 전환 (터미널 1 demo가 켜진 뒤 실행)
source ~/Desktop/sh_gen2_ws/install/setup.bash
POSE="[0.0, 0.0, 0.0, -0.4, 0.0, 0.3, 0.0]"
for side in left right; do
ros2 topic pub --once /${side}_arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
"{joint_names: [${side}_shoulder_pitch_joint, ${side}_shoulder_roll_joint, ${side}_shoulder_yaw_joint, ${side}_elbow_joint, ${side}_wrist_yaw_joint, ${side}_wrist_pitch_joint, ${side}_wrist_roll_joint],
  points: [{positions: $POSE, time_from_start: {sec: 2}}]}"
done
sleep 3
ros2 control load_controller --set-state inactive left_arm_forward_controller
ros2 control load_controller --set-state inactive right_arm_forward_controller
ros2 control switch_controllers --deactivate left_arm_controller right_arm_controller \
                                --activate left_arm_forward_controller right_arm_forward_controller
