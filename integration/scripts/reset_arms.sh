#!/bin/bash
# 팔이 멈추거나 꼬였을 때 준비 자세로 즉시 복귀 (터미널 3 끈 상태에서)
source ~/Desktop/sh_gen2_ws/install/setup.bash
for side in left right; do
ros2 topic pub --once /${side}_arm_forward_controller/commands std_msgs/msg/Float64MultiArray \
"{data: [0.0, 0.0, 0.0, -0.4, 0.0, 0.3, 0.0]}"
done
