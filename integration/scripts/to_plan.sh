#!/bin/bash
# forward 컨트롤러 → 궤적 컨트롤러로 되돌리기 (모션 플래닝 모드)
set -e
ros2 control switch_controllers \
  --deactivate left_arm_forward_controller right_arm_forward_controller \
  --activate left_arm_controller right_arm_controller
echo "플래닝 모드. 활성 컨트롤러:"
ros2 control list_controllers | grep -E "forward|arm_controller"
