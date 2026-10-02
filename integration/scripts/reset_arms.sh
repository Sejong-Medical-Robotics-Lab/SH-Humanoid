#!/bin/bash
# 팔이 꼬였을 때 준비 자세로 복귀 (궤적 방식 — 실물에서도 안전)
# 전제: Servo(터미널 3)를 끈 상태
exec "$(dirname "$(readlink -f "$0")")/ready.sh"
