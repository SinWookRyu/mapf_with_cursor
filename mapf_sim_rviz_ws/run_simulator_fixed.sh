#!/bin/bash

# MAPF 시뮬레이터 실행 스크립트 (개선된 버전)

echo "MAPF 시뮬레이터를 시작합니다 (개선된 버전)..."

# WSL GUI 환경 설정
export DISPLAY=:0
export LIBGL_ALWAYS_INDIRECT=1

# 워크스페이스 설정
source install/setup.bash

# 시뮬레이터 실행 (새로운 launch 파일 사용)
echo "시뮬레이터, GUI, RViz를 시작합니다..."
ros2 launch mapf_simulator mapf_simulator_fixed.launch.py
