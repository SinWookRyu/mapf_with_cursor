#!/bin/bash

# MAPF 시뮬레이터 실행 스크립트 (RViz 문제 해결 버전)

echo "MAPF 시뮬레이터를 시작합니다 (RViz 문제 해결 버전)..."

# WSL GUI 환경 설정 (더 안정적인 설정)
export DISPLAY=:0
export LIBGL_ALWAYS_INDIRECT=1
export MESA_GL_VERSION_OVERRIDE=3.3
export MESA_GLSL_VERSION_OVERRIDE=330

# 워크스페이스 설정
source install/setup.bash

# 시뮬레이터 노드만 먼저 실행
echo "시뮬레이터 노드를 시작합니다..."
ros2 run mapf_simulator simple_mapf_node &
SIMULATOR_PID=$!

# GUI 컨트롤 노드 실행
echo "GUI 컨트롤 노드를 시작합니다..."
ros2 run mapf_simulator gui_control_node &
GUI_PID=$!

# Static transform publisher 실행
echo "Static transform publisher를 시작합니다..."
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world map &
TF_PID=$!

# 잠시 기다린 후 RViz 실행
echo "3초 후 RViz를 시작합니다..."
sleep 3

# RViz 실행 (더 안정적인 설정으로)
echo "RViz를 시작합니다..."
rviz2 -d install/mapf_simulator/share/mapf_simulator/config/mapf_simulator_fixed.rviz &
RVIZ_PID=$!

echo "모든 노드가 시작되었습니다."
echo "시뮬레이터 PID: $SIMULATOR_PID"
echo "GUI 컨트롤 PID: $GUI_PID"
echo "TF Publisher PID: $TF_PID"
echo "RViz PID: $RVIZ_PID"
echo ""
echo "시뮬레이션을 종료하려면 Ctrl+C를 누르세요."

# 프로세스 종료 함수
cleanup() {
    echo ""
    echo "시뮬레이션을 종료합니다..."
    kill $SIMULATOR_PID 2>/dev/null
    kill $GUI_PID 2>/dev/null
    kill $TF_PID 2>/dev/null
    kill $RVIZ_PID 2>/dev/null
    exit 0
}

# Ctrl+C 시그널 처리
trap cleanup SIGINT

# 모든 프로세스가 종료될 때까지 대기
wait
