#!/bin/bash

# MAPF 시뮬레이터 실행 스크립트 (RViz 없이)

echo "MAPF 시뮬레이터를 시작합니다 (RViz 없이)..."

# 워크스페이스 설정
source install/setup.bash

# 시뮬레이터 노드 실행
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

echo "모든 노드가 시작되었습니다."
echo "시뮬레이터 PID: $SIMULATOR_PID"
echo "GUI 컨트롤 PID: $GUI_PID"
echo "TF Publisher PID: $TF_PID"
echo ""
echo "에이전트 상태 확인: ros2 topic echo /agent_poses"
echo "시뮬레이션 리셋: ros2 topic pub /reset_simulation std_msgs/msg/Bool 'data: true' --once"
echo "시뮬레이션을 종료하려면 Ctrl+C를 누르세요."

# 프로세스 종료 함수
cleanup() {
    echo ""
    echo "시뮬레이션을 종료합니다..."
    kill $SIMULATOR_PID 2>/dev/null
    kill $GUI_PID 2>/dev/null
    kill $TF_PID 2>/dev/null
    exit 0
}

# Ctrl+C 시그널 처리
trap cleanup SIGINT

# 모든 프로세스가 종료될 때까지 대기
wait

