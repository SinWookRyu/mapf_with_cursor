#!/bin/bash

# MAPF 시뮬레이터 실행 스크립트

echo "MAPF 시뮬레이터를 시작합니다..."

# 기존 프로세스 정리
echo "기존 ROS2 프로세스를 정리합니다..."
pkill -f "ros2\|rviz\|simple_mapf_node\|gui_control_node" 2>/dev/null || true
sleep 2

# 워크스페이스 설정
echo "워크스페이스를 설정합니다..."
cd /home/mnerd/mapf_sim_ws
source install/setup.bash

# TF2 관련 패키지 확인
echo "TF2 관련 패키지를 확인합니다..."
if ! dpkg -l | grep -q "ros-humble-tf2-ros"; then
    echo "TF2 패키지를 설치합니다..."
    sudo apt update
    sudo apt install -y ros-humble-tf2-ros ros-humble-tf2-tools
fi

# 시뮬레이터 실행
echo "시뮬레이터를 시작합니다..."
echo "RViz 설정 파일이 자동으로 로딩됩니다."
echo "GUI에서 '시뮬레이션 시작' 버튼을 눌러 시뮬레이션을 시작하세요."
echo ""
echo "GUI에서 다음 기능들을 사용할 수 있습니다:"
echo "- 에이전트 수 조정 및 위치 설정"
echo "- 이동 모드 선택 (연속/그리드)"
echo "- 격자 크기 및 이동 속도 조정"
echo "- 충돌 회피 파라미터 조정"
echo "- 충돌 영역 관리"
echo ""

# 각 노드를 개별적으로 실행
echo "1. Static Transform Publisher 시작..."
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world map &
sleep 0.5

echo "2. MAPF 시뮬레이터 노드 시작..."
ros2 run mapf_simulator simple_mapf_node &
sleep 1

echo "3. GUI 컨트롤 노드 시작..."
ros2 run mapf_simulator gui_control_node &
sleep 0.5

echo "4. RViz 시작..."
ros2 run rviz2 rviz2 -d config/mapf_simulator.rviz &
sleep 0.5

echo "모든 노드가 시작되었습니다."
echo "시뮬레이션을 중단하려면 Ctrl+C를 누르세요."

# 모든 백그라운드 프로세스가 종료될 때까지 대기
wait 