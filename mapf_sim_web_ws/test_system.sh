#!/bin/bash

echo "MAPF 시뮬레이터 웹 시스템 테스트를 시작합니다..."

# 워크스페이스 설정
cd /home/mnerd/mapf_sim_web_ws

# 기존 프로세스 정리
echo "기존 프로세스를 정리합니다..."
pkill -f "ros2\|rviz\|simple_mapf_node\|gui_control_node\|web_viz_node" 2>/dev/null || true
pkill -f "python3.*app.py" 2>/dev/null || true
sleep 2

# ROS2 환경 설정
echo "ROS2 환경을 설정합니다..."
source /opt/ros/humble/setup.bash
source install/setup.bash

# 1. 웹 서버 시작
echo "1. 웹 서버를 시작합니다..."
cd web_server
python3 app.py &
WEB_SERVER_PID=$!
cd ..
sleep 3

# 웹 서버 상태 확인
echo "웹 서버 상태를 확인합니다..."
if curl -s http://localhost:5000/api/status > /dev/null; then
    echo "✅ 웹 서버가 성공적으로 시작되었습니다."
else
    echo "❌ 웹 서버 시작에 실패했습니다."
    exit 1
fi

# 2. Static Transform Publisher 시작
echo "2. Static Transform Publisher를 시작합니다..."
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world map &
TF_PID=$!
sleep 2

# 3. MAPF 시뮬레이터 노드 시작
echo "3. MAPF 시뮬레이터 노드를 시작합니다..."
ros2 run mapf_simulator simple_mapf_node &
MAPF_PID=$!
sleep 3

# MAPF 노드 상태 확인
if ps -p $MAPF_PID > /dev/null; then
    echo "✅ MAPF 시뮬레이터 노드가 시작되었습니다. PID: $MAPF_PID"
else
    echo "❌ MAPF 시뮬레이터 노드 시작에 실패했습니다."
    exit 1
fi

# 4. 웹 시각화 노드 시작
echo "4. 웹 시각화 노드를 시작합니다..."
cd web_visualizer
python3 web_viz_node.py &
WEB_VIZ_PID=$!
cd ..
sleep 3

# 웹 시각화 노드 상태 확인
if ps -p $WEB_VIZ_PID > /dev/null; then
    echo "✅ 웹 시각화 노드가 시작되었습니다. PID: $WEB_VIZ_PID"
else
    echo "❌ 웹 시각화 노드 시작에 실패했습니다."
    exit 1
fi

# 5. 시스템 상태 확인
echo "5. 시스템 상태를 확인합니다..."
sleep 2

echo ""
echo "=== 시스템 상태 확인 ==="
echo "실행 중인 프로세스:"
ps aux | grep -E "(python3.*app.py|ros2|simple_mapf_node|web_viz_node)" | grep -v grep

echo ""
echo "ROS2 노드 목록:"
ros2 node list

echo ""
echo "ROS2 토픽 목록:"
ros2 topic list

echo ""
echo "=== MAPF 시뮬레이터 웹 시스템이 시작되었습니다 ==="
echo "웹 브라우저에서 http://localhost:5000 으로 접속하세요."
echo ""
echo "시스템을 중단하려면 Ctrl+C를 누르세요."
echo ""

# 모든 프로세스가 종료될 때까지 대기
wait
