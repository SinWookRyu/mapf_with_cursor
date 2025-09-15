#!/bin/bash

# 에이전트 상태 모니터링 스크립트

echo "MAPF 시뮬레이터 에이전트 모니터링을 시작합니다..."
echo "Ctrl+C를 눌러 종료하세요."
echo ""

# 에이전트 상태를 실시간으로 모니터링
ros2 topic echo /agent_poses --once | while read line; do
    if [[ $line == *"position:"* ]]; then
        echo "에이전트 위치 업데이트: $line"
    elif [[ $line == *"color:"* ]]; then
        echo "에이전트 색상: $line"
    fi
done
