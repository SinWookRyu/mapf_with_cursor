# MAPF 시뮬레이터 - RViz 버전 (Multi-Agent Path Finding Simulator)

ROS2 기반의 다중 에이전트 경로 계획 시뮬레이터입니다. RViz를 사용한 전통적인 시각화 방식으로 각 에이전트는 연속적으로 동작하며 충돌을 자동으로 감지하고 회피하여 경로를 계획합니다.

> **참고**: 이 프로젝트는 **레거시 버전**입니다. 새로운 웹 기반 버전은 [mapf_sim_web_ws](../mapf_sim_web_ws/README.md)를 참조하세요.

## 🚀 주요 기능

- **다중 에이전트 시뮬레이션**: 여러 에이전트가 동시에 목표 지점으로 이동
- **충돌 감지 및 회피**: 에이전트 간 충돌과 장애물 충돌을 자동으로 감지하고 회피
- **RViz 시각화**: ROS2의 표준 시각화 도구인 RViz를 통한 실시간 시각화
- **GUI 제어**: Tkinter 기반 GUI를 통한 시뮬레이션 파라미터 조정
- **충돌 영역 설정**: 사용자가 충돌 영역을 동적으로 추가/제거 가능
- **그리드 이동 모드**: 1-칸씩 정확한 그리드 기반 이동 지원

## 🛠️ 설치 및 빌드

### 1. 워크스페이스 빌드

```bash
cd mapf_sim_rviz_ws
colcon build --packages-select mapf_simulator
source install/setup.bash
```

### 2. 의존성 설치

```bash
sudo apt update
sudo apt install python3-numpy python3-tk
```

## 🎮 사용법

### 1. 전체 시뮬레이션 실행

```bash
# 스크립트를 사용한 실행 (권장)
./run_simulator.sh

# 또는 직접 launch 파일 실행
ros2 launch mapf_simulator mapf_simulator.launch.py
```

이 명령어는 다음을 실행합니다:
- MAPF 시뮬레이터 노드 (`simple_mapf_node`)
- GUI 제어 노드 (`gui_control_node`)
- RViz 시각화 (자동 config 로드)
- TF2 static transform publisher

### 2. 개별 노드 실행

#### 시뮬레이터만 실행
```bash
ros2 run mapf_simulator simple_mapf_node
```

#### GUI 제어만 실행
```bash
ros2 run mapf_simulator gui_control_node
```

#### RViz만 실행
```bash
rviz2 -d install/mapf_simulator/share/mapf_simulator/config/mapf_simulator.rviz
```

### 3. 파라미터 설정

Launch 파일에서 파라미터를 설정할 수 있습니다:

```bash
ros2 launch mapf_simulator mapf_simulator.launch.py num_agents:=5 world_width:=30.0 world_height:=30.0
```

## 🎛️ GUI 제어 기능

### 탭 기반 인터페이스
1. **기본 설정 탭**: 
   - 에이전트 수 조정 (1-10개)
   - 움직임 모드 (연속/그리드)
   - 이동 속도 및 그리드 이동 간격
   - 월드 크기 조정 (너비, 높이)

2. **에이전트 탭**: 
   - 각 에이전트의 시작/목표 위치 설정
   - 실시간 위치 조정
   - 에이전트별 개별 설정

3. **충돌 설정 탭**: 
   - 충돌 감지 거리 조정
   - 회피 강도 설정
   - 충돌 대기 시간 조정
   - 에이전트 반지름 설정

### 충돌 영역 관리
- 새로운 충돌 영역 추가 (위치, 반지름 설정)
- 모든 충돌 영역 제거
- 동적 장애물 생성

### 시뮬레이션 제어
- 시뮬레이션 시작/중지 (GUI 버튼으로 제어)
- 시뮬레이션 리셋
- 업데이트 속도 조정

## 🖥️ 시각화

### RViz에서 확인 가능한 요소들

1. **에이전트 위치** (`/agent_poses`)
   - 각 에이전트는 고유한 색상으로 표시
   - 에이전트 ID가 텍스트로 표시
   - 이동 중인 에이전트는 반투명하게 표시

2. **시작/목표 위치** (`/start_goal_poses`)
   - 시작 위치: 초록색 마커
   - 목표 위치: 빨간색 마커
   - 격자 모드에서 정확한 위치 표시

3. **격자 시각화** (`/grid_visualization`)
   - 격자 모드에서 격자선 표시
   - 격자 셀 크기 및 경계 표시

4. **충돌 영역** (`/collision_zones`)
   - 빨간색 원형 장애물로 표시
   - 반투명한 색상으로 표시

5. **월드 경계** (`/world_bounds`)
   - 파란색 선으로 시뮬레이션 영역 표시

### RViz 설정
- **Fixed Frame**: `world` (TF2 static transform publisher 사용)
- **자동 config 로드**: `run_simulator.sh` 실행 시 자동으로 config 파일 로드

## 📡 토픽 구조

### 발행 토픽
- `/agent_poses` (visualization_msgs/MarkerArray): 에이전트 위치
- `/start_goal_poses` (visualization_msgs/MarkerArray): 시작/목표 위치
- `/grid_visualization` (visualization_msgs/MarkerArray): 격자 시각화
- `/collision_zones` (visualization_msgs/MarkerArray): 충돌 영역
- `/world_bounds` (visualization_msgs/MarkerArray): 월드 경계

### 구독 토픽 (GUI 제어)
- `/num_agents` (std_msgs/Int32): 에이전트 수 변경
- `/world_width` (std_msgs/Float32): 월드 너비 변경
- `/world_height` (std_msgs/Float32): 월드 높이 변경
- `/collision_distance` (std_msgs/Float32): 충돌 거리 변경
- `/update_rate` (std_msgs/Float32): 업데이트 속도 변경
- `/start_simulation` (std_msgs/Bool): 시뮬레이션 시작/중지
- `/reset_simulation` (std_msgs/Bool): 시뮬레이션 리셋
- `/add_collision_zone` (geometry_msgs/Point): 충돌 영역 추가
- `/clear_collision_zones` (std_msgs/Bool): 충돌 영역 모두 제거
- `/movement_mode` (std_msgs/Int32): 이동 모드 (0: 연속, 1: 그리드)
- `/grid_cell_size` (std_msgs/Float32): 격자 셀 크기
- `/grid_move_interval` (std_msgs/Float32): 격자 이동 간격
- `/movement_speed` (std_msgs/Float32): 이동 속도

## 🔬 알고리즘 설명

### 충돌 회피 알고리즘
1. **에이전트 간 충돌 회피**: 각 에이전트는 다른 에이전트와의 거리를 계산하여 회피 힘을 생성
2. **장애물 회피**: 충돌 영역과의 거리를 계산하여 장애물에서 멀어지는 힘을 생성
3. **힘 합성**: 목표 지점으로의 힘과 회피 힘을 합성하여 최종 이동 방향 결정

### 이동 모드
1. **연속 모드**: 부드러운 연속적인 이동
2. **그리드 모드**: 1-칸씩 정확한 격자 기반 이동
   - 대각선 이동 방지
   - 격자 인덱스 기반 충돌 검사
   - 정확한 목표 도달 판정

### 경로 계획
- 각 에이전트는 시작점에서 목표점까지 직선 경로를 계획
- 충돌이 감지되면 실시간으로 경로를 조정
- 목표 지점에 도달하면 이동을 중단
- 그리드 모드에서는 격자 단위로 정확한 이동

## 📁 파일 구조

```
mapf_sim_rviz_ws/
├── mapf_simulator/
│   ├── mapf_simulator/
│   │   ├── __init__.py
│   │   ├── simple_mapf_node.py   # 메인 시뮬레이터 노드
│   │   └── gui_control_node.py   # GUI 제어 노드
│   ├── launch/
│   │   └── mapf_simulator.launch.py  # Launch 파일
│   ├── config/
│   │   └── mapf_simulator.rviz       # RViz 설정 파일
│   ├── setup.py                      # 패키지 설정
│   ├── package.xml                   # 패키지 메타데이터
│   └── README.md                     # 이 파일
├── run_simulator.sh              # 시뮬레이터 실행 스크립트
└── README.md                     # 워크스페이스 README
```

## 🚨 문제 해결

### 일반적인 문제들

1. **GUI가 열리지 않는 경우**
   ```bash
   sudo apt install python3-tk
   ```

2. **RViz에서 마커가 보이지 않는 경우**
   - Fixed Frame이 "world"로 설정되어 있는지 확인
   - 토픽이 발행되고 있는지 확인: `ros2 topic list`
   - TF2 static transform publisher가 실행 중인지 확인

3. **시뮬레이션이 시작되지 않는 경우**
   - GUI에서 "시뮬레이션 시작" 버튼 클릭
   - ROS2 환경 설정 확인: `source install/setup.bash`

4. **에이전트가 움직이지 않는 경우**
   - 시작/목표 위치가 올바르게 설정되었는지 확인
   - 그리드 모드에서 격자 셀 크기 확인
   - 충돌 감지 거리 설정 확인

5. **시뮬레이션이 느린 경우**
   - 업데이트 속도를 낮춰보세요 (GUI에서 조정 가능)
   - 에이전트 수를 줄여보세요
   - 충돌 감지 거리를 적절히 조정하세요

### 디버깅 명령어
```bash
# ROS2 노드 상태 확인
ros2 node list

# 토픽 발행 확인
ros2 topic echo /agent_poses --once
ros2 topic echo /start_simulation --once

# TF 트리 확인
ros2 run tf2_tools view_frames
```

## 🔄 웹 버전과의 차이점

이 RViz 버전은 전통적인 ROS2 시각화 방식을 사용하며, 웹 버전과 다음과 같은 차이점이 있습니다:

| 기능 | RViz 버전 | 웹 버전 |
|------|-----------|---------|
| 시각화 | RViz | HTML5 Canvas |
| 제어 | Tkinter GUI | 웹 인터페이스 |
| 접근성 | 로컬 전용 | 웹 브라우저 |
| 충돌 시각화 | 기본 | 고급 (색상 변경) |
| 프리셋 | 없음 | 성능/부드러운/빠른 이동 |
| 실시간 모니터링 | 기본 | 상세한 상태 표시 |

## 📄 라이선스

이 프로젝트는 MIT 라이선스 하에 배포됩니다.

## 🔄 버전 히스토리

- **v1.0**: 기본 RViz 기반 시뮬레이터
- **v1.1**: 그리드 이동 모드 추가
- **v1.2**: 시작/목표 위치 시각화 추가
- **v1.3**: GUI 개선 및 안정성 향상 