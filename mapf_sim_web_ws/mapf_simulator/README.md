# MAPF 시뮬레이터 (Multi-Agent Path Finding Simulator)

ROS2 기반의 다중 에이전트 경로 계획 시뮬레이터입니다. 각 에이전트는 연속적으로 동작하며 충돌을 자동으로 감지하고 회피하여 경로를 계획합니다.

## 주요 기능

- **다중 에이전트 시뮬레이션**: 여러 에이전트가 동시에 목표 지점으로 이동
- **충돌 감지 및 회피**: 에이전트 간 충돌과 장애물 충돌을 자동으로 감지하고 회피
- **실시간 시각화**: RViz를 통한 실시간 에이전트 위치 및 경로 시각화
- **GUI 제어**: Tkinter 기반 GUI를 통한 시뮬레이션 파라미터 조정
- **충돌 영역 설정**: 사용자가 충돌 영역을 동적으로 추가/제거 가능

## 설치 및 빌드

### 1. 워크스페이스 빌드

```bash
cd mapf_sim_ws
colcon build
source install/setup.bash
```

### 2. 의존성 설치

```bash
sudo apt update
sudo apt install python3-numpy python3-tk
```

## 사용법

### 1. 전체 시뮬레이션 실행

```bash
ros2 launch mapf_simulator mapf_simulator.launch.py
```

이 명령어는 다음을 실행합니다:
- MAPF 시뮬레이터 노드
- GUI 제어 노드
- RViz 시각화

### 2. 개별 노드 실행

#### 시뮬레이터만 실행
```bash
ros2 run mapf_simulator mapf_simulator_node
```

#### GUI 제어만 실행
```bash
ros2 run mapf_simulator gui_control_node
```

#### RViz만 실행
```bash
rviz2 -d src/mapf_simulator/config/mapf_simulator.rviz
```

### 3. 파라미터 설정

Launch 파일에서 파라미터를 설정할 수 있습니다:

```bash
ros2 launch mapf_simulator mapf_simulator.launch.py num_agents:=5 world_width:=30.0 world_height:=30.0
```

## GUI 제어 기능

### 에이전트 설정
- 에이전트 수 조정 (1-10개)
- 실시간으로 에이전트 수 변경 가능

### 월드 설정
- 월드 크기 조정 (너비, 높이)
- 시뮬레이션 공간 크기 변경

### 충돌 설정
- 충돌 감지 거리 조정
- 에이전트 간 최소 안전 거리 설정

### 충돌 영역 관리
- 새로운 충돌 영역 추가 (위치, 반지름 설정)
- 모든 충돌 영역 제거
- 동적 장애물 생성

### 시뮬레이션 제어
- 시뮬레이션 리셋
- 업데이트 속도 조정

## 시각화

### RViz에서 확인 가능한 요소들

1. **에이전트 위치** (`/agent_poses`)
   - 각 에이전트는 고유한 색상으로 표시
   - 에이전트 ID가 텍스트로 표시

2. **에이전트 경로** (`/agent_paths`)
   - 각 에이전트의 이동 경로를 선으로 표시
   - 반투명한 선으로 경로 히스토리 표시

3. **충돌 영역** (`/collision_zones`)
   - 빨간색 원형 장애물로 표시
   - 반투명한 색상으로 표시

4. **월드 경계** (`/world_bounds`)
   - 파란색 선으로 시뮬레이션 영역 표시

## 토픽 구조

### 발행 토픽
- `/agent_poses` (visualization_msgs/MarkerArray): 에이전트 위치
- `/agent_paths` (visualization_msgs/MarkerArray): 에이전트 경로
- `/collision_zones` (visualization_msgs/MarkerArray): 충돌 영역
- `/world_bounds` (visualization_msgs/MarkerArray): 월드 경계

### 구독 토픽 (GUI 제어)
- `/num_agents` (std_msgs/Int32): 에이전트 수 변경
- `/world_width` (std_msgs/Float32): 월드 너비 변경
- `/world_height` (std_msgs/Float32): 월드 높이 변경
- `/collision_distance` (std_msgs/Float32): 충돌 거리 변경
- `/update_rate` (std_msgs/Float32): 업데이트 속도 변경
- `/reset_simulation` (std_msgs/Bool): 시뮬레이션 리셋
- `/add_collision_zone` (geometry_msgs/Point): 충돌 영역 추가
- `/clear_collision_zones` (std_msgs/Bool): 충돌 영역 모두 제거

## 알고리즘 설명

### 충돌 회피 알고리즘
1. **에이전트 간 충돌 회피**: 각 에이전트는 다른 에이전트와의 거리를 계산하여 회피 힘을 생성
2. **장애물 회피**: 충돌 영역과의 거리를 계산하여 장애물에서 멀어지는 힘을 생성
3. **힘 합성**: 목표 지점으로의 힘과 회피 힘을 합성하여 최종 이동 방향 결정

### 경로 계획
- 각 에이전트는 시작점에서 목표점까지 직선 경로를 계획
- 충돌이 감지되면 실시간으로 경로를 조정
- 목표 지점에 도달하면 이동을 중단

## 파일 구조

```
mapf_simulator/
├── mapf_simulator/
│   ├── __init__.py
│   ├── mapf_simulator_node.py    # 메인 시뮬레이터 노드
│   └── gui_control_node.py       # GUI 제어 노드
├── launch/
│   └── mapf_simulator.launch.py  # Launch 파일
├── config/
│   └── mapf_simulator.rviz       # RViz 설정 파일
├── setup.py                      # 패키지 설정
├── package.xml                   # 패키지 메타데이터
└── README.md                     # 이 파일
```

## 문제 해결

### 일반적인 문제들

1. **GUI가 열리지 않는 경우**
   ```bash
   sudo apt install python3-tk
   ```

2. **RViz에서 마커가 보이지 않는 경우**
   - Fixed Frame이 "map"으로 설정되어 있는지 확인
   - 토픽이 발행되고 있는지 확인: `ros2 topic list`

3. **시뮬레이션이 느린 경우**
   - 업데이트 속도를 낮춰보세요 (GUI에서 조정 가능)
   - 에이전트 수를 줄여보세요

## 라이선스

이 프로젝트는 MIT 라이선스 하에 배포됩니다. 