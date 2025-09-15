# WSL에서 RViz 실행 문제 해결 가이드

## 문제 상황
WSL 환경에서 RViz가 OpenGL 컨텍스트 생성 오류로 실행되지 않는 문제

## 해결 방법

### 방법 1: Windows에서 X11 서버 설정

#### 1단계: Windows에서 VcXsrv 설치
```bash
# Windows에서 실행
# 1. VcXsrv 다운로드: https://sourceforge.net/projects/vcxsrv/
# 2. 설치 후 실행
# 3. Display settings: Multiple windows
# 4. Client startup: Start no client
# 5. Extra settings: Disable access control 체크
```

#### 2단계: WSL에서 환경변수 설정
```bash
export DISPLAY=:0
export LIBGL_ALWAYS_INDIRECT=1
```

### 방법 2: WSL2 GPU 지원 활성화

#### 1단계: Windows에서 WSL2 GPU 지원 확인
```powershell
# Windows PowerShell에서 실행
wsl --update
```

#### 2단계: WSL에서 GPU 확인
```bash
nvidia-smi  # NVIDIA GPU가 있는 경우
```

### 방법 3: 대안 시각화 도구

#### A. 웹 기반 시각화
```bash
# ROS2 웹 브리지 설치
sudo apt install ros-humble-rosbridge-suite
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

#### B. 터미널 기반 모니터링
```bash
# 에이전트 상태 실시간 확인
ros2 topic echo /agent_poses

# 시뮬레이션 리셋
ros2 topic pub /reset_simulation std_msgs/msg/Bool "data: true" --once
```

### 방법 4: 수정된 실행 스크립트

#### 새로운 실행 스크립트 사용
```bash
# RViz 없이 시뮬레이터만 실행
./run_simulator_no_rviz.sh

# 또는 수동으로 노드 실행
ros2 run mapf_simulator simple_mapf_node &
ros2 run mapf_simulator gui_control_node &
```

## 즉시 사용 가능한 명령어

### 시뮬레이터 실행
```bash
# 방법 1: 개별 노드 실행
ros2 run mapf_simulator simple_mapf_node

# 방법 2: 테스트 노드 실행
python3 test_agent_visualization.py
```

### 에이전트 모니터링
```bash
# 실시간 에이전트 위치 확인
ros2 topic echo /agent_poses

# 시뮬레이션 리셋
ros2 topic pub /reset_simulation std_msgs/msg/Bool "data: true" --once
```

## 현재 상태
✅ **시뮬레이터**: 정상 작동  
✅ **에이전트 데이터**: 정상 발행  
❌ **RViz**: WSL GUI 환경 문제로 실행 불가

## 권장 해결 순서
1. Windows에서 VcXsrv 설치 및 실행
2. WSL에서 환경변수 설정
3. 새로운 실행 스크립트 사용
4. 웹 기반 시각화 고려
