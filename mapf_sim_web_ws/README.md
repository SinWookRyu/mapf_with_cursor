# MAPF 시뮬레이터 - 웹 버전

웹 브라우저에서 MAPF 시뮬레이터를 실행하고 제어할 수 있는 고급 웹 기반 시스템입니다.

## 🚀 주요 기능

- **웹 기반 시각화**: HTML5 Canvas를 사용한 실시간 시뮬레이션 표시
- **실시간 제어**: 웹 인터페이스를 통한 파라미터 조정 및 시뮬레이션 제어
- **WebSocket 통신**: 실시간 데이터 전송으로 부드러운 시각화
- **반응형 디자인**: 데스크톱과 모바일 모두 지원 (1920x1200 125% 줌 최적화)
- **ROS2 연동**: 기존 ROS2 시뮬레이터와 완벽 호환
- **고급 충돌 시스템**: 충돌 감지, 예측, 회피 알고리즘
- **프리셋 설정**: 성능, 부드러운 이동, 빠른 이동 프리셋 제공
- **동적 UI**: 리사이즈 가능한 제어 패널
- **상세 모니터링**: 실시간 충돌 상태 및 에이전트 정보 표시

## 📁 프로젝트 구조

```
mapf_sim_web_ws/
├── web_server/                 # Flask 웹 서버
│   ├── app.py                 # 메인 웹 애플리케이션
│   ├── requirements.txt       # Python 의존성
│   └── templates/             # HTML 템플릿
│       └── simulator.html    # 메인 시뮬레이터 페이지
├── web_visualizer/            # ROS2 웹 시각화 노드
│   ├── web_viz_node.py       # 웹 시각화 노드
│   └── requirements.txt       # Python 의존성
├── mapf_simulator/            # 기존 MAPF 시뮬레이터 (수정됨)
├── start_web_server.sh        # 웹 서버만 시작
├── start_complete_system.sh   # 전체 시스템 시작
└── README.md                  # 이 파일
```

## 🛠️ 설치 및 실행

### 1. 의존성 설치

```bash
# 웹 서버 의존성
cd web_server
pip3 install -r requirements.txt

# 웹 시각화 노드 의존성
cd ../web_visualizer
pip3 install -r requirements.txt

# ROS2 워크스페이스 빌드
cd ..
colcon build --packages-select mapf_simulator
source install/setup.bash
```

### 2. 시스템 실행

#### 전체 시스템 시작 (권장)
```bash
./start_complete_system.sh
```

#### 웹 서버만 시작
```bash
./start_web_server.sh
```

### 3. 웹 브라우저 접속

브라우저에서 `http://localhost:5000`으로 접속하세요.

### 4. 명령행 옵션

#### 상세 로깅 모드로 실행
```bash
# 웹 서버 (상세 로깅)
python3 web_server/app.py --verbose

# 웹 시각화 노드 (상세 로깅)
python3 web_visualizer/web_viz_node.py --verbose

# 포트 변경 (기본: 5000/5001)
python3 web_server/app.py --port 8080
python3 web_visualizer/web_viz_node.py --port 8081
```

## 🎮 사용법

### 시뮬레이션 제어
- **시뮬레이션 시작/중지**: 토글 버튼으로 간편한 제어
- **시뮬레이션 리셋**: 모든 에이전트를 시작 위치로 초기화
- **충돌 영역 표시/숨기기**: 충돌 감지 영역 시각화 토글

### 파라미터 조정
1. **기본 설정 탭**: 
   - 에이전트 수 (1-10개)
   - 움직임 모드 (연속/그리드)
   - 이동 속도 및 그리드 이동 간격
   - 월드 경계 설정

2. **에이전트 탭**: 
   - 각 에이전트의 시작/목표 위치 설정
   - 실시간 위치 조정
   - 에이전트별 개별 설정

3. **충돌 설정 탭**: 
   - **프리셋 선택**: 성능향상, 부드러운 이동, 빠른 이동
   - **상세 설정**: 충돌 감지 거리, 회피 강도, 예측 시간 등
   - **실시간 조정**: 설정 변경 즉시 적용

### 고급 기능
- **충돌 시각화**: 충돌 중인 에이전트는 빨간색으로 표시
- **충돌 예측**: 충돌이 예상되는 에이전트는 주황색으로 표시
- **동적 UI**: 제어 패널 크기 조정 가능
- **상세 모니터링**: 현재 설정값 및 에이전트 상태 실시간 표시

## 🔧 기술 스택

- **백엔드**: Flask, Flask-SocketIO
- **프론트엔드**: HTML5, CSS3, JavaScript, Canvas API
- **실시간 통신**: WebSocket (Socket.IO)
- **ROS2 연동**: rclpy, std_msgs, visualization_msgs
- **데이터 시각화**: HTML5 Canvas

## 🌐 웹 인터페이스 구성

### 제어 패널 (좌측)
- 시뮬레이션 상태 표시
- 탭 기반 설정 인터페이스
- 실시간 에이전트 정보

### 시뮬레이션 캔버스 (우측)
- 800x600 픽셀 Canvas
- 실시간 에이전트 위치 표시
- 격자 시각화
- 충돌 영역 표시

## 📡 API 엔드포인트

### 기본 API
- `GET /`: 메인 시뮬레이터 페이지
- `GET /api/status`: 시뮬레이션 상태 조회
- `POST /api/start`: 시뮬레이션 시작
- `POST /api/stop`: 시뮬레이션 중지
- `POST /api/reset`: 시뮬레이션 리셋

### 설정 API
- `POST /api/update_agents`: 에이전트 정보 업데이트
- `POST /api/update_grid`: 격자 정보 업데이트
- `POST /api/update_start_goal`: 시작/목표 위치 업데이트
- `POST /api/update_basic_settings`: 기본 설정 업데이트
- `POST /api/update_collision_settings`: 충돌 설정 업데이트
- `POST /api/update_world_settings`: 월드 설정 업데이트

### 충돌 관련 API
- `POST /api/update_collision_info`: 충돌 정보 업데이트
- `POST /api/update_collision_zones`: 충돌 영역 정보 업데이트
- `POST /api/toggle_collision_zones`: 충돌 영역 시각화 토글

## 🔌 WebSocket 이벤트

### 연결 이벤트
- `connect`: 클라이언트 연결
- `disconnect`: 클라이언트 연결 해제

### 시뮬레이션 이벤트
- `simulation_state`: 시뮬레이션 상태 전송
- `simulation_started`: 시뮬레이션 시작 알림
- `simulation_stopped`: 시뮬레이션 중지 알림
- `simulation_reset`: 시뮬레이션 리셋 알림

### 데이터 업데이트 이벤트
- `agents_updated`: 에이전트 정보 업데이트
- `grid_updated`: 격자 정보 업데이트
- `start_goal_poses_updated`: 시작/목표 위치 업데이트
- `collision_info_updated`: 충돌 정보 업데이트
- `collision_zones_updated`: 충돌 영역 정보 업데이트

### 제어 이벤트
- `control_command`: 제어 명령 수신
- `command_response`: 명령 처리 응답

## 🚨 문제 해결

### 웹 서버가 시작되지 않는 경우
```bash
# 포트 5000이 사용 중인지 확인
sudo netstat -tulpn | grep :5000

# 사용 중인 프로세스 종료
sudo kill -9 <PID>

# 또는 전체 시스템 프로세스 정리
./start_complete_system.sh
```

### ROS2 노드가 연결되지 않는 경우
```bash
# ROS2 환경 설정 확인
echo $ROS_DISTRO

# 워크스페이스 빌드 확인
colcon build --packages-select mapf_simulator
source install/setup.bash

# ROS2 노드 상태 확인
ros2 node list
ros2 topic list
```

### 웹 브라우저에서 연결 오류가 발생하는 경우
- 브라우저 캐시 및 쿠키 삭제
- 다른 브라우저로 시도
- 방화벽 설정 확인
- `http://localhost:5000` 대신 `http://127.0.0.1:5000` 시도

### 시뮬레이션이 시작되지 않는 경우
```bash
# 웹 시각화 노드 상태 확인
ros2 node list | grep web_visualizer

# ROS2 토픽 발행 확인
ros2 topic echo /agent_poses --once
ros2 topic echo /start_simulation --once
```

### 충돌 설정이 적용되지 않는 경우
- 웹 인터페이스에서 "상세 설정 표시" 체크박스 확인
- 설정 변경 후 "상세 설정 적용" 버튼 클릭
- 프리셋 선택 시 자동 적용됨

### 성능 최적화
- 에이전트 수를 5개 이하로 제한
- 충돌 감지 거리를 적절히 조정
- 상세 로깅 비활성화 (`--verbose` 옵션 제거)

## 📝 개발자 정보

이 프로젝트는 ROS2 기반 MAPF 시뮬레이터를 완전히 웹 기반으로 재구성한 고급 시스템입니다.

### 주요 구성 요소
- `simple_mapf_node.py`: 웹 시각화 노드와의 연동을 위한 수정된 시뮬레이터
- `web_viz_node.py`: ROS2 데이터를 웹 서버로 전송하는 노드 (Flask 서버 포함)
- `app.py`: Flask 기반 웹 서버 및 WebSocket 처리
- `simulator.html`: HTML5 Canvas 기반 고급 시각화 인터페이스

### 새로운 기능들
- **고급 충돌 시스템**: 충돌 감지, 예측, 회피 알고리즘
- **프리셋 설정**: 성능, 부드러운 이동, 빠른 이동 프리셋
- **동적 UI**: 리사이즈 가능한 제어 패널
- **실시간 모니터링**: 충돌 상태 및 에이전트 정보 표시
- **명령행 옵션**: 상세 로깅 및 포트 설정

### RViz 관련 파일 제거
- RViz 설정 파일들 (`*.rviz`)
- Launch 파일들 (`*.launch.py`)
- 기존 GUI 제어 노드 (`gui_control_node.py`)
- RViz 관련 스크립트들

### 시스템 아키텍처
```
웹 브라우저 (HTML5 Canvas)
    ↕ WebSocket/HTTP
Flask 웹 서버 (app.py)
    ↕ HTTP API
웹 시각화 노드 (web_viz_node.py)
    ↕ ROS2 Topics
MAPF 시뮬레이터 (simple_mapf_node.py)
```

## 📄 라이선스

이 프로젝트는 기존 MAPF 시뮬레이터와 동일한 라이선스를 따릅니다.

## 🔄 버전 히스토리

- **v2.0**: 웹 기반 시스템으로 완전 전환
- **v2.1**: 고급 충돌 시스템 및 프리셋 추가
- **v2.2**: 동적 UI 및 실시간 모니터링 기능 추가
- **v2.3**: 성능 최적화 및 안정성 개선
