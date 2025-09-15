# MAPF 시뮬레이터 프로젝트

Multi-Agent Path Finding (MAPF) 시뮬레이터 프로젝트입니다. ROS2 기반으로 개발되었으며, 두 가지 버전의 시각화 및 제어 인터페이스를 제공합니다.

## 🚀 프로젝트 개요

이 프로젝트는 다중 에이전트 경로 계획 시뮬레이션을 위한 완전한 솔루션을 제공합니다. 각 에이전트는 시작점에서 목표점까지 이동하며, 실시간 충돌 감지 및 회피 알고리즘을 통해 안전한 경로를 계획합니다.

## 📁 프로젝트 구조

```
mapf_sim_ws/
├── mapf_sim_web_ws/          # 웹 기반 시뮬레이터 (권장)
│   ├── web_server/           # Flask 웹 서버
│   ├── web_visualizer/       # ROS2 웹 시각화 노드
│   ├── mapf_simulator/       # ROS2 시뮬레이터 패키지
│   └── README.md            # 웹 버전 상세 문서
├── mapf_sim_rviz_ws/         # RViz 기반 시뮬레이터 (레거시)
│   ├── mapf_simulator/       # ROS2 시뮬레이터 패키지
│   └── README.md            # RViz 버전 상세 문서
└── README.md                 # 이 파일 (메인 문서)
```

## 🎯 버전별 특징

### 🌐 웹 버전 (mapf_sim_web_ws) - **권장**

**장점:**
- ✅ **웹 브라우저 접근**: 어디서든 접근 가능
- ✅ **고급 시각화**: HTML5 Canvas 기반 실시간 시각화
- ✅ **프리셋 설정**: 성능, 부드러운 이동, 빠른 이동 프리셋
- ✅ **실시간 모니터링**: 상세한 충돌 상태 및 에이전트 정보
- ✅ **동적 UI**: 리사이즈 가능한 제어 패널
- ✅ **충돌 시각화**: 충돌 중인 에이전트 색상 변경
- ✅ **명령행 옵션**: 상세 로깅 및 포트 설정

**기술 스택:**
- 백엔드: Flask, Flask-SocketIO
- 프론트엔드: HTML5, CSS3, JavaScript, Canvas API
- 실시간 통신: WebSocket (Socket.IO)
- ROS2 연동: rclpy, visualization_msgs

### 🖥️ RViz 버전 (mapf_sim_rviz_ws) - 레거시

**장점:**
- ✅ **ROS2 네이티브**: 표준 ROS2 시각화 도구 사용
- ✅ **안정성**: 검증된 RViz 기반 시각화
- ✅ **디버깅**: ROS2 표준 도구로 쉬운 디버깅
- ✅ **그리드 이동**: 1-칸씩 정확한 격자 기반 이동

**기술 스택:**
- 시각화: RViz2
- 제어: Tkinter GUI
- ROS2: rclpy, visualization_msgs

## 🚀 빠른 시작

### 웹 버전 사용 (권장)

```bash
cd mapf_sim_web_ws
./start_complete_system.sh
```

브라우저에서 `http://localhost:5000`으로 접속하세요.

### RViz 버전 사용

```bash
cd mapf_sim_rviz_ws
./run_simulator.sh
```

## 🎮 주요 기능

### 공통 기능
- **다중 에이전트 시뮬레이션**: 여러 에이전트가 동시에 목표 지점으로 이동
- **충돌 감지 및 회피**: 에이전트 간 충돌과 장애물 충돌을 자동으로 감지하고 회피
- **그리드 이동 모드**: 1-칸씩 정확한 격자 기반 이동
- **실시간 파라미터 조정**: 시뮬레이션 중 실시간으로 파라미터 변경 가능
- **시작/목표 위치 설정**: 각 에이전트의 시작점과 목표점 개별 설정

### 웹 버전 전용 기능
- **충돌 프리셋**: 성능향상, 부드러운 이동, 빠른 이동 프리셋
- **실시간 충돌 시각화**: 충돌 중인 에이전트는 빨간색, 충돌 예상은 주황색
- **동적 UI**: 제어 패널 크기 조정 가능
- **상세 모니터링**: 현재 설정값 및 에이전트 상태 실시간 표시
- **웹 접근성**: 로컬 네트워크에서 다른 기기로 접근 가능

## 🔧 설치 및 설정

### 시스템 요구사항
- Ubuntu 20.04+ 또는 Ubuntu 22.04+
- ROS2 Humble
- Python 3.8+
- 웹 브라우저 (웹 버전 사용 시)

### 의존성 설치

```bash
# ROS2 Humble 설치 (Ubuntu 22.04)
sudo apt update
sudo apt install ros-humble-desktop python3-argcomplete python3-colcon-common-extensions

# Python 의존성
sudo apt install python3-pip python3-numpy python3-tk

# 웹 버전 의존성
cd mapf_sim_web_ws
pip3 install -r web_server/requirements.txt
pip3 install -r web_visualizer/requirements.txt
```

### 빌드

```bash
# 웹 버전 빌드
cd mapf_sim_web_ws
colcon build --packages-select mapf_simulator

# RViz 버전 빌드
cd ../mapf_sim_rviz_ws
colcon build --packages-select mapf_simulator
```

## 📖 상세 문서

각 버전의 상세한 사용법과 기능에 대해서는 다음 문서를 참조하세요:

- **[웹 버전 상세 문서](mapf_sim_web_ws/README.md)**: 웹 기반 시뮬레이터의 모든 기능과 사용법
- **[RViz 버전 상세 문서](mapf_sim_rviz_ws/README.md)**: RViz 기반 시뮬레이터의 모든 기능과 사용법

## 🔄 버전 선택 가이드

### 웹 버전을 선택하세요:
- ✅ 웹 브라우저에서 접근하고 싶은 경우
- ✅ 고급 시각화와 실시간 모니터링이 필요한 경우
- ✅ 프리셋 설정으로 빠른 설정을 원하는 경우
- ✅ 다른 기기에서도 접근 가능하도록 하려는 경우
- ✅ 현대적인 UI/UX를 선호하는 경우

### RViz 버전을 선택하세요:
- ✅ ROS2 표준 도구를 선호하는 경우
- ✅ RViz의 강력한 시각화 기능이 필요한 경우
- ✅ ROS2 생태계와 완벽한 호환성을 원하는 경우
- ✅ 안정성과 검증된 도구를 우선시하는 경우

## 🚨 문제 해결

### 일반적인 문제들

1. **빌드 오류**
   ```bash
   # 의존성 재설치
   sudo apt update
   sudo apt install python3-numpy python3-tk
   
   # 워크스페이스 정리 후 재빌드
   rm -rf build install log
   colcon build --packages-select mapf_simulator
   ```

2. **웹 서버 연결 오류**
   ```bash
   # 포트 사용 확인
   sudo netstat -tulpn | grep :5000
   
   # 프로세스 정리
   pkill -f "python3.*app.py"
   ```

3. **RViz 마커가 보이지 않는 경우**
   ```bash
   # ROS2 환경 확인
   source install/setup.bash
   
   # 토픽 발행 확인
   ros2 topic list
   ros2 topic echo /agent_poses --once
   ```

### 디버깅 명령어

```bash
# ROS2 노드 상태 확인
ros2 node list

# 토픽 발행 확인
ros2 topic list
ros2 topic echo /agent_poses --once

# 웹 서버 상태 확인 (웹 버전)
curl http://localhost:5000/api/status
```

## 🤝 기여하기

이 프로젝트에 기여하고 싶으시다면:

1. 이슈를 생성하여 버그나 개선 사항을 제안해주세요
2. 새로운 기능이나 버그 수정을 위한 Pull Request를 보내주세요
3. 문서 개선이나 번역에 도움을 주세요

## 📄 라이선스

이 프로젝트는 MIT 라이선스 하에 배포됩니다.

## 🔗 관련 링크

- [ROS2 공식 문서](https://docs.ros.org/en/humble/)
- [RViz 사용법](https://github.com/ros2/rviz)
- [Flask 공식 문서](https://flask.palletsprojects.com/)
- [Socket.IO 문서](https://socket.io/docs/)

## 📊 프로젝트 통계

- **개발 기간**: 2024년
- **지원 ROS2 버전**: Humble
- **지원 Python 버전**: 3.8+
- **지원 운영체제**: Ubuntu 20.04+, Ubuntu 22.04+
- **라이선스**: MIT

---

**🎉 MAPF 시뮬레이터를 사용해주셔서 감사합니다!**

문제가 발생하거나 개선 사항이 있으시면 언제든지 이슈를 생성해주세요.
