# MAPF 시뮬레이터 - RViz 버전 워크스페이스

ROS2 기반의 다중 에이전트 경로 계획 시뮬레이터 워크스페이스입니다. RViz를 사용한 전통적인 시각화 방식으로 MAPF 시뮬레이션을 제공합니다.

> **참고**: 이 프로젝트는 **레거시 버전**입니다. 새로운 웹 기반 버전은 [mapf_sim_web_ws](../mapf_sim_web_ws/README.md)를 참조하세요.

## 🚀 빠른 시작

### 1. 워크스페이스 빌드
```bash
cd mapf_sim_rviz_ws
colcon build --packages-select mapf_simulator
source install/setup.bash
```

### 2. 시뮬레이터 실행
```bash
./run_simulator.sh
```

### 3. 사용법
- GUI 창에서 파라미터 조정
- "시뮬레이션 시작" 버튼으로 시뮬레이션 시작
- RViz에서 실시간 시각화 확인

## 📁 워크스페이스 구조

```
mapf_sim_rviz_ws/
├── mapf_simulator/              # MAPF 시뮬레이터 패키지
│   ├── mapf_simulator/          # Python 모듈
│   ├── launch/                  # Launch 파일
│   ├── config/                  # RViz 설정 파일
│   └── README.md               # 패키지 상세 문서
├── run_simulator.sh            # 실행 스크립트
└── README.md                   # 이 파일
```

## 🔧 주요 기능

- **다중 에이전트 시뮬레이션**: 여러 에이전트가 동시에 목표 지점으로 이동
- **RViz 시각화**: ROS2의 표준 시각화 도구 사용
- **GUI 제어**: Tkinter 기반 파라미터 조정 인터페이스
- **그리드 이동**: 1-칸씩 정확한 격자 기반 이동
- **충돌 회피**: 자동 충돌 감지 및 회피 알고리즘

## 📖 상세 문서

자세한 사용법과 기능에 대해서는 [mapf_simulator README](mapf_simulator/README.md)를 참조하세요.

## 🔄 웹 버전과의 비교

| 특징 | RViz 버전 | 웹 버전 |
|------|-----------|---------|
| 시각화 | RViz | HTML5 Canvas |
| 제어 | Tkinter GUI | 웹 인터페이스 |
| 접근성 | 로컬 전용 | 웹 브라우저 |
| 설정 | 기본 | 고급 프리셋 |
| 모니터링 | 기본 | 실시간 상세 정보 |

## 🚨 문제 해결

### 빌드 오류
```bash
# 의존성 설치
sudo apt update
sudo apt install python3-numpy python3-tk

# 워크스페이스 정리 후 재빌드
rm -rf build install log
colcon build --packages-select mapf_simulator
```

### 실행 오류
```bash
# ROS2 환경 확인
echo $ROS_DISTRO
source install/setup.bash

# 노드 상태 확인
ros2 node list
```

## 📄 라이선스

이 프로젝트는 MIT 라이선스 하에 배포됩니다.

## 🔗 관련 링크

- [웹 버전 시뮬레이터](../mapf_sim_web_ws/README.md)
- [패키지 상세 문서](mapf_simulator/README.md)
- [ROS2 공식 문서](https://docs.ros.org/en/humble/)
- [RViz 사용법](https://github.com/ros2/rviz)
