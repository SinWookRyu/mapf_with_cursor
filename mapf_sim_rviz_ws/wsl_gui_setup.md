# WSL GUI 설정 가이드

## 문제 상황
WSL 환경에서 RViz가 실행되지 않는 문제가 발생하고 있습니다.

## 해결 방안

### 1. Windows에서 X11 서버 설치
- **VcXsrv** 또는 **Xming** 설치
- Windows에서 X11 서버 실행

### 2. WSL 환경변수 설정
```bash
export DISPLAY=:0
export LIBGL_ALWAYS_INDIRECT=1
```

### 3. 대안 시각화 방법

#### A. 터미널 모니터링
```bash
# 에이전트 상태 실시간 확인
ros2 topic echo /agent_poses

# 시뮬레이션 리셋
ros2 topic pub /reset_simulation std_msgs/msg/Bool "data: true" --once
```

#### B. 웹 기반 시각화 (향후 개발)
- ROS2 웹 브리지 사용
- 웹 브라우저에서 시각화

### 4. 현재 상태
✅ **시뮬레이터**: 정상 작동  
✅ **에이전트 데이터**: 정상 발행  
❌ **RViz**: WSL GUI 환경 문제로 실행 불가

### 5. 즉시 사용 가능한 명령어
```bash
# 시뮬레이터 실행
./run_simulator_fixed.sh

# 에이전트 상태 확인
ros2 topic echo /agent_poses --once

# 시뮬레이션 리셋
ros2 topic pub /reset_simulation std_msgs/msg/Bool "data: true" --once
```
