# MAPF Web Simulator 로깅 옵션

## 개요
MAPF Web Simulator의 HTTP 요청 로그를 프로그램 옵션으로 제어할 수 있습니다. 기본적으로는 HTTP 요청 로그가 비활성화되어 깔끔한 출력을 제공합니다.

## 사용법

### 1. 웹 서버 (app.py)

#### 기본 실행 (HTTP 로그 비활성화)
```bash
python3 app.py
```

#### 상세 로깅 모드 (HTTP 로그 활성화)
```bash
python3 app.py --verbose
# 또는
python3 app.py -v
```

#### 포트 변경
```bash
python3 app.py --port 8080
# 또는
python3 app.py -p 8080
```

#### 상세 로깅 + 포트 변경
```bash
python3 app.py --verbose --port 8080
# 또는
python3 app.py -v -p 8080
```

### 2. 웹 시각화 노드 (web_viz_node.py)

#### 기본 실행 (HTTP 로그 비활성화)
```bash
python3 web_viz_node.py
```

#### 상세 로깅 모드 (HTTP 로그 활성화)
```bash
python3 web_viz_node.py --verbose
# 또는
python3 web_viz_node.py -v
```

#### 포트 변경
```bash
python3 web_viz_node.py --port 5002
# 또는
python3 web_viz_node.py -p 5002
```

#### 상세 로깅 + 포트 변경
```bash
python3 web_viz_node.py --verbose --port 5002
# 또는
python3 web_viz_node.py -v -p 5002
```

## 로깅 레벨

### 기본 모드 (--verbose 없이)
- HTTP 요청 로그 비활성화
- 중요한 시스템 메시지만 출력
- 깔끔한 콘솔 출력

### 상세 모드 (--verbose 옵션)
- 모든 HTTP 요청 로그 출력
- Flask 디버그 정보 출력
- 개발 및 디버깅에 유용

## 출력 예시

### 기본 모드 출력
```
웹 서버를 시작합니다...
브라우저에서 http://localhost:5000 으로 접속하세요.
HTTP 요청 로그가 비활성화되었습니다. (--verbose 옵션으로 활성화 가능)
```

### 상세 모드 출력
```
웹 서버를 시작합니다...
브라우저에서 http://localhost:5000 으로 접속하세요.
상세 로깅 모드가 활성화되었습니다.
 * Running on all addresses (0.0.0.0)
 * Running on http://127.0.0.1:5000
 * Running on http://[::1]:5000
127.0.0.1 - - [05/Sep/2025 16:06:43] "POST /api/update_grid HTTP/1.1" 200 263 0.000597
127.0.0.1 - - [05/Sep/2025 16:06:43] "POST /api/update_agents HTTP/1.1" 200 275 0.000629
```

## 도움말

각 프로그램의 도움말을 보려면:
```bash
python3 app.py --help
python3 web_viz_node.py --help
```

## 권장사항

- **일반 사용**: 기본 모드 사용 (HTTP 로그 비활성화)
- **개발/디버깅**: `--verbose` 옵션 사용
- **프로덕션**: 기본 모드 사용으로 성능 최적화
