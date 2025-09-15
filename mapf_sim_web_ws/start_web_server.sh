#!/bin/bash

echo "MAPF 시뮬레이터 웹 서버를 시작합니다..."

# 워크스페이스 설정
cd /home/mnerd/mapf_sim_web_ws

# Python 가상환경 활성화 (선택사항)
# source venv/bin/activate

# 필요한 패키지 설치 확인
echo "필요한 Python 패키지를 설치합니다..."
pip3 install -r web_server/requirements.txt

# 웹 서버 시작
echo "웹 서버를 시작합니다..."
cd web_server
python3 app.py

echo "웹 서버가 시작되었습니다."
echo "브라우저에서 http://localhost:5000 으로 접속하세요."
