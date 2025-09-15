#!/usr/bin/env python3

from flask import Flask, render_template, jsonify, request
from flask_socketio import SocketIO, emit
import json
import threading
import time
import requests
import argparse
import logging
from datetime import datetime

# 명령행 인수 파싱
parser = argparse.ArgumentParser(description='MAPF Web Simulator Server')
parser.add_argument('--verbose', '-v', action='store_true', 
                   help='Enable verbose HTTP request logging')
parser.add_argument('--port', '-p', type=int, default=5000,
                   help='Port to run the server on (default: 5000)')
args = parser.parse_args()

app = Flask(__name__)
app.config['SECRET_KEY'] = 'mapf_simulator_secret_key'

# 로깅 설정
if not args.verbose:
    # HTTP 요청 로그 비활성화
    log = logging.getLogger('werkzeug')
    log.setLevel(logging.ERROR)
    
    # Flask 앱 로그 레벨 설정
    app.logger.setLevel(logging.ERROR)

socketio = SocketIO(app, cors_allowed_origins="*", async_mode='eventlet')

# 시뮬레이션 상태 저장
simulation_state = {
    'is_running': False,
    'agents': [],
    'start_goal_poses': [],
    'grid_info': {},
    'collision_zones': [],
    'collision_info': [],  # 충돌 정보 추가
    'last_update': None
}

# 연결된 클라이언트 수
connected_clients = 0

def send_ros2_command(command, value):
    """ROS2 시뮬레이터에 명령 전달"""
    try:
        # 웹 시각화 노드에 직접 HTTP 요청으로 명령 전달
        web_viz_url = "http://localhost:5001"
        
        if command == 'start':
            response = requests.post(f"{web_viz_url}/api/start_simulation", 
                                   json={'start': True}, timeout=5)
            if response.status_code != 200:
                print(f"[ERROR] 시뮬레이션 시작 요청 실패: {response.status_code}")
        elif command == 'stop':
            response = requests.post(f"{web_viz_url}/api/start_simulation", 
                                   json={'start': False}, timeout=5)
            if response.status_code != 200:
                print(f"[ERROR] 시뮬레이션 중지 요청 실패: {response.status_code}")
            
    except requests.exceptions.Timeout:
        print(f"[ERROR] ROS2 명령 전달 타임아웃: {command}")
    except requests.exceptions.ConnectionError:
        print(f"[ERROR] ROS2 명령 전달 연결 실패: {command}")
    except Exception as e:
        print(f"[ERROR] ROS2 명령 전달 실패: {e}")

def send_ros2_reset_command():
    """ROS2 시뮬레이터에 리셋 명령 전달"""
    try:
        # 웹 시각화 노드에 직접 HTTP 요청으로 리셋 명령 전달
        web_viz_url = "http://localhost:5001"
        
        response = requests.post(f"{web_viz_url}/api/reset_simulation", 
                               json={'reset': True}, timeout=2)
        print(f"시뮬레이션 리셋 요청: {response.status_code}")
            
    except Exception as e:
        print(f"ROS2 리셋 명령 전달 실패: {e}")

@app.route('/')
def index():
    """메인 시뮬레이터 페이지"""
    return render_template('simulator.html')

@app.route('/api/status')
def get_status():
    """시뮬레이션 상태 API"""
    return jsonify(simulation_state)

@app.route('/api/start', methods=['POST'])
def start_simulation():
    """시뮬레이션 시작"""
    global simulation_state
    simulation_state['is_running'] = True
    simulation_state['last_update'] = datetime.now().isoformat()
    
    # WebSocket으로 모든 클라이언트에 시작 알림
    socketio.emit('simulation_started', {'status': 'started'})
    
    # ROS2 시뮬레이터에 시작 명령 전달 (별도 스레드에서 실행)
    threading.Thread(target=send_ros2_command, args=('start', True), daemon=True).start()
    
    return jsonify({'status': 'success', 'message': '시뮬레이션이 시작되었습니다.'})

@app.route('/api/stop', methods=['POST'])
def stop_simulation():
    """시뮬레이션 중지"""
    global simulation_state
    simulation_state['is_running'] = False
    simulation_state['last_update'] = datetime.now().isoformat()
    
    # WebSocket으로 모든 클라이언트에 중지 알림
    socketio.emit('simulation_stopped', {'status': 'stopped'})
    
    # ROS2 시뮬레이터에 중지 명령 전달 (별도 스레드에서 실행)
    threading.Thread(target=send_ros2_command, args=('stop', False), daemon=True).start()
    
    return jsonify({'status': 'success', 'message': '시뮬레이션이 중지되었습니다.'})

@app.route('/api/reset', methods=['POST'])
def reset_simulation():
    """시뮬레이션 리셋"""
    global simulation_state
    
    # 먼저 시뮬레이션을 중지
    simulation_state['is_running'] = False
    simulation_state['last_update'] = datetime.now().isoformat()
    
    # WebSocket으로 모든 클라이언트에 중지 알림
    socketio.emit('simulation_stopped', {'status': 'stopped'})
    
    # ROS2 시뮬레이터에 중지 명령 전달
    threading.Thread(target=send_ros2_command, args=('stop', False), daemon=True).start()
    
    # 잠시 대기 후 리셋
    time.sleep(0.5)
    
    # 상태 초기화
    simulation_state['agents'] = []
    simulation_state['start_goal_poses'] = []
    simulation_state['grid_info'] = {}
    simulation_state['collision_zones'] = []
    simulation_state['last_update'] = datetime.now().isoformat()
    
    # WebSocket으로 모든 클라이언트에 리셋 알림
    socketio.emit('simulation_reset', {'status': 'reset'})
    
    # ROS2 시뮬레이터에 리셋 명령 전달 (별도 스레드에서 실행)
    threading.Thread(target=send_ros2_reset_command, daemon=True).start()
    
    return jsonify({'status': 'success', 'message': '시뮬레이션이 리셋되었습니다.'})

@app.route('/api/update_agents', methods=['POST'])
def update_agents():
    """에이전트 정보 업데이트"""
    data = request.get_json()
    if data and 'agents' in data:
        simulation_state['agents'] = data['agents']
        simulation_state['last_update'] = datetime.now().isoformat()
        
        # WebSocket으로 실시간 업데이트 전송
        socketio.emit('agents_updated', {'agents': data['agents']})
        
        return jsonify({'status': 'success', 'message': '에이전트 정보가 업데이트되었습니다.'})
    
    return jsonify({'status': 'error', 'message': '잘못된 데이터입니다.'}), 400

@app.route('/api/update_grid', methods=['POST'])
def update_grid():
    """격자 정보 업데이트"""
    data = request.get_json()
    if data and 'grid_info' in data:
        simulation_state['grid_info'] = data['grid_info']
        simulation_state['last_update'] = datetime.now().isoformat()
        
        # WebSocket으로 실시간 업데이트 전송
        socketio.emit('grid_updated', {'grid_info': data['grid_info']})
        
        return jsonify({'status': 'success', 'message': '격자 정보가 업데이트되었습니다.'})
    
    return jsonify({'status': 'error', 'message': '잘못된 데이터입니다.'}), 400

@app.route('/api/update_start_goal', methods=['POST'])
def update_start_goal():
    """시작/목표 위치 정보 업데이트"""
    data = request.get_json()
    if data and 'start_goal_poses' in data:
        simulation_state['start_goal_poses'] = data['start_goal_poses']
        simulation_state['last_update'] = datetime.now().isoformat()
        
        # WebSocket으로 실시간 업데이트 전송
        socketio.emit('start_goal_updated', {'start_goal_poses': data['start_goal_poses']})
        
        return jsonify({'status': 'success', 'message': '시작/목표 위치 정보가 업데이트되었습니다.'})
    
    return jsonify({'status': 'error', 'message': '잘못된 데이터입니다.'}), 400

@app.route('/api/update_collision_info', methods=['POST'])
def update_collision_info():
    """충돌 정보 업데이트"""
    data = request.get_json()
    if data:
        collision_info = data.get('collision_info', [])
        prev_collision_info = simulation_state.get('collision_info', [])
        
        # 충돌 정보가 변경되었을 때만 로그 출력
        if collision_info != prev_collision_info:
            if collision_info:
                print(f"[DEBUG] 충돌 정보 업데이트: {len(collision_info)}개 충돌")
            else:
                print(f"[DEBUG] 충돌 정보 해제: 모든 충돌 해결")
        
        simulation_state['collision_info'] = collision_info
        simulation_state['last_update'] = datetime.now().isoformat()
        
        # WebSocket으로 모든 클라이언트에 충돌 정보 전송
        socketio.emit('collision_info_updated', {
            'collision_info': simulation_state['collision_info']
        })
        
        return jsonify({'status': 'success', 'message': '충돌 정보가 업데이트되었습니다.'})
    
    return jsonify({'status': 'error', 'message': '잘못된 데이터입니다.'}), 400

@app.route('/api/update_basic_settings', methods=['POST'])
def update_basic_settings():
    """기본 설정 업데이트"""
    data = request.get_json()
    if data:
        print(f'기본 설정 업데이트 수신: {data}')
        # 설정을 시뮬레이션 상태에 저장
        if 'basic_settings' not in simulation_state:
            simulation_state['basic_settings'] = {}
        simulation_state['basic_settings'].update(data)
        simulation_state['last_update'] = datetime.now().isoformat()
        
        return jsonify({'status': 'success', 'message': '기본 설정이 업데이트되었습니다.'})
    
    return jsonify({'status': 'error', 'message': '잘못된 데이터입니다.'}), 400

@app.route('/api/toggle_collision_zones', methods=['POST'])
def toggle_collision_zones():
    """충돌 영역 시각화 토글"""
    data = request.get_json()
    if data and 'show_collision_zones' in data:
        show_zones = data['show_collision_zones']
        
        # 웹 시각화 노드에 명령 전달
        try:
            web_viz_url = "http://localhost:5001"
            response = requests.post(f"{web_viz_url}/api/toggle_collision_zones", 
                                   json={'show_collision_zones': show_zones}, timeout=5)
            if response.status_code == 200:
                print(f'충돌 영역 시각화 {"활성화" if show_zones else "비활성화"}됨')
                return jsonify({'status': 'success', 'message': f'충돌 영역 시각화가 {"활성화" if show_zones else "비활성화"}되었습니다.'})
            else:
                return jsonify({'status': 'error', 'message': '충돌 영역 시각화 토글 실패'}), 500
        except Exception as e:
            print(f'충돌 영역 시각화 토글 실패: {e}')
            return jsonify({'status': 'error', 'message': '충돌 영역 시각화 토글 실패'}), 500
    
    return jsonify({'status': 'error', 'message': '잘못된 데이터입니다.'}), 400

@app.route('/api/update_collision_zones', methods=['POST'])
def update_collision_zones():
    """충돌 감지 영역 정보 업데이트"""
    data = request.get_json()
    if data and 'collision_zones' in data:
        simulation_state['collision_zones'] = data['collision_zones']
        simulation_state['last_update'] = datetime.now().isoformat()
        
        # WebSocket으로 실시간 업데이트 전송
        socketio.emit('collision_zones_updated', {'collision_zones': data['collision_zones']})
        
        return jsonify({'status': 'success', 'message': '충돌 감지 영역 정보가 업데이트되었습니다.'})
    
    return jsonify({'status': 'error', 'message': '잘못된 데이터입니다.'}), 400

@app.route('/api/update_collision_settings', methods=['POST'])
def update_collision_settings():
    """상세 충돌 설정 업데이트"""
    print('상세 충돌 설정 업데이트 요청 수신')
    data = request.get_json()
    if data:
        print(f'충돌 설정 데이터 수신: {data}')
        
        # 웹 시각화 노드에 명령 전달
        try:
            web_viz_url = "http://localhost:5001"
            response = requests.post(f"{web_viz_url}/api/update_collision_settings", 
                                   json=data, timeout=5)
            if response.status_code == 200:
                print('상세 충돌 설정 업데이트 성공')
                return jsonify({'status': 'success', 'message': '상세 충돌 설정이 업데이트되었습니다.'})
            else:
                print(f'상세 충돌 설정 업데이트 실패: {response.status_code}')
                return jsonify({'status': 'error', 'message': '상세 충돌 설정 업데이트 실패'}), 500
        except Exception as e:
            print(f'상세 충돌 설정 업데이트 실패: {e}')
            return jsonify({'status': 'error', 'message': '상세 충돌 설정 업데이트 실패'}), 500
    
    return jsonify({'status': 'error', 'message': '잘못된 데이터입니다.'}), 400


# WebSocket 이벤트 핸들러
@socketio.on('connect')
def handle_connect():
    """클라이언트 연결"""
    global connected_clients
    connected_clients += 1
    print(f'클라이언트가 연결되었습니다. 총 연결 수: {connected_clients}')
    
    # 현재 시뮬레이션 상태 전송
    emit('simulation_state', simulation_state)

@socketio.on('disconnect')
def handle_disconnect():
    """클라이언트 연결 해제"""
    global connected_clients
    connected_clients -= 1
    print(f'클라이언트 연결이 해제되었습니다. 총 연결 수: {connected_clients}')

@socketio.on('request_update')
def handle_update_request():
    """업데이트 요청 처리"""
    emit('simulation_state', simulation_state)

@socketio.on('control_command')
def handle_control_command(data):
    """제어 명령 처리"""
    command = data.get('command')
    params = data.get('params', {})
    
    # 명령을 웹 시각화 노드로 전달
    try:
        web_viz_url = "http://localhost:5001"
        
        if command == 'update_basic_settings':
            response = requests.post(f"{web_viz_url}/api/update_basic_settings", 
                                   json=params, timeout=5)
            if response.status_code == 200:
                print(f'설정 적용됨: 에이전트 {params.get("num_agents", "N/A")}개')
            else:
                print(f'설정 적용 실패: {response.status_code}')
        elif command == 'update_agent_positions':
            response = requests.post(f"{web_viz_url}/api/update_agent_positions", 
                                   json=params, timeout=5)
            if response.status_code == 200:
                print('에이전트 위치 업데이트됨')
            else:
                print(f'위치 업데이트 실패: {response.status_code}')
        elif command == 'update_collision_settings':
            response = requests.post(f"{web_viz_url}/api/update_collision_settings", 
                                   json=params, timeout=5)
            if response.status_code == 200:
                print('충돌 설정 업데이트됨')
            else:
                print(f'충돌 설정 실패: {response.status_code}')
        elif command == 'update_world_settings':
            response = requests.post(f"{web_viz_url}/api/update_world_settings", 
                                   json=params, timeout=5)
            if response.status_code == 200:
                print('월드 설정 업데이트됨')
            else:
                print(f'월드 설정 실패: {response.status_code}')
        
    except requests.exceptions.RequestException as e:
        print(f'통신 실패: {e}')
        print('웹 시각화 노드가 실행 중인지 확인하세요.')
    
    # 응답 전송
    emit('command_response', {
        'status': 'received',
        'command': command,
        'params': params,
        'timestamp': datetime.now().isoformat()
    })

def start_web_server():
    """웹 서버 시작"""
    print("웹 서버를 시작합니다...")
    print(f"브라우저에서 http://localhost:{args.port} 으로 접속하세요.")
    
    if args.verbose:
        print("상세 로깅 모드가 활성화되었습니다.")
    else:
        print("HTTP 요청 로그가 비활성화되었습니다. (--verbose 옵션으로 활성화 가능)")
    
    socketio.run(app, host='0.0.0.0', port=args.port, debug=False)

if __name__ == '__main__':
    start_web_server()
