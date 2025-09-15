#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, Float32, String
from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray
import json
import requests
import threading
import time
import argparse
import logging
from datetime import datetime
from flask import Flask, request, jsonify

# 명령행 인수 파싱
parser = argparse.ArgumentParser(description='MAPF Web Visualizer Node')
parser.add_argument('--verbose', '-v', action='store_true', 
                   help='Enable verbose HTTP request logging')
parser.add_argument('--port', '-p', type=int, default=5001,
                   help='Port to run the Flask server on (default: 5001)')
args = parser.parse_args()

class WebVisualizerNode(Node):
    def __init__(self):
        super().__init__('web_visualizer_node')
        
        # 웹 서버 설정
        self.web_server_url = "http://localhost:5000"
        self.update_interval = 0.1  # 100ms마다 업데이트
        
        # 시뮬레이션 상태
        self.simulation_running = False
        self.agents = []
        self.grid_info = {}
        self.collision_zones = []
        
        # 구독자 설정
        self.agent_poses_sub = self.create_subscription(
            MarkerArray, '/agent_poses', self.on_agent_poses, 10)
        self.collision_info_sub = self.create_subscription(
            String, '/collision_info', self.on_collision_info, 10)
        self.start_goal_poses_sub = self.create_subscription(
            MarkerArray, '/start_goal_poses', self.on_start_goal_poses, 10)
        self.grid_viz_sub = self.create_subscription(
            MarkerArray, '/grid_visualization', self.on_grid_visualization, 10)
        self.collision_detection_zones_sub = self.create_subscription(
            MarkerArray, '/collision_detection_zones', self.on_collision_detection_zones, 10)
        # start_simulation 구독 제거 (무한 루프 방지)
        # self.start_sim_sub = self.create_subscription(
        #     Bool, '/start_simulation', self.on_start_simulation, 10)
        
        # 퍼블리셔 설정 (웹 서버에서 받은 명령을 ROS2 토픽으로 발행)
        self.start_simulation_pub = self.create_publisher(Bool, '/start_simulation', 10)
        self.reset_simulation_pub = self.create_publisher(Bool, '/reset_simulation', 10)
        
        # 설정 업데이트를 위한 퍼블리셔들
        self.basic_settings_pub = self.create_publisher(String, '/basic_settings', 10)
        self.agent_positions_pub = self.create_publisher(String, '/agent_positions', 10)
        self.collision_settings_pub = self.create_publisher(String, '/collision_settings', 10)
        self.world_settings_pub = self.create_publisher(String, '/world_settings', 10)
        self.movement_mode_pub = self.create_publisher(Int32, '/movement_mode', 10)
        self.collision_info_pub = self.create_publisher(String, '/collision_info', 10)
        
        # 웹 서버 연결 확인
        self.check_web_server_connection()
        
        # 주기적 업데이트 타이머
        self.update_timer = self.create_timer(self.update_interval, self.update_web_interface)
        
        # 웹 서버 상태 확인 타이머 완전 제거 (무한 루프 방지)
        
        # Flask 서버 시작 (웹 서버의 명령을 받기 위해)
        self.start_flask_server()
        
        self.get_logger().info('웹 시각화 노드가 시작되었습니다.')
        self.get_logger().info(f'웹 서버 URL: {self.web_server_url}')
    
    def check_web_server_connection(self):
        """웹 서버 연결 확인"""
        try:
            response = requests.get(f"{self.web_server_url}/api/status", timeout=5)
            if response.status_code == 200:
                self.get_logger().info('웹 서버에 성공적으로 연결되었습니다.')
            else:
                self.get_logger().warn(f'웹 서버 응답 오류: {response.status_code}')
        except requests.exceptions.RequestException as e:
            self.get_logger().warn(f'웹 서버 연결 실패: {e}')
            self.get_logger().info('웹 서버를 먼저 시작해주세요: python web_server/app.py')
    
    def on_agent_poses(self, msg):
        """에이전트 위치 정보 처리"""
        agents_data = []
        
        for marker in msg.markers:
            agent_data = {
                'id': marker.id,
                'position': {
                    'x': marker.pose.position.x,
                    'y': marker.pose.position.y,
                    'z': marker.pose.position.z
                },
                'color': {
                    'r': marker.color.r,
                    'g': marker.color.g,
                    'b': marker.color.b,
                    'a': marker.color.a
                },
                'scale': {
                    'x': marker.scale.x,
                    'y': marker.scale.y,
                    'z': marker.scale.z
                },
                'is_moving': marker.color.a > 0.5,  # 투명도로 이동 상태 판단
                'is_colliding': False,  # 충돌 상태는 별도로 처리
                'is_collision_predicted': False  # 충돌 예상 상태는 별도로 처리
            }
            agents_data.append(agent_data)
        
        self.agents = agents_data
    
    def on_collision_info(self, msg: String):
        """충돌 정보 콜백"""
        try:
            collision_info = json.loads(msg.data)
            
            # 웹 서버에 충돌 정보 전달
            try:
                response = requests.post(
                    f"{self.web_server_url}/api/update_collision_info",
                    json={'collision_info': collision_info},
                    timeout=1
                )
                if response.status_code != 200:
                    self.get_logger().warn(f'충돌 정보 업데이트 실패: {response.status_code}')
            except requests.exceptions.RequestException:
                # 웹 서버 연결 실패 시 로그만 출력
                pass
                    
        except json.JSONDecodeError as e:
            self.get_logger().warn(f'충돌 정보 파싱 실패: {e}')

    def on_start_goal_poses(self, msg):
        """시작위치와 목표위치 정보 처리"""
        start_goal_data = []
        
        for marker in msg.markers:
            pose_data = {
                'id': marker.id,
                'type': 'start' if 'start' in marker.ns else 'goal',
                'position': {
                    'x': marker.pose.position.x,
                    'y': marker.pose.position.y,
                    'z': marker.pose.position.z
                },
                'color': {
                    'r': marker.color.r,
                    'g': marker.color.g,
                    'b': marker.color.b,
                    'a': marker.color.a
                },
                'scale': {
                    'x': marker.scale.x,
                    'y': marker.scale.y,
                    'z': marker.scale.z
                }
            }
            start_goal_data.append(pose_data)
        
        # 시작위치와 목표위치 정보를 에이전트 정보에 추가
        if hasattr(self, 'start_goal_poses'):
            self.start_goal_poses = start_goal_data
        else:
            self.start_goal_poses = start_goal_data
    
    def on_grid_visualization(self, msg):
        """격자 시각화 정보 처리"""
        grid_lines = []
        
        for marker in msg.markers:
            if marker.type == marker.LINE_STRIP and len(marker.points) >= 2:
                line_data = {
                    'start': {
                        'x': marker.points[0].x,
                        'y': marker.points[0].y,
                        'z': marker.points[0].z
                    },
                    'end': {
                        'x': marker.points[1].x,
                        'y': marker.points[1].y,
                        'z': marker.points[1].z
                    },
                    'color': {
                        'r': marker.color.r,
                        'g': marker.color.g,
                        'b': marker.color.b,
                        'a': marker.color.a
                    }
                }
                grid_lines.append(line_data)
        
        self.grid_info = {
            'lines': grid_lines,
            'cell_size': 1.0,
            'world_bounds': {
                'min_x': -10.0,
                'max_x': 10.0,
                'min_y': -10.0,
                'max_y': 10.0
            }
        }
    
    def on_collision_detection_zones(self, msg):
        """충돌 감지 영역 정보 처리"""
        collision_zones = []
        
        for marker in msg.markers:
            zone_data = {
                'id': marker.id,
                'type': marker.ns,  # 'collision_detection_zones' 또는 'agent_radius_zones'
                'position': {
                    'x': marker.pose.position.x,
                    'y': marker.pose.position.y,
                    'z': marker.pose.position.z
                },
                'scale': {
                    'x': marker.scale.x,
                    'y': marker.scale.y,
                    'z': marker.scale.z
                },
                'color': {
                    'r': marker.color.r,
                    'g': marker.color.g,
                    'b': marker.color.b,
                    'a': marker.color.a
                }
            }
            collision_zones.append(zone_data)
        
        # 웹 서버에 충돌 감지 영역 정보 전달
        try:
            response = requests.post(
                f"{self.web_server_url}/api/update_collision_zones",
                json={'collision_zones': collision_zones},
                timeout=1
            )
            if response.status_code != 200:
                self.get_logger().warn(f'충돌 감지 영역 업데이트 실패: {response.status_code}')
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f'웹 서버 연결 실패: {e}')
        except Exception as e:
            self.get_logger().error(f'충돌 영역 데이터 전송 중 오류: {e}')
    
    # on_start_simulation 관련 함수들 제거 (무한 루프 방지)
    
    def update_web_interface(self):
        """웹 인터페이스 업데이트"""
        # 시뮬레이션 상태와 관계없이 항상 데이터 전송
        
        # 에이전트 정보 업데이트
        if self.agents:
            try:
                response = requests.post(
                    f"{self.web_server_url}/api/update_agents",
                    json={'agents': self.agents},
                    timeout=1
                )
                if response.status_code != 200:
                    self.get_logger().warn(f'에이전트 업데이트 실패: {response.status_code}')
            except requests.exceptions.RequestException as e:
                # 웹 서버 연결 실패 시 로그만 출력 (너무 자주 출력하지 않음)
                pass
        
        # 시작/목표 위치 정보 업데이트
        if hasattr(self, 'start_goal_poses') and self.start_goal_poses:
            try:
                response = requests.post(
                    f"{self.web_server_url}/api/update_start_goal",
                    json={'start_goal_poses': self.start_goal_poses},
                    timeout=1
                )
                if response.status_code != 200:
                    self.get_logger().warn(f'시작/목표 위치 업데이트 실패: {response.status_code}')
            except requests.exceptions.RequestException as e:
                # 웹 서버 연결 실패 시 로그만 출력
                pass
        
        # 격자 정보 업데이트
        if self.grid_info:
            try:
                response = requests.post(
                    f"{self.web_server_url}/api/update_grid",
                    json={'grid_info': self.grid_info},
                    timeout=1
                )
                if response.status_code != 200:
                    self.get_logger().warn(f'격자 업데이트 실패: {response.status_code}')
            except requests.exceptions.RequestException as e:
                # 웹 서버 연결 실패 시 로그만 출력
                pass
        
        # 충돌 정보는 별도 토픽에서 처리하므로 여기서는 제거
    
    
    def start_flask_server(self):
        """Flask 서버 시작 (웹 서버의 명령을 받기 위해)"""
        app = Flask(__name__)
        
        # 로깅 설정
        if not args.verbose:
            # HTTP 요청 로그 비활성화
            log = logging.getLogger('werkzeug')
            log.setLevel(logging.ERROR)
            
            # Flask 앱 로그 레벨 설정
            app.logger.setLevel(logging.ERROR)
        
        @app.route('/api/start_simulation', methods=['POST'])
        def handle_start_simulation():
            """시뮬레이션 시작/중지 명령 처리"""
            try:
                data = request.get_json()
                start = data.get('start', False)
                
                # ROS2 퍼블리셔가 유효한지 확인
                if self.start_simulation_pub is None:
                    self.get_logger().error('ROS2 퍼블리셔가 초기화되지 않았습니다.')
                    return jsonify({'status': 'error', 'message': 'ROS2 퍼블리셔가 초기화되지 않았습니다.'}), 500
                
                # ROS2 토픽에 메시지 발행
                status_msg = Bool()
                status_msg.data = start
                self.start_simulation_pub.publish(status_msg)
                
                return jsonify({'status': 'success'})
            except Exception as e:
                self.get_logger().error(f'시뮬레이션 명령 처리 실패: {e}')
                return jsonify({'status': 'error', 'message': str(e)}), 500
        
        @app.route('/api/reset_simulation', methods=['POST'])
        def handle_reset_simulation():
            """시뮬레이션 리셋 명령 처리"""
            try:
                # ROS2 토픽에 리셋 메시지 발행
                reset_msg = Bool()
                reset_msg.data = True
                self.reset_simulation_pub.publish(reset_msg)
                
                # 성공 시 로그 없음 (중복 방지)
                
                return jsonify({'status': 'success'})
            except Exception as e:
                self.get_logger().error(f'시뮬레이션 리셋 명령 처리 실패: {e}')
                return jsonify({'status': 'error', 'message': str(e)}), 500
        
        @app.route('/api/update_basic_settings', methods=['POST'])
        def handle_update_basic_settings():
            """기본 설정 업데이트 처리"""
            try:
                data = request.get_json()
                self.get_logger().info(f'기본 설정 업데이트: {data}')
                
                # ROS2 토픽으로 기본 설정 전달
                settings_msg = String()
                settings_msg.data = json.dumps(data)
                self.basic_settings_pub.publish(settings_msg)
                self.get_logger().info('ROS2 /basic_settings 토픽에 기본 설정을 발행했습니다.')
                
                # movement_mode가 있으면 별도로 발행
                if 'movement_mode' in data:
                    movement_mode_msg = Int32()
                    movement_mode_msg.data = int(data['movement_mode'])
                    self.movement_mode_pub.publish(movement_mode_msg)
                    self.get_logger().info(f'ROS2 /movement_mode 토픽에 {data["movement_mode"]}를 발행했습니다.')
                
                # 웹 서버에 설정 업데이트 알림 (순환 요청 방지를 위해 제거)
                # 웹 서버에서 이미 이 요청을 받았으므로 다시 보낼 필요 없음
                
                return jsonify({'status': 'success'})
            except Exception as e:
                self.get_logger().error(f'기본 설정 업데이트 실패: {e}')
                return jsonify({'status': 'error', 'message': str(e)}), 500
        
        @app.route('/api/update_agent_positions', methods=['POST'])
        def handle_update_agent_positions():
            """에이전트 위치 업데이트 처리"""
            try:
                data = request.get_json()
                self.get_logger().info(f'에이전트 위치 업데이트: {data}')
                
                # ROS2 토픽으로 에이전트 위치 정보 전달
                positions_msg = String()
                positions_msg.data = json.dumps(data)
                self.agent_positions_pub.publish(positions_msg)
                self.get_logger().info('ROS2 /agent_positions 토픽에 에이전트 위치를 발행했습니다.')
                
                return jsonify({'status': 'success'})
            except Exception as e:
                self.get_logger().error(f'에이전트 위치 업데이트 실패: {e}')
                return jsonify({'status': 'error', 'message': str(e)}), 500
        
        @app.route('/api/update_collision_settings', methods=['POST'])
        def handle_update_collision_settings():
            """충돌 설정 업데이트 처리"""
            try:
                data = request.get_json()
                self.get_logger().info(f'충돌 설정 업데이트: {data}')
                
                # ROS2 토픽으로 충돌 설정 전달
                settings_msg = String()
                settings_msg.data = json.dumps(data)
                self.collision_settings_pub.publish(settings_msg)
                self.get_logger().info('ROS2 /collision_settings 토픽에 충돌 설정을 발행했습니다.')
                
                # 개별 설정들을 별도 토픽으로도 발행
                if 'collision_distance' in data:
                    from std_msgs.msg import Float32
                    collision_distance_msg = Float32()
                    collision_distance_msg.data = float(data['collision_distance'])
                    # collision_distance 퍼블리셔가 필요하지만 현재 없으므로 basic_settings로 전달
                    self.basic_settings_pub.publish(settings_msg)
                
                return jsonify({'status': 'success'})
            except Exception as e:
                self.get_logger().error(f'충돌 설정 업데이트 실패: {e}')
                return jsonify({'status': 'error', 'message': str(e)}), 500
        
        @app.route('/api/update_world_settings', methods=['POST'])
        def handle_update_world_settings():
            """월드 설정 업데이트 처리"""
            try:
                data = request.get_json()
                self.get_logger().info(f'월드 설정 업데이트: {data}')
                
                # TODO: ROS2 토픽으로 월드 설정 전달
                # 여기서는 로그만 출력
                
                return jsonify({'status': 'success'})
            except Exception as e:
                self.get_logger().error(f'월드 설정 업데이트 실패: {e}')
                return jsonify({'status': 'error', 'message': str(e)}), 500
        
        @app.route('/api/toggle_collision_zones', methods=['POST'])
        def handle_toggle_collision_zones():
            """충돌 영역 시각화 토글 처리"""
            try:
                data = request.get_json()
                show_zones = data.get('show_collision_zones', True)
                self.get_logger().info(f'충돌 영역 시각화 토글: {show_zones}')
                
                # ROS2 토픽으로 충돌 영역 시각화 설정 전달
                settings_msg = String()
                settings_msg.data = json.dumps({'show_collision_zones': show_zones})
                self.basic_settings_pub.publish(settings_msg)
                self.get_logger().info(f'ROS2 /basic_settings 토픽에 충돌 영역 시각화 설정을 발행했습니다: {show_zones}')
                
                return jsonify({'status': 'success'})
            except Exception as e:
                self.get_logger().error(f'충돌 영역 시각화 토글 실패: {e}')
                return jsonify({'status': 'error', 'message': str(e)}), 500
        
        # Flask 서버를 별도 스레드에서 실행
        def run_flask():
            app.run(host='0.0.0.0', port=args.port, debug=False, use_reloader=False)
        
        flask_thread = threading.Thread(target=run_flask, daemon=True)
        flask_thread.start()
        
        if args.verbose:
            self.get_logger().info(f'Flask 서버가 포트 {args.port}에서 시작되었습니다. (상세 로깅 활성화)')
        else:
            self.get_logger().info(f'Flask 서버가 포트 {args.port}에서 시작되었습니다. (HTTP 로그 비활성화)')

def main():
    # ROS2 인수와 Flask 인수를 분리
    import sys
    ros2_args = []
    flask_args = []
    
    # --verbose, --port 옵션을 Flask 인수로 분리
    i = 0
    while i < len(sys.argv):
        if sys.argv[i] in ['--verbose', '-v', '--port', '-p']:
            flask_args.extend(sys.argv[i:i+2] if sys.argv[i] in ['--port', '-p'] else [sys.argv[i]])
            i += 2 if sys.argv[i] in ['--port', '-p'] else 1
        else:
            ros2_args.append(sys.argv[i])
            i += 1
    
    # Flask 인수 파싱
    global args
    args = parser.parse_args(flask_args)
    
    # ROS2 초기화
    rclpy.init(args=ros2_args)
    
    web_viz_node = WebVisualizerNode()
    
    try:
        rclpy.spin(web_viz_node)
    except KeyboardInterrupt:
        web_viz_node.get_logger().info('웹 시각화 노드를 종료합니다.')
    finally:
        web_viz_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
