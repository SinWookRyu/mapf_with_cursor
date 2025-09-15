#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA, Bool, Float32, Int32, String
import numpy as np
from typing import Optional
import time
import json

class SimpleAgent:
    def __init__(self, agent_id: int, start_pos: tuple, goal_pos: tuple):
        self.id = agent_id
        self.start_pos = np.array(start_pos, dtype=float)  # 시작 위치 저장
        self.current_pos = np.array(start_pos, dtype=float)
        self.goal_pos = np.array(goal_pos, dtype=float)
        self.velocity = np.array([0.0, 0.0])
        self.max_speed = 2.0
        self.radius = 0.5
        self.is_moving = True
        self.goal_reached_logged = False  # 목표 도달 로그 중복 방지
        self.is_colliding = False  # 충돌 상태 추적
        self.is_collision_predicted = False  # 충돌 예상 상태 추적
        self.prev_collision_state = False  # 이전 충돌 상태 추적
        self.prev_prediction_state = False  # 이전 충돌 예상 상태 추적
        self.wait_timer = 0.0  # 대기 타이머
        self.is_waiting = False  # 대기 상태
        
        # 이동 방향 관리
        self.initial_direction = None  # 초기 목표지 방향 (격자 단위)
        self.current_direction = None  # 현재 이동 방향 (격자 단위)
        self.direction_initialized = False  # 방향 초기화 여부
        self.last_collision_position = None  # 마지막 충돌 위치
        self.collision_avoidance_active = False  # 충돌 회피 활성화 여부
        
        # 색상 설정
        colors = [
            ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),  # 빨강
            ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0),  # 초록
            ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0),  # 파랑
        ]
        self.color = colors[agent_id % len(colors)]
    
    def snap_to_grid(self, grid_cell_size: float):
        """위치를 가장 가까운 격자점으로 스냅"""
        self.current_pos[0] = round(self.current_pos[0] / grid_cell_size) * grid_cell_size
        self.current_pos[1] = round(self.current_pos[1] / grid_cell_size) * grid_cell_size
    
    def get_grid_position(self, grid_cell_size: float):
        """현재 위치를 격자 인덱스로 변환"""
        # 정확한 격자 인덱스 계산 (반올림 사용)
        grid_x = int(round(self.current_pos[0] / grid_cell_size))
        grid_y = int(round(self.current_pos[1] / grid_cell_size))
        return grid_x, grid_y
    
    def get_grid_goal(self, grid_cell_size: float):
        """목표 위치를 격자 인덱스로 변환"""
        # 정확한 격자 인덱스 계산 (반올림 사용)
        grid_x = int(round(self.goal_pos[0] / grid_cell_size))
        grid_y = int(round(self.goal_pos[1] / grid_cell_size))
        return grid_x, grid_y
    
    def initialize_direction(self, grid_cell_size: float):
        """초기 이동 방향 설정 (목표지 방향)"""
        if self.direction_initialized:
            return
            
        current_grid_x, current_grid_y = self.get_grid_position(grid_cell_size)
        goal_grid_x, goal_grid_y = self.get_grid_goal(grid_cell_size)
        
        # 목표지 방향 벡터 계산
        dx = goal_grid_x - current_grid_x
        dy = goal_grid_y - current_grid_y
        
        # 목표지 방향으로 정확히 이동하는 방향 설정
        if dx == 0 and dy == 0:
            # 이미 목표에 도달한 경우
            self.initial_direction = (0, 0)
        elif dx == 0:
            # Y 방향으로만 이동
            self.initial_direction = (0, 1 if dy > 0 else -1)
        elif dy == 0:
            # X 방향으로만 이동
            self.initial_direction = (1 if dx > 0 else -1, 0)
        else:
            # 대각선 방향: 더 큰 차이를 가진 방향 우선
            if abs(dx) >= abs(dy):
                self.initial_direction = (1 if dx > 0 else -1, 0)
            else:
                self.initial_direction = (0, 1 if dy > 0 else -1)
        
        self.current_direction = self.initial_direction
        self.direction_initialized = True
        print(f"Agent {self.id}: 초기 이동 방향 설정 - {self.initial_direction} (목표: {goal_grid_x}, {goal_grid_y}, 현재: {current_grid_x}, {current_grid_y})")

    def update_position(self, dt: float, steering: Optional[np.ndarray] = None, grid_mode: bool = False, grid_cell_size: float = 1.0, collision_wait_time: float = 1.0):
        """에이전트 위치 업데이트"""
        if not self.is_moving:
            return
        
        # 대기 상태 처리
        if self.is_waiting:
            self.wait_timer += dt
            if self.wait_timer >= collision_wait_time:
                self.is_waiting = False
                self.wait_timer = 0.0
                print(f"Agent {self.id}: 대기 시간이 끝나 다시 이동을 시도합니다.")
            else:
                return  # 대기 중이면 이동하지 않음
            
        # 목표까지의 거리 계산
        to_goal = self.goal_pos - self.current_pos
        distance = np.linalg.norm(to_goal)
        
        # 그리드 모드와 연속 모드에 따른 목표 도달 조건 설정
        if grid_mode:
            # 그리드 모드: 더 정확한 목표 도달 조건 (격자 셀 크기의 1/4)
            goal_threshold = grid_cell_size * 0.25
        else:
            # 연속 모드: 기존 허용 오차
            goal_threshold = 0.3
            
        if distance < goal_threshold:
            # 정확한 목표 위치로 스냅
            self.current_pos = self.goal_pos.copy()
            if grid_mode:
                self.snap_to_grid(grid_cell_size)
            
            # 목표 도달 시 정지
            self.is_moving = False
            if not self.goal_reached_logged:
                print(f"Agent {self.id}: 목표 도달! 정지 상태로 전환 (거리: {distance:.3f}, 임계값: {goal_threshold:.3f})")
                self.goal_reached_logged = True
            return
            
        if grid_mode:
            # 그리드 모드: 격자 단위로만 이동
            current_grid_x, current_grid_y = self.get_grid_position(grid_cell_size)
            goal_grid_x, goal_grid_y = self.get_grid_goal(grid_cell_size)
            
            # 초기 이동 방향 설정 (한 번만)
            if not self.direction_initialized:
                self.initialize_direction(grid_cell_size)
            
            # 그리드 모드에서 목표 도달 판정 (격자 인덱스 기반)
            grid_distance = max(abs(current_grid_x - goal_grid_x), abs(current_grid_y - goal_grid_y))
            
            # 목표 도달 판정: 정확히 목표 격자에 도달했을 때만
            if grid_distance == 0:
                # 정확한 목표 위치로 스냅
                self.current_pos = self.goal_pos.copy()
                
                # 목표 도달 시 정지
                self.is_moving = False
                if not self.goal_reached_logged:
                    print(f"Agent {self.id}: 목표 도달! 정지 상태로 전환 (격자 거리: {grid_distance}, 실제 거리: {distance:.3f})")
                    self.goal_reached_logged = True
                return
            
            # 목표 위치에 도달하지 않았으면 현재 방향으로 이동
            if grid_distance > 0:
                # 현재 설정된 이동 방향 사용
                move_x, move_y = self.current_direction
                
                # 한 칸씩 이동 (격자 단위) - 정확한 격자점으로 이동
                new_x = self.current_pos[0]
                new_y = self.current_pos[1]
                
                if move_x != 0:
                    new_x = (current_grid_x + move_x) * grid_cell_size
                    # Y 좌표는 현재 위치 유지 (대각선 이동 방지)
                    new_y = self.current_pos[1]
                if move_y != 0:
                    new_y = (current_grid_y + move_y) * grid_cell_size
                    # X 좌표는 현재 위치 유지 (대각선 이동 방지)
                    new_x = self.current_pos[0]
                
                # 위치 업데이트 전에 유효성 검사 및 충돌 검사
                if -10.0 <= new_x <= 10.0 and -10.0 <= new_y <= 10.0:
                    # 충돌 검사를 위해 임시로 위치 저장
                    temp_pos = np.array([new_x, new_y], dtype=float)
                    
                    # 다른 에이전트와의 충돌 검사 (격자 인덱스 기반)
                    collision_detected = False
                    new_grid_x, new_grid_y = int(round(new_x / grid_cell_size)), int(round(new_y / grid_cell_size))
                    
                    for other_agent in self.parent_node.agents:
                        if other_agent.id != self.id:  # 모든 에이전트를 충돌 감지에 포함 (목표 도달한 에이전트도 포함)
                            # 격자 인덱스 기반 충돌 검사
                            other_grid_x, other_grid_y = other_agent.get_grid_position(grid_cell_size)
                            if new_grid_x == other_grid_x and new_grid_y == other_grid_y:
                                collision_detected = True
                                self.is_colliding = True
                                if other_agent.is_moving:  # 움직이는 에이전트만 충돌 상태 설정
                                    other_agent.is_colliding = True
                                print(f"Agent {self.id}: Agent {other_agent.id}와 격자 위치 충돌 감지")
                                break
                    
                    if not collision_detected:
                        # 충돌이 없으면 이동하고 충돌 회피 상태 해제
                        self.current_pos[0] = new_x
                        self.current_pos[1] = new_y
                        self.is_colliding = False  # 충돌 해결
                        
                        # 충돌 회피가 활성화되어 있었다면 원래 방향으로 복원
                        if self.collision_avoidance_active:
                            # 목표지 방향을 다시 계산하여 원래 방향으로 복원
                            goal_grid_x, goal_grid_y = self.get_grid_goal(grid_cell_size)
                            new_current_grid_x, new_current_grid_y = self.get_grid_position(grid_cell_size)
                            dx = goal_grid_x - new_current_grid_x
                            dy = goal_grid_y - new_current_grid_y
                            
                            # 목표지 방향으로 정확한 방향 복원 (초기 방향 설정과 동일한 로직)
                            if dx == 0 and dy == 0:
                                self.current_direction = (0, 0)
                            elif dx == 0:
                                self.current_direction = (0, 1 if dy > 0 else -1)
                            elif dy == 0:
                                self.current_direction = (1 if dx > 0 else -1, 0)
                            else:
                                if abs(dx) >= abs(dy):
                                    self.current_direction = (1 if dx > 0 else -1, 0)
                                else:
                                    self.current_direction = (0, 1 if dy > 0 else -1)
                            
                            self.collision_avoidance_active = False
                            print(f"Agent {self.id}: 충돌 회피 해제, 원래 방향으로 복원 - {self.current_direction}")
                        
                        print(f"Agent {self.id}: ({self.current_pos[0]:.1f}, {self.current_pos[1]:.1f})로 이동")
                    else:
                        # 충돌이 감지되면 목표지 방향을 고려한 90도 대안 경로 찾기
                        goal_grid_x, goal_grid_y = self.get_grid_goal(grid_cell_size)
                        dx = goal_grid_x - current_grid_x
                        dy = goal_grid_y - current_grid_y
                        
                        # 목표지 방향을 고려한 우선순위로 대안 경로 설정
                        alternative_moves = []
                        
                        # 목표지 방향으로의 이동을 우선 고려 (원래 방향 제외)
                        if dx > 0 and (1, 0) != self.current_direction:  # 목표지가 오른쪽에 있으면
                            alternative_moves.append((1, 0))   # 오른쪽 우선
                        elif dx < 0 and (-1, 0) != self.current_direction:  # 목표지가 왼쪽에 있으면
                            alternative_moves.append((-1, 0))  # 왼쪽 우선
                            
                        if dy > 0 and (0, 1) != self.current_direction:  # 목표지가 위에 있으면
                            alternative_moves.append((0, 1))   # 위 우선
                        elif dy < 0 and (0, -1) != self.current_direction:  # 목표지가 아래에 있으면
                            alternative_moves.append((0, -1))  # 아래 우선
                        
                        # 나머지 방향들 추가 (원래 방향과 목표지 방향 제외)
                        all_directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]
                        for direction in all_directions:
                            if direction not in alternative_moves and direction != self.current_direction:
                                alternative_moves.append(direction)
                        
                        alternative_found = False
                        for alt_x, alt_y in alternative_moves:
                            alt_new_x = (current_grid_x + alt_x) * grid_cell_size
                            alt_new_y = (current_grid_y + alt_y) * grid_cell_size
                            
                            # 대안 위치가 유효한지 확인
                            if -10.0 <= alt_new_x <= 10.0 and -10.0 <= alt_new_y <= 10.0:
                                # 대안 위치에서 충돌 검사
                                alt_collision_detected = False
                                alt_grid_x, alt_grid_y = int(round(alt_new_x / grid_cell_size)), int(round(alt_new_y / grid_cell_size))
                                
                                for other_agent in self.parent_node.agents:
                                    if other_agent.id != self.id:  # 모든 에이전트를 충돌 감지에 포함
                                        other_grid_x, other_grid_y = other_agent.get_grid_position(grid_cell_size)
                                        if alt_grid_x == other_grid_x and alt_grid_y == other_grid_y:
                                            alt_collision_detected = True
                                            break
                                
                                if not alt_collision_detected:
                                    # 대안 경로로 이동하고 현재 방향 업데이트
                                    self.current_pos[0] = alt_new_x
                                    self.current_pos[1] = alt_new_y
                                    self.current_direction = (alt_x, alt_y)  # 새로운 방향으로 업데이트
                                    self.collision_avoidance_active = True  # 충돌 회피 상태 활성화
                                    self.is_colliding = False
                                    print(f"Agent {self.id}: 대안 경로로 ({self.current_pos[0]:.1f}, {self.current_pos[1]:.1f})로 이동, 방향: {self.current_direction}")
                                    alternative_found = True
                                    break
                        
                        if not alternative_found:
                            # 모든 대안 경로가 막혀있으면 대기
                            self.is_waiting = True
                            self.wait_timer = 0.0  # 대기 타이머 초기화
                            print(f"Agent {self.id}: 모든 대안 경로가 막혀있어 대기합니다.")
                else:
                    print(f"Agent {self.id}: 유효하지 않은 위치 ({new_x:.1f}, {new_y:.1f}) - 이동 취소")
        else:
            # 연속 모드: 기존 로직
            # 방향 벡터 계산
            direction = to_goal / distance if distance > 0 else np.array([0.0, 0.0])
            
            # 속도 업데이트 (목표 속도 + 외부 스티어링)
            self.velocity = direction * self.max_speed
            if steering is not None:
                self.velocity = self.velocity + steering
            # 최고 속도 제한
            speed = np.linalg.norm(self.velocity)
            if speed > self.max_speed and speed > 0:
                self.velocity = (self.velocity / speed) * self.max_speed
            
            # 위치 업데이트
            self.current_pos += self.velocity * dt

class SimpleMAPFNode(Node):
    def __init__(self):
        super().__init__('simple_mapf_node')
        
        # 파라미터 설정
        self.declare_parameter('num_agents', 3)
        self.declare_parameter('update_rate', 30.0)
        self.declare_parameter('collision_distance', 1.5)
        
        # 파라미터 가져오기
        self.num_agents = self.get_parameter('num_agents').value
        self.update_rate = self.get_parameter('update_rate').value
        self.collision_distance = self.get_parameter('collision_distance').value
        
        # 충돌 회피 파라미터들 (GUI에서 조정 가능)
        self.avoidance_strength = 2.0  # 반발력 강도 배수
        self.prediction_time = 0.5     # 예측 시간 (초)
        self.repulsion_power = 0.5     # 반발력 지수 (0.5 = 제곱근, 1.0 = 선형, 2.0 = 제곱)
        self.collision_wait_time = 1.0  # 충돌 대기 시간 (초)
        
        # 움직임 모드 및 격자 설정
        self.movement_mode = 0  # 0: continuous, 1: grid
        self.grid_x_count = 20
        self.grid_y_count = 20
        self.grid_cell_size = 1.0  # 격자 셀 크기
        self.grid_move_timer = 0.0  # 그리드 이동 타이머
        self.grid_move_interval = 1.2  # 그리드 이동 간격 (초) - 더 느린 이동으로 1칸씩 이동 보장
        
        # 시뮬레이션 시작 제어
        self.simulation_started = False
        
        # 충돌 영역 시각화 설정
        self.show_collision_zones = True  # 충돌 영역 시각화 활성화
        self.collision_zone_alpha = 0.3   # 충돌 영역 투명도
        
        # 에이전트 리스트
        self.agents = []

        # 상태 로그 스로틀링을 위한 시간 저장
        self._last_status_log_time = 0.0
        
        # 퍼블리셔 설정
        self.agent_poses_pub = self.create_publisher(MarkerArray, '/agent_poses', 10)
        self.collision_info_pub = self.create_publisher(String, '/collision_info', 10)
        self.grid_visualization_pub = self.create_publisher(MarkerArray, '/grid_visualization', 10)
        self.start_goal_poses_pub = self.create_publisher(MarkerArray, '/start_goal_poses', 10)
        self.collision_detection_zones_pub = self.create_publisher(MarkerArray, '/collision_detection_zones', 10)

        # 구독자 설정
        self.reset_sub = self.create_subscription(Bool, '/reset_simulation', self.on_reset_simulation, 10)
        self.update_rate_sub = self.create_subscription(Float32, '/update_rate', self.on_update_rate, 10)
        self.num_agents_sub = self.create_subscription(Int32, '/num_agents', self.on_num_agents, 10)
        self.basic_settings_sub = self.create_subscription(String, '/basic_settings', self.on_basic_settings, 10)
        self.collision_settings_sub = self.create_subscription(String, '/collision_settings', self.on_collision_settings, 10)
        self.collision_distance_sub = self.create_subscription(Float32, '/collision_distance', self.on_collision_distance, 10)
        self.add_zone_sub = self.create_subscription(Point, '/add_collision_zone', self.on_add_collision_zone, 10)
        self.clear_zones_sub = self.create_subscription(Bool, '/clear_collision_zones', self.on_clear_collision_zones, 10)
        
        # 충돌 회피 파라미터 구독자들
        self.avoidance_strength_sub = self.create_subscription(Float32, '/avoidance_strength', self.on_avoidance_strength, 10)
        self.prediction_time_sub = self.create_subscription(Float32, '/prediction_time', self.on_prediction_time, 10)
        self.repulsion_power_sub = self.create_subscription(Float32, '/repulsion_power', self.on_repulsion_power, 10)
        
        # 움직임 모드 및 격자 설정 구독자들
        self.movement_mode_sub = self.create_subscription(Int32, '/movement_mode', self.on_movement_mode, 10)
        self.grid_size_sub = self.create_subscription(Point, '/grid_size', self.on_grid_size, 10)
        self.grid_move_interval_sub = self.create_subscription(Float32, '/grid_move_interval', self.on_grid_move_interval, 10)
        
        # 에이전트 위치 설정 구독자
        self.agent_positions_sub = self.create_subscription(Point, '/agent_positions', self.on_agent_positions, 10)
        
        # 시뮬레이션 시작 제어 구독자
        self.simulation_start_sub = self.create_subscription(Bool, '/start_simulation', self.on_start_simulation, 10)
        
        # 시뮬레이션 리셋 제어 구독자
        self.simulation_reset_sub = self.create_subscription(Bool, '/reset_simulation', self.on_reset_simulation, 10)
        
        # 충돌 존 저장 및 퍼블리셔
        self.collision_zones = []  # 각 원소: (x, y, radius)
        self.collision_zones_pub = self.create_publisher(MarkerArray, '/collision_zones', 10)
        
        # 마커 깜빡임 방지를 위한 마지막 발행 위치 저장
        self.last_published_positions = {}

        # 타이머 설정
        self.timer = self.create_timer(1.0 / self.update_rate, self.update_simulation)
        
        # 시각화 업데이트 타이머 (시뮬레이션 상태와 관계없이 계속 실행)
        self.viz_timer = self.create_timer(0.1, self.update_visualization)  # 10Hz로 시각화 업데이트
        
        # 에이전트 초기화 (모든 설정 완료 후)
        self.initialize_agents()
        
        # 초기 시각화 실행 (모든 에이전트가 보이도록)
        self.publish_agent_poses()
        self.publish_start_goal_poses()
        
        self.get_logger().info(f'간단한 MAPF 시뮬레이터가 시작되었습니다. 에이전트 수: {self.num_agents}')
        self.get_logger().info('GUI에서 "시뮬레이션 시작" 버튼을 눌러 시뮬레이션을 시작하세요.')
        self.get_logger().info('시뮬레이션을 중단하려면 Ctrl+C를 누르세요.')
    
    def initialize_agents(self):
        """에이전트 초기화"""
        for i in range(self.num_agents):
            # 시작 위치와 목표 위치를 격자점에 맞춰 설정
            start_x = -5.0 + i * 2.0
            start_y = -3.0
            goal_x = 5.0 - i * 2.0
            goal_y = 3.0
            
            agent = SimpleAgent(i, (start_x, start_y), (goal_x, goal_y))
            
            # 에이전트에 부모 노드 참조 추가 (충돌 검사용)
            agent.parent_node = self
            
            # 에이전트를 리스트에 추가 (격자 스냅은 나중에)
            self.agents.append(agent)
            
            # 에이전트를 격자점에 스냅
            agent.snap_to_grid(self.grid_cell_size)
            
            # 에이전트를 정지 상태로 설정 (시뮬레이션 시작 전까지)
            agent.is_moving = False
            agent.goal_reached_logged = False  # 목표 도달 로그 상태 리셋
            agent.is_colliding = False  # 충돌 상태 리셋
            agent.is_collision_predicted = False  # 충돌 예상 상태 리셋
            agent.prev_collision_state = False  # 이전 충돌 상태 리셋
            agent.prev_prediction_state = False  # 이전 충돌 예상 상태 리셋
            agent.direction_initialized = False  # 방향 초기화 상태 리셋
            agent.current_direction = None  # 현재 방향 리셋
            agent.collision_avoidance_active = False  # 충돌 회피 상태 리셋
            agent.last_collision_position = None  # 마지막 충돌 위치 리셋
            
            self.get_logger().info(f'에이전트 {i}: 시작({start_x:.1f}, {start_y:.1f}) -> 목표({goal_x:.1f}, {goal_y:.1f})')
    
    def check_simulation_completion(self):
        """시뮬레이션 완료 체크 - 모든 에이전트가 목표에 도달했으면 자동 중지"""
        if not self.simulation_started or not self.agents:
            return
            
        # 모든 에이전트가 목표에 도달했는지 확인
        all_reached = all(not agent.is_moving for agent in self.agents)
        
        if all_reached:
            # 시뮬레이션 자동 중지
            self.simulation_started = False
            print("모든 에이전트가 목표에 도달했습니다. 시뮬레이션을 자동으로 중지합니다.")
            # ROS2 토픽 발행은 제거 (웹 서버에서 처리)
    
    def update_simulation(self):
        """시뮬레이션 업데이트"""
        # 시뮬레이션이 시작되지 않았으면 업데이트하지 않음
        if not self.simulation_started:
            return
            
        dt = 1.0 / self.update_rate
        
        # 그리드 모드 타이머 업데이트
        if self.movement_mode == 1:  # 그리드 모드
            self.grid_move_timer += dt
        
        # 각 에이전트 업데이트 (간단한 충돌 회피 스티어링 적용)
        # 모든 에이전트를 충돌 감지에 참여 (목표에 도달한 에이전트도 포함)
        all_agents = self.agents
        moving_agents = [agent for agent in self.agents if agent.is_moving]
        positions = [agent.current_pos.copy() for agent in all_agents]
        radii = [agent.radius for agent in all_agents]
        
        for i, agent in enumerate(self.agents):
            # 에이전트가 움직이지 않아도 위치는 업데이트 (시각화를 위해)
            if not agent.is_moving:
                # 위치 업데이트는 하지 않지만 시각화는 계속
                pass
            else:
                # 그리드 모드와 연속 모드에 따라 다른 충돌 회피 로직 적용
                if self.movement_mode == 1:  # 그리드 모드
                    # 그리드 모드에서는 스티어링을 사용하지 않음 (격자 기반 이동)
                    # 격자 기반 충돌 감지는 update_position 함수에서 처리됨
                    steering = np.zeros(2, dtype=float)
                else:  # 연속 모드
                    # 움직이는 에이전트만 충돌 회피 스티어링 적용
                    steering = np.zeros(2, dtype=float)
                    
                    # 에이전트의 이동 방향 계산 (목표 방향)
                    if agent.is_moving:
                        goal_direction = agent.goal_pos - agent.current_pos
                        goal_distance = float(np.linalg.norm(goal_direction))
                        if goal_distance > 1e-6:
                            goal_direction = goal_direction / goal_distance
                        else:
                            goal_direction = np.zeros(2, dtype=float)
                    else:
                        goal_direction = np.zeros(2, dtype=float)
                    
                    for j, other_agent in enumerate(all_agents):
                        if agent == other_agent:  # 같은 에이전트는 건너뛰기
                            continue
                        if not agent.is_moving:  # 현재 에이전트가 목표에 도달했다면 충돌 감지하지 않음
                            continue
                        
                        other_pos = other_agent.current_pos
                        diff = agent.current_pos - other_pos
                        dist = float(np.linalg.norm(diff))
                        if dist < 1e-6:
                            continue
                        
                        # 이동 방향을 고려한 충돌 감지
                        # 에이전트가 이동하지 않거나, 다른 에이전트가 이동 방향에 있지 않으면 충돌 감지하지 않음
                        if not agent.is_moving or np.dot(goal_direction, -diff) <= 0:
                            continue
                        
                        # 연속 모드에서는 충돌 감지 거리를 더 크게 설정하여 조기 회피
                        continuous_adjusted_distance = self.collision_distance * 1.2
                        # 충돌 감지 거리가 에이전트 반지름 합보다 작으면 에이전트 반지름 합을 사용
                        desired_sep = max(continuous_adjusted_distance, agent.radius + other_agent.radius)
                        prediction_threshold = desired_sep * 1.2  # 충돌 예상 임계값
                        
                        if dist < desired_sep:
                            # 충돌 상태 설정
                            agent.is_colliding = True
                            other_agent.is_colliding = True
                            
                            overlap = desired_sep - dist
                            repulse_dir = diff / dist
                            # 조정 가능한 반발 스티어링
                            repulse_strength = (overlap / desired_sep) ** self.repulsion_power * agent.max_speed * self.avoidance_strength
                            steering += repulse_dir * repulse_strength
                        elif dist < prediction_threshold:
                            # 충돌 예상 상태 설정
                            agent.is_collision_predicted = True
                            other_agent.is_collision_predicted = True
                # 충돌 존 반발력 추가 (움직이는 에이전트에만)
                for (zx, zy, zr) in self.collision_zones:
                    to_agent = agent.current_pos - np.array([zx, zy], dtype=float)
                    dist_zone = float(np.linalg.norm(to_agent))
                    if dist_zone < 1e-6:
                        continue
                    desired_zone_sep = zr + agent.radius
                    if dist_zone < desired_zone_sep:
                        overlap_z = desired_zone_sep - dist_zone
                        repel_dir_z = to_agent / dist_zone
                        steering += repel_dir_z * (overlap_z / desired_zone_sep) * agent.max_speed

                # 예측적 충돌 회피 (조정 가능한 예측 시간) - 연속 모드에서만 적용
                if self.movement_mode == 0 and np.linalg.norm(agent.velocity) > 0 and agent.is_moving:
                    future_pos = agent.current_pos + agent.velocity * self.prediction_time
                    for other_agent in all_agents:
                        if agent == other_agent:
                            continue
                        
                        
                        # 이동 방향을 고려한 예측적 충돌 감지
                        other_future_pos = other_agent.current_pos + other_agent.velocity * self.prediction_time
                        future_diff = future_pos - other_future_pos
                        future_dist = float(np.linalg.norm(future_diff))
                        
                        # 에이전트가 이동 방향에 있는 다른 에이전트와만 충돌 감지
                        if np.dot(goal_direction, -future_diff) <= 0:
                            continue
                        
                        # 그리드 모드에서는 그리드 셀 크기에 맞게 조정
                        if self.movement_mode == 1:  # 그리드 모드
                            grid_adjusted_distance = min(self.collision_distance, self.grid_cell_size * 1.5)
                            desired_sep = max(grid_adjusted_distance, agent.radius + other_agent.radius)
                        else:  # 연속 모드
                            desired_sep = max(self.collision_distance, agent.radius + other_agent.radius)
                        if future_dist < desired_sep and future_dist > 1e-6:
                            # 미래 충돌 예상 시 추가 회피 (강도 조정 가능)
                            avoidance_dir = future_diff / future_dist
                            steering += avoidance_dir * agent.max_speed * (self.avoidance_strength * 0.4)
            
            # 위치 업데이트에 반영
            if self.movement_mode == 1:  # 그리드 모드
                # 타이머 기반 이동 제어 (움직이는 에이전트만 이동)
                if self.grid_move_timer >= self.grid_move_interval and agent.is_moving:
                    # 그리드 모드에서는 steering을 사용하지 않음 (격자 기반 이동)
                    agent.update_position(dt, None, grid_mode=True, grid_cell_size=self.grid_cell_size, collision_wait_time=self.collision_wait_time)
            else:  # 연속 모드
                if agent.is_moving:  # 움직이는 에이전트만 위치 업데이트
                    agent.update_position(dt, steering, grid_mode=False, grid_cell_size=self.grid_cell_size, collision_wait_time=self.collision_wait_time)
            # 월드 경계 체크
            agent.current_pos[0] = np.clip(agent.current_pos[0], -10.0, 10.0)
            agent.current_pos[1] = np.clip(agent.current_pos[1], -10.0, 10.0)
        
        # 그리드 모드에서 타이머 리셋
        if self.movement_mode == 1 and self.grid_move_timer >= self.grid_move_interval:
            self.grid_move_timer = 0.0
        
        # 충돌 상태 리셋 (연속 모드에서 충돌이 해결되었는지 확인)
        if self.movement_mode == 0:  # 연속 모드
            for agent in self.agents:
                # 충돌 상태 리셋 (움직이는 에이전트만 고려)
                if agent.is_colliding:
                    collision_resolved = True
                    for other_agent in moving_agents:
                        if agent != other_agent:
                            dist = float(np.linalg.norm(agent.current_pos - other_agent.current_pos))
                            desired_sep = max(self.collision_distance, agent.radius + other_agent.radius)
                            if dist < desired_sep:
                                collision_resolved = False
                                break
                    if collision_resolved:
                        agent.is_colliding = False
                
                # 충돌 예상 상태 리셋 (움직이는 에이전트만 고려)
                if agent.is_collision_predicted:
                    prediction_resolved = True
                    for other_agent in moving_agents:
                        if agent != other_agent:
                            dist = float(np.linalg.norm(agent.current_pos - other_agent.current_pos))
                            desired_sep = max(self.collision_distance, agent.radius + other_agent.radius)
                            prediction_threshold = desired_sep * 1.2
                            if dist < prediction_threshold:
                                prediction_resolved = False
                                break
                    if prediction_resolved:
                        agent.is_collision_predicted = False
        
        # 시뮬레이션 완료 체크
        self.check_simulation_completion()
        
        # 충돌 정보 발행
        self.publish_collision_info()
        
        # 시각화 업데이트는 별도 타이머에서 처리
        pass
        
        # 상태 출력 (5초 스로틀)
        now = time.time()
        if now - self._last_status_log_time >= 5.0:
            self._last_status_log_time = now
            self.print_status()
    
    def update_visualization(self):
        """시각화 업데이트 (시뮬레이션 상태와 관계없이 계속 실행)"""
        # 에이전트 위치 시각화
        self.publish_agent_poses()
        
        # 시작/목표 위치 시각화
        self.publish_start_goal_poses()
        
        # 충돌 영역 시각화
        self.publish_collision_zones()
        
        # 충돌 감지 영역 시각화
        self.publish_collision_detection_zones()
        
        # 격자 시각화
        self.publish_grid_visualization()
    
    def publish_agent_poses(self):
        """에이전트 위치 시각화 (웹 인터페이스용 데이터 전송)"""
        marker_array = MarkerArray()
        
        for agent in self.agents:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = f"agent_{agent.id}"
            marker.id = agent.id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            # 위치 설정
            marker.pose.position.x = agent.current_pos[0]
            marker.pose.position.y = agent.current_pos[1]
            # Grid(평면 z=0)와의 z-fighting 방지를 위해 확실히 높이 올림
            marker.pose.position.z = 0.5
            marker.pose.orientation.w = 1.0
            
            # 크기 설정
            marker.scale.x = agent.radius * 2
            marker.scale.y = agent.radius * 2
            marker.scale.z = 0.5
            
            # 색상 설정
            marker.color = agent.color
            
            # 이동 상태에 따른 투명도 조정
            if agent.is_moving:
                marker.color.a = 1.0  # 움직이는 에이전트는 불투명
            else:
                marker.color.a = 0.3  # 정지한 에이전트는 반투명
            
            marker_array.markers.append(marker)
        
        # 웹 인터페이스로 실시간 데이터 전송
        self.agent_poses_pub.publish(marker_array)
    
    def publish_start_goal_poses(self):
        """시작위치와 목표위치를 MarkerArray로 발행"""
        marker_array = MarkerArray()
        
        for agent in self.agents:
            # 시작 위치 마커 (작은 구)
            start_marker = Marker()
            start_marker.header.frame_id = "map"
            start_marker.header.stamp = self.get_clock().now().to_msg()
            start_marker.ns = f"start_agent_{agent.id}"
            start_marker.id = agent.id * 2
            start_marker.type = Marker.SPHERE
            start_marker.action = Marker.ADD
            start_marker.pose.position.x = agent.start_pos[0]
            start_marker.pose.position.y = agent.start_pos[1]
            start_marker.pose.position.z = 0.1
            start_marker.pose.orientation.w = 1.0
            start_marker.scale.x = 0.3
            start_marker.scale.y = 0.3
            start_marker.scale.z = 0.3
            # 시작 위치는 반투명한 회색
            start_marker.color = ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.5)
            marker_array.markers.append(start_marker)
            
            # 목표 위치 마커 (큰 구)
            goal_marker = Marker()
            goal_marker.header.frame_id = "map"
            goal_marker.header.stamp = self.get_clock().now().to_msg()
            goal_marker.ns = f"goal_agent_{agent.id}"
            goal_marker.id = agent.id * 2 + 1
            goal_marker.type = Marker.SPHERE
            goal_marker.action = Marker.ADD
            goal_marker.pose.position.x = agent.goal_pos[0]
            goal_marker.pose.position.y = agent.goal_pos[1]
            goal_marker.pose.position.z = 0.1
            goal_marker.pose.orientation.w = 1.0
            goal_marker.scale.x = 0.5
            goal_marker.scale.y = 0.5
            goal_marker.scale.z = 0.5
            # 목표 위치는 에이전트 색상과 같은 색이지만 반투명
            goal_marker.color = ColorRGBA(
                r=agent.color.r, 
                g=agent.color.g, 
                b=agent.color.b, 
                a=0.3
            )
            marker_array.markers.append(goal_marker)
        
        self.start_goal_poses_pub.publish(marker_array)
    
    def print_status(self):
        """상태 출력"""
        status_msg = "에이전트 상태: "
        for agent in self.agents:
            status = "이동중" if agent.is_moving else "도착"
            status_msg += f"A{agent.id}({agent.current_pos[0]:.1f}, {agent.current_pos[1]:.1f})[{status}] "
        
        self.get_logger().info(status_msg)

    def on_reset_simulation(self, msg: Bool):
        """/reset_simulation 콜백: 에이전트 재초기화"""
        if not msg.data:
            return
        self.get_logger().info('리셋 요청을 수신했습니다. 시뮬레이션을 재초기화합니다.')
        self.reset_simulation()

    def on_update_rate(self, msg: Float32):
        """/update_rate 콜백: 업데이트 속도 변경 및 타이머 재생성"""
        new_rate = float(msg.data)
        if new_rate <= 0.0:
            self.get_logger().warn(f'유효하지 않은 update_rate: {new_rate}. 무시합니다.')
            return
        if abs(new_rate - self.update_rate) < 1e-6:
            return
        self.update_rate = new_rate
        try:
            if self.timer is not None:
                self.timer.cancel()
        except Exception:
            pass
        self.timer = self.create_timer(1.0 / self.update_rate, self.update_simulation)
        self.get_logger().info(f'업데이트 속도를 {self.update_rate:.1f} Hz로 변경했습니다.')

    def on_num_agents(self, msg: Int32):
        """/num_agents 콜백: 에이전트 수 변경 후 재초기화"""
        new_num = int(msg.data)
        if new_num <= 0:
            self.get_logger().warn(f'유효하지 않은 에이전트 수: {new_num}. 무시합니다.')
            return
        if new_num == self.num_agents:
            return
        self.num_agents = new_num
        self.get_logger().info(f'에이전트 수를 {self.num_agents}로 변경합니다. 재초기화합니다.')
        self.reset_simulation()

    def on_basic_settings(self, msg: String):
        """/basic_settings 콜백: 기본 설정 업데이트"""
        try:
            data = json.loads(msg.data)
            num_agents = data.get('num_agents')
            if num_agents and num_agents != self.num_agents:
                self.num_agents = num_agents
                self.get_logger().info(f'기본 설정에서 에이전트 수를 {self.num_agents}로 변경합니다.')
                self.reset_simulation()
            
            # 충돌 영역 시각화 설정
            if 'show_collision_zones' in data:
                self.show_collision_zones = bool(data['show_collision_zones'])
                self.get_logger().info(f'충돌 영역 시각화를 {"활성화" if self.show_collision_zones else "비활성화"}했습니다.')
                
        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().warn(f'기본 설정 파싱 실패: {e}')

    def on_collision_settings(self, msg: String):
        """/collision_settings 콜백: 충돌 설정 업데이트"""
        try:
            data = json.loads(msg.data)
            
            # collision_distance 설정
            if 'collision_distance' in data:
                new_dist = float(data['collision_distance'])
                if new_dist > 0.0:
                    self.collision_distance = new_dist
                    self.get_logger().info(f'충돌 설정에서 collision_distance를 {self.collision_distance:.2f}로 변경했습니다.')
                else:
                    self.get_logger().warn(f'유효하지 않은 collision_distance: {new_dist}')
            
            # avoidance_strength 설정
            if 'avoidance_strength' in data:
                new_strength = float(data['avoidance_strength'])
                if new_strength >= 0.0:
                    self.avoidance_strength = new_strength
                    self.get_logger().info(f'충돌 설정에서 avoidance_strength를 {self.avoidance_strength:.2f}로 변경했습니다.')
            
            # prediction_time 설정
            if 'prediction_time' in data:
                new_time = float(data['prediction_time'])
                if new_time >= 0.0:
                    self.prediction_time = new_time
                    self.get_logger().info(f'충돌 설정에서 prediction_time를 {self.prediction_time:.2f}로 변경했습니다.')
            
            # repulsion_power 설정
            if 'repulsion_power' in data:
                new_power = float(data['repulsion_power'])
                if new_power >= 0.0:
                    self.repulsion_power = new_power
                    self.get_logger().info(f'충돌 설정에서 repulsion_power를 {self.repulsion_power:.2f}로 변경했습니다.')
            
            # collision_wait_time 설정
            if 'collision_wait_time' in data:
                new_wait_time = float(data['collision_wait_time'])
                if new_wait_time >= 0.0:
                    self.collision_wait_time = new_wait_time
                    self.get_logger().info(f'충돌 설정에서 collision_wait_time를 {self.collision_wait_time:.2f}로 변경했습니다.')
            
            # 충돌 설정이 변경되었으므로 시각화 업데이트
            self.get_logger().info('충돌 설정이 변경되어 시각화를 업데이트합니다.')
            # 시각화 업데이트를 강제로 트리거
            self.publish_collision_detection_zones()
                    
        except (json.JSONDecodeError, KeyError, ValueError) as e:
            self.get_logger().warn(f'충돌 설정 파싱 실패: {e}')

    def publish_collision_info(self):
        """충돌 정보를 ROS2 토픽으로 발행"""
        try:
            collision_info = []
            state_changed = False
            
            for agent in self.agents:
                # 충돌 상태 변화 확인
                if agent.is_colliding != agent.prev_collision_state:
                    agent.prev_collision_state = agent.is_colliding
                    state_changed = True
                    if agent.is_colliding:
                        self.get_logger().info(f'에이전트 {agent.id}: 충돌 발생!')
                
                # 충돌 예상 상태 변화 확인
                if agent.is_collision_predicted != agent.prev_prediction_state:
                    agent.prev_prediction_state = agent.is_collision_predicted
                    state_changed = True
                    if agent.is_collision_predicted:
                        self.get_logger().info(f'에이전트 {agent.id}: 충돌 예상!')
                
                # 충돌 정보 수집
                if agent.is_colliding:
                    collision_info.append({
                        'agent_id': agent.id,
                        'position': [float(agent.current_pos[0]), float(agent.current_pos[1])],
                        'collision_type': 'agent_agent'
                    })
                elif agent.is_collision_predicted:
                    collision_info.append({
                        'agent_id': agent.id,
                        'position': [float(agent.current_pos[0]), float(agent.current_pos[1])],
                        'collision_type': 'predicted'
                    })
            
            # 충돌 정보를 JSON 문자열로 발행
            collision_msg = String()
            collision_msg.data = json.dumps(collision_info)
            self.collision_info_pub.publish(collision_msg)
            
        except Exception as e:
            self.get_logger().warn(f'충돌 정보 발행 실패: {e}')

    def reset_simulation(self):
        """에이전트 재초기화 및 상태 리셋"""
        self.agents = []
        self.initialize_agents()
        self._last_status_log_time = 0.0
        self.last_published_positions = {}  # 마커 위치 기록 초기화
        # 존은 유지 (원하면 on_clear_collision_zones로 제거)

    def on_collision_distance(self, msg: Float32):
        """/collision_distance 콜백: 에이전트 간 최소 거리 변경"""
        new_dist = float(msg.data)
        if new_dist <= 0.0:
            self.get_logger().warn(f'유효하지 않은 collision_distance: {new_dist}. 무시합니다.')
            return
        self.collision_distance = new_dist
        self.get_logger().info(f'collision_distance를 {self.collision_distance:.2f}로 설정했습니다.')

    def on_add_collision_zone(self, msg: Point):
        """/add_collision_zone 콜백: 충돌 존 추가 (msg.z를 반지름으로 사용)"""
        self.collision_zones.append((float(msg.x), float(msg.y), float(msg.z)))
        self.get_logger().info(f'충돌 존 추가: ({msg.x:.1f}, {msg.y:.1f}), r={msg.z:.1f}')

    def on_clear_collision_zones(self, msg: Bool):
        """/clear_collision_zones 콜백: 모든 존 삭제"""
        if msg.data:
            self.collision_zones.clear()
            self.get_logger().info('모든 충돌 존을 제거했습니다.')

    def publish_collision_zones(self):
        """충돌 존을 MarkerArray로 발행"""
        marker_array = MarkerArray()
        for idx, (zx, zy, zr) in enumerate(self.collision_zones):
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "collision_zones"
            m.id = idx
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = zx
            m.pose.position.y = zy
            m.pose.position.z = 0.0
            m.pose.orientation.w = 1.0
            m.scale.x = zr * 2.0
            m.scale.y = zr * 2.0
            m.scale.z = 0.1
            m.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.3)
            marker_array.markers.append(m)
        self.collision_zones_pub.publish(marker_array)

    def publish_collision_detection_zones(self):
        """에이전트 주변의 충돌 감지 영역을 시각화"""
        if not self.show_collision_zones:
            return
            
        marker_array = MarkerArray()
        
        for agent in self.agents:
            # 충돌 감지 영역 (원형)
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "collision_detection_zones"
            marker.id = agent.id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            # 에이전트 위치에 중심
            marker.pose.position.x = agent.current_pos[0]
            marker.pose.position.y = agent.current_pos[1]
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            
            # 충돌 감지 거리로 반지름 설정 (실제 충돌 감지 로직과 일치)
            # 그리드 모드에서는 그리드 셀 크기에 맞게 조정
            if self.movement_mode == 1:  # 그리드 모드
                grid_adjusted_distance = min(self.collision_distance, self.grid_cell_size * 1.0)
                detection_radius = max(grid_adjusted_distance, agent.radius * 2)
            else:  # 연속 모드
                detection_radius = max(self.collision_distance, agent.radius * 2)
            marker.scale.x = detection_radius * 2.0
            marker.scale.y = detection_radius * 2.0
            marker.scale.z = 0.05  # 얇게 표시
            
            # 반투명한 빨간색으로 표시
            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=self.collision_zone_alpha)
            marker_array.markers.append(marker)
            
            # 에이전트 반지름 영역 (더 진한 색으로)
            agent_marker = Marker()
            agent_marker.header.frame_id = "map"
            agent_marker.header.stamp = self.get_clock().now().to_msg()
            agent_marker.ns = "agent_radius_zones"
            agent_marker.id = agent.id
            agent_marker.type = Marker.CYLINDER
            agent_marker.action = Marker.ADD
            
            agent_marker.pose.position.x = agent.current_pos[0]
            agent_marker.pose.position.y = agent.current_pos[1]
            agent_marker.pose.position.z = 0.0
            agent_marker.pose.orientation.w = 1.0
            
            agent_marker.scale.x = agent.radius * 2.0
            agent_marker.scale.y = agent.radius * 2.0
            agent_marker.scale.z = 0.1
            
            # 반투명한 파란색으로 표시
            agent_marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=self.collision_zone_alpha * 0.7)
            marker_array.markers.append(agent_marker)
        
        # 충돌 감지 영역 마커 발행
        if hasattr(self, 'collision_detection_zones_pub'):
            self.collision_detection_zones_pub.publish(marker_array)

    def on_avoidance_strength(self, msg: Float32):
        """/avoidance_strength 콜백: 반발력 강도 조정"""
        new_strength = float(msg.data)
        if new_strength < 0.0:
            self.get_logger().warn(f'유효하지 않은 avoidance_strength: {new_strength}. 무시합니다.')
            return
        self.avoidance_strength = new_strength
        self.get_logger().info(f'반발력 강도를 {self.avoidance_strength:.2f}로 설정했습니다.')

    def on_prediction_time(self, msg: Float32):
        """/prediction_time 콜백: 예측 시간 조정"""
        new_time = float(msg.data)
        if new_time < 0.0:
            self.get_logger().warn(f'유효하지 않은 prediction_time: {new_time}. 무시합니다.')
            return
        self.prediction_time = new_time
        self.get_logger().info(f'예측 시간을 {self.prediction_time:.2f}초로 설정했습니다.')

    def on_repulsion_power(self, msg: Float32):
        """/repulsion_power 콜백: 반발력 지수 조정"""
        new_power = float(msg.data)
        if new_power < 0.1 or new_power > 3.0:
            self.get_logger().warn(f'유효하지 않은 repulsion_power: {new_power}. 0.1~3.0 범위여야 합니다.')
            return
        self.repulsion_power = new_power
        self.get_logger().info(f'반발력 지수를 {self.repulsion_power:.2f}로 설정했습니다.')

    def on_movement_mode(self, msg: Int32):
        """/movement_mode 콜백: 움직임 모드 변경"""
        new_mode = int(msg.data)
        if new_mode not in [0, 1]:
            self.get_logger().warn(f'유효하지 않은 movement_mode: {new_mode}. 무시합니다.')
            return
        self.movement_mode = new_mode
        self.get_logger().info(f'움직임 모드를 {self.movement_mode}로 변경했습니다.')

    def on_grid_size(self, msg: Point):
        """/grid_size 콜백: 격자 크기 변경"""
        self.grid_x_count = int(msg.x)
        self.grid_y_count = int(msg.y)
        # 격자 셀 크기는 월드 크기 / 격자 수로 계산
        self.grid_cell_size = 20.0 / max(self.grid_x_count, self.grid_y_count)
        self.get_logger().info(f'격자 크기를 {self.grid_x_count}x{self.grid_y_count}로 설정했습니다. 셀 크기: {self.grid_cell_size:.2f}')
    
    def on_grid_move_interval(self, msg: Float32):
        """/grid_move_interval 콜백: 그리드 이동 간격 변경"""
        new_interval = float(msg.data)
        if new_interval < 0.1 or new_interval > 3.0:
            self.get_logger().warn(f'유효하지 않은 grid_move_interval: {new_interval}. 0.1~3.0 범위여야 합니다.')
            return
        self.grid_move_interval = new_interval
        self.get_logger().info(f'그리드 이동 간격을 {self.grid_move_interval:.2f}초로 설정했습니다.')
    
    def on_agent_positions(self, msg: Point):
        """에이전트 위치 설정 업데이트"""
        agent_id = int(msg.z)
        if 0 <= agent_id < len(self.agents):
            agent = self.agents[agent_id]
            # Point 메시지의 x, y가 시작 위치인지 목표 위치인지 구분
            # 짝수 번째는 시작 위치, 홀수 번째는 목표 위치로 가정
            if hasattr(self, '_position_update_count'):
                self._position_update_count += 1
            else:
                self._position_update_count = 0
            
            if self._position_update_count % 2 == 0:  # 시작 위치
                agent.start_pos = np.array([msg.x, msg.y], dtype=float)
                agent.current_pos = agent.start_pos.copy()
                self.get_logger().info(f'에이전트 {agent_id} 시작 위치를 ({msg.x:.1f}, {msg.y:.1f})로 설정했습니다.')
            else:  # 목표 위치
                agent.goal_pos = np.array([msg.x, msg.y], dtype=float)
                self.get_logger().info(f'에이전트 {agent_id} 목표 위치를 ({msg.x:.1f}, {msg.y:.1f})로 설정했습니다.')
            
            # 에이전트를 격자점에 스냅
            agent.snap_to_grid(self.grid_cell_size)
            # 시뮬레이션이 시작된 경우에만 움직이는 상태로 설정
            if self.simulation_started:
                agent.is_moving = True
        else:
            self.get_logger().warn(f'유효하지 않은 에이전트 ID: {agent_id}')
    
    def on_start_simulation(self, msg: Bool):
        """시뮬레이션 시작 제어"""
        if msg.data:
            self.simulation_started = True
            # 모든 에이전트를 움직이는 상태로 설정
            for agent in self.agents:
                agent.is_moving = True
        else:
            self.simulation_started = False
            # 모든 에이전트를 정지 상태로 설정
            for agent in self.agents:
                agent.is_moving = False
    
    def on_reset_simulation(self, msg: Bool):
        """시뮬레이션 리셋"""
        if msg.data:
            # 시뮬레이션 중지
            self.simulation_started = False
            
            # 모든 에이전트를 시작 위치로 이동하고 정지
            for agent in self.agents:
                agent.current_pos = agent.start_pos.copy()
                agent.is_moving = False
                # 격자 모드인 경우 격자점에 스냅
                if self.movement_mode == 1:  # 그리드 모드
                    agent.snap_to_grid(self.grid_cell_size)
            
            self.get_logger().info('시뮬레이션이 리셋되었습니다. 모든 에이전트가 시작 위치로 이동했습니다.')

    def publish_grid_visualization(self):
        """격자 그리드를 MarkerArray로 발행"""
        marker_array = MarkerArray()
        
        # 수직선 그리기
        for i in range(self.grid_x_count + 1):
            x = -10.0 + i * self.grid_cell_size
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "grid_lines"
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.05  # 선 두께
            marker.color = ColorRGBA(r=0.3, g=0.3, b=0.3, a=0.3)
            
            # 수직선의 시작점과 끝점
            marker.points = [Point(x=x, y=-10.0, z=0.0), Point(x=x, y=10.0, z=0.0)]
            marker_array.markers.append(marker)
        
        # 수평선 그리기
        for i in range(self.grid_y_count + 1):
            y = -10.0 + i * self.grid_cell_size
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "grid_lines"
            marker.id = i + self.grid_x_count + 1
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.05  # 선 두께
            marker.color = ColorRGBA(r=0.3, g=0.3, b=0.3, a=0.3)
            
            # 수평선의 시작점과 끝점
            marker.points = [Point(x=-10.0, y=y, z=0.0), Point(x=10.0, y=y, z=0.0)]
            marker_array.markers.append(marker)
        
        self.grid_visualization_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    
    simulator = SimpleMAPFNode()
    
    try:
        rclpy.spin(simulator)
    except KeyboardInterrupt:
        simulator.get_logger().info('시뮬레이션을 종료합니다.')
    finally:
        simulator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 