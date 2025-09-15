#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32, Bool
from geometry_msgs.msg import Point
import tkinter as tk
from tkinter import ttk, messagebox
import threading
import numpy as np

class GUIControlNode(Node):
    def __init__(self):
        super().__init__('gui_control_node')
        
        # 파라미터 퍼블리셔
        self.num_agents_pub = self.create_publisher(Int32, '/num_agents', 10)
        self.world_width_pub = self.create_publisher(Float32, '/world_width', 10)
        self.world_height_pub = self.create_publisher(Float32, '/world_height', 10)
        self.collision_distance_pub = self.create_publisher(Float32, '/collision_distance', 10)
        self.update_rate_pub = self.create_publisher(Float32, '/update_rate', 10)
        self.reset_sim_pub = self.create_publisher(Bool, '/reset_simulation', 10)
        
        # 충돌 영역 추가 퍼블리셔
        self.add_collision_zone_pub = self.create_publisher(Point, '/add_collision_zone', 10)
        self.clear_collision_zones_pub = self.create_publisher(Bool, '/clear_collision_zones', 10)
        
        # 충돌 회피 파라미터 퍼블리셔들
        self.avoidance_strength_pub = self.create_publisher(Float32, '/avoidance_strength', 10)
        self.prediction_time_pub = self.create_publisher(Float32, '/prediction_time', 10)
        self.repulsion_power_pub = self.create_publisher(Float32, '/repulsion_power', 10)
        
        # 움직임 모드 및 격자 설정 퍼블리셔들
        self.movement_mode_pub = self.create_publisher(Int32, '/movement_mode', 10)  # 0: continuous, 1: grid
        self.grid_size_pub = self.create_publisher(Point, '/grid_size', 10)  # x, y: grid counts, z: unused
        
        # 이동 속도 조정 퍼블리셔
        self.grid_move_interval_pub = self.create_publisher(Float32, '/grid_move_interval', 10)
        
        # 에이전트 위치 설정 퍼블리셔
        self.agent_positions_pub = self.create_publisher(Point, '/agent_positions', 10)
        
        # 시뮬레이션 시작 제어 퍼블리셔
        self.start_simulation_pub = self.create_publisher(Bool, '/start_simulation', 10)
        
        # GUI 스레드 시작
        self.gui_thread = threading.Thread(target=self.create_gui)
        self.gui_thread.daemon = True
        self.gui_thread.start()
        
        self.get_logger().info('GUI 컨트롤 노드가 시작되었습니다.')
    
    def create_gui(self):
        """GUI 생성"""
        self.root = tk.Tk()
        self.root.title("MAPF 시뮬레이터 제어판")
        self.root.geometry("500x700")
        
        # 노트북 (탭) 생성
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # 메인 제어 탭
        main_tab = ttk.Frame(self.notebook, padding="10")
        self.notebook.add(main_tab, text="메인 제어")
        
        # 에이전트 설정 섹션
        agent_frame = ttk.LabelFrame(main_tab, text="에이전트 설정", padding="5")
        agent_frame.grid(row=0, column=0, sticky=(tk.W, tk.E), pady=5)
        
        ttk.Label(agent_frame, text="에이전트 수:").grid(row=0, column=0, sticky=tk.W)
        self.num_agents_var = tk.IntVar(value=3)
        num_agents_spin = ttk.Spinbox(agent_frame, from_=1, to=10, textvariable=self.num_agents_var, width=10)
        num_agents_spin.grid(row=0, column=1, padx=5)
        ttk.Button(agent_frame, text="적용", command=self.update_num_agents).grid(row=0, column=2, padx=5)
        
        # 움직임 모드 설정 섹션
        movement_frame = ttk.LabelFrame(main_tab, text="움직임 모드 설정", padding="5")
        movement_frame.grid(row=1, column=0, sticky=(tk.W, tk.E), pady=5)
        
        ttk.Label(movement_frame, text="움직임 모드:").grid(row=0, column=0, sticky=tk.W)
        self.movement_mode_var = tk.StringVar(value="continuous")
        movement_mode_combo = ttk.Combobox(movement_frame, textvariable=self.movement_mode_var, 
                                          values=["continuous", "grid"], state="readonly", width=15)
        movement_mode_combo.grid(row=0, column=1, padx=5)
        ttk.Button(movement_frame, text="적용", command=self.update_movement_mode).grid(row=0, column=2, padx=5)
        
        ttk.Label(movement_frame, text="격자 수 (X):").grid(row=1, column=0, sticky=tk.W)
        self.grid_x_var = tk.IntVar(value=20)
        grid_x_spin = ttk.Spinbox(movement_frame, from_=5, to=100, textvariable=self.grid_x_var, width=10)
        grid_x_spin.grid(row=1, column=1, padx=5)
        ttk.Button(movement_frame, text="적용", command=self.update_grid_size).grid(row=1, column=2, padx=5)
        
        ttk.Label(movement_frame, text="격자 수 (Y):").grid(row=2, column=0, sticky=tk.W)
        self.grid_y_var = tk.IntVar(value=20)
        grid_y_spin = ttk.Spinbox(movement_frame, from_=5, to=100, textvariable=self.grid_y_var, width=10)
        grid_y_spin.grid(row=2, column=1, padx=5)
        
        ttk.Label(movement_frame, text="이동 속도 (초):").grid(row=3, column=0, sticky=tk.W)
        self.grid_move_interval_var = tk.DoubleVar(value=0.8)
        grid_move_interval_spin = ttk.Spinbox(movement_frame, from_=0.1, to=3.0, increment=0.1, 
                                             textvariable=self.grid_move_interval_var, width=10)
        grid_move_interval_spin.grid(row=3, column=1, padx=5)
        ttk.Button(movement_frame, text="적용", command=self.update_grid_move_interval).grid(row=3, column=2, padx=5)
        
        # 월드 설정 섹션
        world_frame = ttk.LabelFrame(main_tab, text="월드 설정", padding="5")
        world_frame.grid(row=2, column=0, sticky=(tk.W, tk.E), pady=5)
        
        ttk.Label(world_frame, text="월드 너비:").grid(row=0, column=0, sticky=tk.W)
        self.world_width_var = tk.DoubleVar(value=20.0)
        world_width_spin = ttk.Spinbox(world_frame, from_=10.0, to=50.0, increment=1.0, 
                                      textvariable=self.world_width_var, width=10)
        world_width_spin.grid(row=0, column=1, padx=5)
        ttk.Button(world_frame, text="적용", command=self.update_world_size).grid(row=0, column=2, padx=5)
        
        ttk.Label(world_frame, text="월드 높이:").grid(row=1, column=0, sticky=tk.W)
        self.world_height_var = tk.DoubleVar(value=20.0)
        world_height_spin = ttk.Spinbox(world_frame, from_=10.0, to=50.0, increment=1.0, 
                                       textvariable=self.world_height_var, width=10)
        world_height_spin.grid(row=1, column=1, padx=5)
        
        # 시뮬레이션 설정 섹션
        sim_frame = ttk.LabelFrame(main_tab, text="시뮬레이션 설정", padding="5")
        sim_frame.grid(row=3, column=0, sticky=(tk.W, tk.E), pady=5)
        
        ttk.Label(sim_frame, text="업데이트 속도 (Hz):").grid(row=0, column=0, sticky=tk.W)
        self.update_rate_var = tk.DoubleVar(value=10.0)
        update_rate_spin = ttk.Spinbox(sim_frame, from_=1.0, to=30.0, increment=1.0, 
                                      textvariable=self.update_rate_var, width=10)
        update_rate_spin.grid(row=0, column=1, padx=5)
        ttk.Button(sim_frame, text="적용", command=self.update_rate).grid(row=0, column=2, padx=5)
        
        # 제어 버튼 섹션
        control_frame = ttk.LabelFrame(main_tab, text="시뮬레이션 제어", padding="5")
        control_frame.grid(row=4, column=0, sticky=(tk.W, tk.E), pady=5)
        
        # 시뮬레이션 시작/중지 버튼
        self.start_button = ttk.Button(control_frame, text="시뮬레이션 시작", command=self.start_simulation)
        self.start_button.grid(row=0, column=0, columnspan=2, pady=5)
        
        ttk.Button(control_frame, text="시뮬레이션 리셋", command=self.reset_simulation).grid(row=1, column=0, columnspan=2, pady=5)
        
        # 상태 표시 섹션
        status_frame = ttk.LabelFrame(main_tab, text="상태 정보", padding="5")
        status_frame.grid(row=5, column=0, sticky=(tk.W, tk.E), pady=5)
        
        self.status_text = tk.Text(status_frame, height=8, width=45)
        self.status_text.grid(row=0, column=0, pady=5)
        
        # 스크롤바 추가
        scrollbar = ttk.Scrollbar(status_frame, orient="vertical", command=self.status_text.yview)
        scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S))
        self.status_text.configure(yscrollcommand=scrollbar.set)
        
        # 에이전트 위치 설정 탭
        position_tab = ttk.Frame(self.notebook, padding="10")
        self.notebook.add(position_tab, text="에이전트 위치")
        
        # 시작 위치 설정 섹션
        start_frame = ttk.LabelFrame(position_tab, text="시작 위치 설정", padding="5")
        start_frame.grid(row=0, column=0, sticky=(tk.W, tk.E), pady=5)
        
        # 에이전트별 시작 위치 입력
        self.start_positions = []
        for i in range(3):  # 기본 3개 에이전트
            agent_start_frame = ttk.Frame(start_frame)
            agent_start_frame.grid(row=i, column=0, sticky=(tk.W, tk.E), pady=2)
            
            ttk.Label(agent_start_frame, text=f"에이전트 {i}:").grid(row=0, column=0, sticky=tk.W)
            ttk.Label(agent_start_frame, text="X:").grid(row=0, column=1, padx=(10, 2))
            start_x_var = tk.DoubleVar(value=-5.0 + i * 2.0)
            start_x_spin = ttk.Spinbox(agent_start_frame, from_=-10.0, to=10.0, increment=0.5, 
                                      textvariable=start_x_var, width=8)
            start_x_spin.grid(row=0, column=2, padx=2)
            
            ttk.Label(agent_start_frame, text="Y:").grid(row=0, column=3, padx=(10, 2))
            start_y_var = tk.DoubleVar(value=-3.0)
            start_y_spin = ttk.Spinbox(agent_start_frame, from_=-10.0, to=10.0, increment=0.5, 
                                      textvariable=start_y_var, width=8)
            start_y_spin.grid(row=0, column=4, padx=2)
            
            self.start_positions.append((start_x_var, start_y_var))
        
        # 목표 위치 설정 섹션
        goal_frame = ttk.LabelFrame(position_tab, text="목표 위치 설정", padding="5")
        goal_frame.grid(row=1, column=0, sticky=(tk.W, tk.E), pady=5)
        
        # 에이전트별 목표 위치 입력
        self.goal_positions = []
        for i in range(3):  # 기본 3개 에이전트
            agent_goal_frame = ttk.Frame(goal_frame)
            agent_goal_frame.grid(row=i, column=0, sticky=(tk.W, tk.E), pady=2)
            
            ttk.Label(agent_goal_frame, text=f"에이전트 {i}:").grid(row=0, column=0, sticky=tk.W)
            ttk.Label(agent_goal_frame, text="X:").grid(row=0, column=1, padx=(10, 2))
            goal_x_var = tk.DoubleVar(value=5.0 - i * 2.0)
            goal_x_spin = ttk.Spinbox(agent_goal_frame, from_=-10.0, to=10.0, increment=0.5, 
                                     textvariable=goal_x_var, width=8)
            goal_x_spin.grid(row=0, column=2, padx=2)
            
            ttk.Label(agent_goal_frame, text="Y:").grid(row=0, column=3, padx=(10, 2))
            goal_y_var = tk.DoubleVar(value=3.0)
            goal_y_spin = ttk.Spinbox(agent_goal_frame, from_=-10.0, to=10.0, increment=0.5, 
                                     textvariable=goal_y_var, width=8)
            goal_y_spin.grid(row=0, column=4, padx=2)
            
            self.goal_positions.append((goal_x_var, goal_y_var))
        
        # 위치 적용 버튼
        ttk.Button(position_tab, text="위치 설정 적용", command=self.update_agent_positions).grid(row=2, column=0, pady=10)
        
        # 충돌 설정 탭
        collision_tab = ttk.Frame(self.notebook, padding="10")
        self.notebook.add(collision_tab, text="충돌 설정")
        
        # 충돌 기본 설정 섹션
        collision_basic_frame = ttk.LabelFrame(collision_tab, text="충돌 기본 설정", padding="5")
        collision_basic_frame.grid(row=0, column=0, sticky=(tk.W, tk.E), pady=5)
        
        ttk.Label(collision_basic_frame, text="충돌 거리:").grid(row=0, column=0, sticky=tk.W)
        self.collision_distance_var = tk.DoubleVar(value=1.5)
        collision_distance_spin = ttk.Spinbox(collision_basic_frame, from_=0.5, to=5.0, increment=0.1, 
                                            textvariable=self.collision_distance_var, width=10)
        collision_distance_spin.grid(row=0, column=1, padx=5)
        ttk.Button(collision_basic_frame, text="적용", command=self.update_collision_distance).grid(row=0, column=2, padx=5)
        
        # 충돌 회피 파라미터 섹션
        avoidance_frame = ttk.LabelFrame(collision_tab, text="충돌 회피 파라미터", padding="5")
        avoidance_frame.grid(row=1, column=0, sticky=(tk.W, tk.E), pady=5)
        
        ttk.Label(avoidance_frame, text="회피 강도:").grid(row=0, column=0, sticky=tk.W)
        self.avoidance_strength_var = tk.DoubleVar(value=2.0)
        avoidance_strength_spin = ttk.Spinbox(avoidance_frame, from_=0.1, to=10.0, increment=0.1, 
                                             textvariable=self.avoidance_strength_var, width=10)
        avoidance_strength_spin.grid(row=0, column=1, padx=5)
        ttk.Button(avoidance_frame, text="적용", command=self.update_avoidance_strength).grid(row=0, column=2, padx=5)
        
        ttk.Label(avoidance_frame, text="예측 시간 (초):").grid(row=1, column=0, sticky=tk.W)
        self.prediction_time_var = tk.DoubleVar(value=0.5)
        prediction_time_spin = ttk.Spinbox(avoidance_frame, from_=0.1, to=2.0, increment=0.1, 
                                          textvariable=self.prediction_time_var, width=10)
        prediction_time_spin.grid(row=1, column=1, padx=5)
        ttk.Button(avoidance_frame, text="적용", command=self.update_prediction_time).grid(row=1, column=2, padx=5)
        
        ttk.Label(avoidance_frame, text="반발력 지수:").grid(row=2, column=0, sticky=tk.W)
        self.repulsion_power_var = tk.DoubleVar(value=0.5)
        repulsion_power_spin = ttk.Spinbox(avoidance_frame, from_=0.1, to=3.0, increment=0.1, 
                                          textvariable=self.repulsion_power_var, width=10)
        repulsion_power_spin.grid(row=2, column=1, padx=5)
        ttk.Button(avoidance_frame, text="적용", command=self.update_repulsion_power).grid(row=2, column=2, padx=5)
        
        # 충돌 영역 관리 섹션
        zone_frame = ttk.LabelFrame(collision_tab, text="충돌 영역 관리", padding="5")
        zone_frame.grid(row=2, column=0, sticky=(tk.W, tk.E), pady=5)
        
        ttk.Label(zone_frame, text="X 위치:").grid(row=0, column=0, sticky=tk.W)
        self.zone_x_var = tk.DoubleVar(value=0.0)
        zone_x_spin = ttk.Spinbox(zone_frame, from_=-10.0, to=10.0, increment=0.5, 
                                  textvariable=self.zone_x_var, width=10)
        zone_x_spin.grid(row=0, column=1, padx=5)
        
        ttk.Label(zone_frame, text="Y 위치:").grid(row=1, column=0, sticky=tk.W)
        self.zone_y_var = tk.DoubleVar(value=0.0)
        zone_y_spin = ttk.Spinbox(zone_frame, from_=-10.0, to=10.0, increment=0.5, 
                                  textvariable=self.zone_y_var, width=10)
        zone_y_spin.grid(row=1, column=1, padx=5)
        
        ttk.Label(zone_frame, text="반지름:").grid(row=2, column=0, sticky=tk.W)
        self.zone_radius_var = tk.DoubleVar(value=2.0)
        zone_radius_spin = ttk.Spinbox(zone_frame, from_=0.5, to=10.0, increment=0.5, 
                                       textvariable=self.zone_radius_var, width=10)
        zone_radius_spin.grid(row=2, column=1, padx=5)
        
        ttk.Button(zone_frame, text="충돌 영역 추가", command=self.add_collision_zone).grid(row=3, column=0, columnspan=2, pady=5)
        ttk.Button(zone_frame, text="충돌 영역 모두 제거", command=self.clear_collision_zones).grid(row=4, column=0, columnspan=2, pady=5)
        
        # 초기 상태 메시지
        self.log_status("GUI가 시작되었습니다.")
        self.log_status("시뮬레이터를 시작하려면 'ros2 run mapf_simulator mapf_simulator_node'를 실행하세요.")
        
        # GUI 메인 루프
        self.root.mainloop()
    
    def log_status(self, message: str):
        """상태 로그 추가"""
        self.status_text.insert(tk.END, f"{message}\n")
        self.status_text.see(tk.END)
    
    def update_num_agents(self):
        """에이전트 수 업데이트"""
        num_agents = self.num_agents_var.get()
        msg = Int32()
        msg.data = num_agents
        self.num_agents_pub.publish(msg)
        self.log_status(f"에이전트 수를 {num_agents}로 설정했습니다.")
    
    def update_world_size(self):
        """월드 크기 업데이트"""
        width = self.world_width_var.get()
        height = self.world_height_var.get()
        
        width_msg = Float32()
        width_msg.data = width
        self.world_width_pub.publish(width_msg)
        
        height_msg = Float32()
        height_msg.data = height
        self.world_height_pub.publish(height_msg)
        
        self.log_status(f"월드 크기를 {width}x{height}로 설정했습니다.")
    
    def update_collision_distance(self):
        """충돌 거리 업데이트"""
        distance = self.collision_distance_var.get()
        msg = Float32()
        msg.data = distance
        self.collision_distance_pub.publish(msg)
        self.log_status(f"충돌 거리를 {distance}로 설정했습니다.")
    
    def update_rate(self):
        """업데이트 속도 변경"""
        rate = self.update_rate_var.get()
        msg = Float32()
        msg.data = rate
        self.update_rate_pub.publish(msg)
        self.log_status(f"업데이트 속도를 {rate}Hz로 설정했습니다.")
    
    def update_avoidance_strength(self):
        """회피 강도 업데이트"""
        strength = self.avoidance_strength_var.get()
        msg = Float32()
        msg.data = strength
        self.avoidance_strength_pub.publish(msg)
        self.log_status(f"회피 강도를 {strength}로 설정했습니다.")
    
    def update_prediction_time(self):
        """예측 시간 업데이트"""
        time = self.prediction_time_var.get()
        msg = Float32()
        msg.data = time
        self.prediction_time_pub.publish(msg)
        self.log_status(f"예측 시간을 {time}초로 설정했습니다.")
    
    def update_repulsion_power(self):
        """반발력 지수 업데이트"""
        power = self.repulsion_power_var.get()
        msg = Float32()
        msg.data = power
        self.repulsion_power_pub.publish(msg)
        self.log_status(f"반발력 지수를 {power}로 설정했습니다.")
    
    def update_movement_mode(self):
        """움직임 모드 업데이트"""
        mode = self.movement_mode_var.get()
        msg = Int32()
        msg.data = 1 if mode == "grid" else 0  # 0: continuous, 1: grid
        self.movement_mode_pub.publish(msg)
        self.log_status(f"움직임 모드를 '{mode}'로 설정했습니다.")
    
    def update_grid_size(self):
        """격자 크기 업데이트"""
        grid_x = self.grid_x_var.get()
        grid_y = self.grid_y_var.get()
        msg = Point()
        msg.x = float(grid_x)
        msg.y = float(grid_y)
        msg.z = 0.0  # 사용하지 않음
        self.grid_size_pub.publish(msg)
        self.log_status(f"격자 크기를 {grid_x}x{grid_y}로 설정했습니다.")
    
    def update_agent_positions(self):
        """에이전트 위치 업데이트"""
        # 현재 에이전트 수에 맞춰 위치 정보 수집
        num_agents = min(len(self.start_positions), len(self.goal_positions))
        
        for i in range(num_agents):
            start_x, start_y = self.start_positions[i]
            goal_x, goal_y = self.goal_positions[i]
            
            # 시작 위치와 목표 위치를 Point 메시지로 발행
            start_msg = Point()
            start_msg.x = start_x.get()
            start_msg.y = start_y.get()
            start_msg.z = float(i)  # 에이전트 ID를 z에 저장
            
            goal_msg = Point()
            goal_msg.x = goal_x.get()
            goal_msg.y = goal_y.get()
            goal_msg.z = float(i)  # 에이전트 ID를 z에 저장
            
            # 시작 위치와 목표 위치를 순차적으로 발행
            self.agent_positions_pub.publish(start_msg)
            self.agent_positions_pub.publish(goal_msg)
        
        self.log_status(f"{num_agents}개 에이전트의 위치를 업데이트했습니다.")
    
    def start_simulation(self):
        """시뮬레이션 시작/중지 토글"""
        if self.start_button.cget("text") == "시뮬레이션 시작":
            # 시뮬레이션 시작
            msg = Bool()
            msg.data = True
            self.start_simulation_pub.publish(msg)
            self.start_button.configure(text="시뮬레이션 중지")
            self.log_status("시뮬레이션을 시작했습니다.")
        else:
            # 시뮬레이션 중지
            msg = Bool()
            msg.data = False
            self.start_simulation_pub.publish(msg)
            self.start_button.configure(text="시뮬레이션 시작")
            self.log_status("시뮬레이션을 중지했습니다.")
    
    def update_grid_move_interval(self):
        """그리드 이동 간격 업데이트"""
        interval = self.grid_move_interval_var.get()
        msg = Float32()
        msg.data = interval
        self.grid_move_interval_pub.publish(msg)
        self.log_status(f"그리드 이동 간격을 {interval}초로 설정했습니다.")
    
    def add_collision_zone(self):
        """충돌 영역 추가"""
        x = self.zone_x_var.get()
        y = self.zone_y_var.get()
        radius = self.zone_radius_var.get()
        
        msg = Point()
        msg.x = x
        msg.y = y
        msg.z = radius  # z 필드를 반지름으로 사용
        
        self.add_collision_zone_pub.publish(msg)
        self.log_status(f"충돌 영역을 추가했습니다: 위치({x}, {y}), 반지름 {radius}")
    
    def clear_collision_zones(self):
        """충돌 영역 모두 제거"""
        msg = Bool()
        msg.data = True
        self.clear_collision_zones_pub.publish(msg)
        self.log_status("모든 충돌 영역을 제거했습니다.")
    
    def reset_simulation(self):
        """시뮬레이션 리셋"""
        msg = Bool()
        msg.data = True
        self.reset_sim_pub.publish(msg)
        self.log_status("시뮬레이션을 리셋했습니다.")

def main(args=None):
    rclpy.init(args=args)
    
    gui_node = GUIControlNode()
    
    try:
        rclpy.spin(gui_node)
    except KeyboardInterrupt:
        pass
    finally:
        gui_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 