#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import numpy as np
import time

class SimpleAgent:
    def __init__(self, agent_id: int, start_pos: tuple, goal_pos: tuple):
        self.id = agent_id
        self.current_pos = np.array(start_pos, dtype=float)
        self.goal_pos = np.array(goal_pos, dtype=float)
        self.velocity = np.array([0.0, 0.0])
        self.max_speed = 1.0
        self.radius = 0.5
        self.is_moving = True
        
        # 색상 설정
        colors = [
            ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),  # 빨강
            ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0),  # 초록
            ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0),  # 파랑
        ]
        self.color = colors[agent_id % len(colors)]
    
    def update_position(self, dt: float):
        """에이전트 위치 업데이트"""
        if not self.is_moving:
            return
            
        # 목표까지의 거리 계산
        to_goal = self.goal_pos - self.current_pos
        distance = np.linalg.norm(to_goal)
        
        if distance < 0.1:  # 목표 도달
            self.is_moving = False
            return
            
        # 방향 벡터 계산
        direction = to_goal / distance if distance > 0 else np.array([0.0, 0.0])
        
        # 속도 업데이트
        self.velocity = direction * self.max_speed
        
        # 위치 업데이트
        self.current_pos += self.velocity * dt

class SimpleMAPFSimulator(Node):
    def __init__(self):
        super().__init__('mapf_simulator')
        
        # 파라미터 설정
        self.declare_parameter('num_agents', 3)
        self.declare_parameter('world_width', 20.0)
        self.declare_parameter('world_height', 20.0)
        self.declare_parameter('update_rate', 10.0)
        
        # 파라미터 가져오기
        self.num_agents = self.get_parameter('num_agents').value
        self.world_width = self.get_parameter('world_width').value
        self.world_height = self.get_parameter('world_height').value
        self.update_rate = self.get_parameter('update_rate').value
        
        # 에이전트 리스트
        self.agents = []
        
        # 퍼블리셔 설정
        self.agent_poses_pub = self.create_publisher(MarkerArray, '/agent_poses', 10)
        
        # 타이머 설정
        self.timer = self.create_timer(1.0 / self.update_rate, self.update_simulation)
        
        # 에이전트 초기화
        self.initialize_agents()
        
        self.get_logger().info(f'MAPF 시뮬레이터가 시작되었습니다. 에이전트 수: {self.num_agents}')
    
    def initialize_agents(self):
        """에이전트 초기화"""
        for i in range(self.num_agents):
            # 시작 위치와 목표 위치 설정
            start_x = -8.0 + i * 2.0
            start_y = -5.0
            goal_x = 8.0 - i * 2.0
            goal_y = 5.0
            
            agent = SimpleAgent(i, (start_x, start_y), (goal_x, goal_y))
            self.agents.append(agent)
    
    def update_simulation(self):
        """시뮬레이션 업데이트"""
        dt = 1.0 / self.update_rate
        
        # 각 에이전트 업데이트
        for agent in self.agents:
            if agent.is_moving:
                agent.update_position(dt)
                
                # 월드 경계 체크
                agent.current_pos[0] = np.clip(agent.current_pos[0], -self.world_width/2, self.world_width/2)
                agent.current_pos[1] = np.clip(agent.current_pos[1], -self.world_height/2, self.world_height/2)
        
        # 시각화 업데이트
        self.publish_agent_poses()
    
    def publish_agent_poses(self):
        """에이전트 위치 시각화"""
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
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            
            # 크기 설정
            marker.scale.x = agent.radius * 2
            marker.scale.y = agent.radius * 2
            marker.scale.z = 0.5
            
            # 색상 설정
            marker.color = agent.color
            
            marker_array.markers.append(marker)
        
        self.agent_poses_pub.publish(marker_array)
        self.get_logger().info(f'에이전트 위치 발행 완료')

def main(args=None):
    rclpy.init(args=args)
    
    simulator = SimpleMAPFSimulator()
    
    try:
        rclpy.spin(simulator)
    except KeyboardInterrupt:
        pass
    finally:
        simulator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 