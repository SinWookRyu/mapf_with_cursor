#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import time

class TestAgentVisualization(Node):
    def __init__(self):
        super().__init__('test_agent_visualization')
        
        # 퍼블리셔 설정
        self.agent_poses_pub = self.create_publisher(MarkerArray, '/agent_poses', 10)
        
        # 타이머 설정 (1Hz로 발행)
        self.timer = self.create_timer(1.0, self.publish_test_agents)
        
        self.get_logger().info('테스트 에이전트 시각화 노드가 시작되었습니다.')
    
    def publish_test_agents(self):
        """테스트 에이전트 마커 발행"""
        marker_array = MarkerArray()
        
        # 3개의 테스트 에이전트 생성
        positions = [(-5.0, -3.0), (-3.0, -3.0), (-1.0, -3.0)]
        colors = [
            ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),  # 빨강
            ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0),  # 초록
            ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0),  # 파랑
        ]
        
        for i, (x, y) in enumerate(positions):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = f"agent_{i}"
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            # 위치 설정
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.5
            marker.pose.orientation.w = 1.0
            
            # 크기 설정
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 0.5
            
            # 색상 설정
            marker.color = colors[i]
            
            marker_array.markers.append(marker)
        
        # 마커 발행
        self.agent_poses_pub.publish(marker_array)
        self.get_logger().info(f'테스트 에이전트 마커 {len(marker_array.markers)}개를 발행했습니다.')

def main(args=None):
    rclpy.init(args=args)
    
    test_node = TestAgentVisualization()
    
    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        test_node.get_logger().info('테스트 노드를 종료합니다.')
    finally:
        test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
