#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class SimpleTestNode(Node):
    def __init__(self):
        super().__init__('simple_test_node')
        
        # 퍼블리셔 생성
        self.publisher = self.create_publisher(String, '/test_topic', 10)
        
        # 타이머 생성 (1초마다)
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.get_logger().info('간단한 테스트 노드가 시작되었습니다.')
    
    def timer_callback(self):
        """타이머 콜백 함수"""
        msg = String()
        msg.data = f'테스트 메시지: {time.time()}'
        self.publisher.publish(msg)
        self.get_logger().info(f'메시지 발행: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    
    node = SimpleTestNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 