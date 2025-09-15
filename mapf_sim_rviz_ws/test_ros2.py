#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

def main():
    print("ROS2 테스트 시작...")
    
    try:
        rclpy.init()
        print("rclpy 초기화 성공")
        
        node = Node('test_node')
        print("노드 생성 성공")
        
        publisher = node.create_publisher(String, '/test_topic', 10)
        print("퍼블리셔 생성 성공")
        
        timer = node.create_timer(1.0, lambda: print("타이머 작동 중..."))
        print("타이머 생성 성공")
        
        print("노드 실행 중... (Ctrl+C로 종료)")
        rclpy.spin(node)
        
    except Exception as e:
        print(f"오류 발생: {e}")
    finally:
        rclpy.shutdown()
        print("ROS2 종료")

if __name__ == '__main__':
    main() 