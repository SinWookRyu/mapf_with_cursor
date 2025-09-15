import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

class TFListener(Node):
    def __init__(self):
        super().__init__('tf_listener')
        self.subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.listener_callback,
            10)
        self.frames = set()

    def listener_callback(self, msg):
        for transform in msg.transforms:
            parent = transform.header.frame_id
            child = transform.child_frame_id
            if child not in self.frames:
                self.frames.add(child)
                self.get_logger().info(f'New frame detected: {parent} -> {child}')

def main(args=None):
    rclpy.init(args=args)
    node = TFListener()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
