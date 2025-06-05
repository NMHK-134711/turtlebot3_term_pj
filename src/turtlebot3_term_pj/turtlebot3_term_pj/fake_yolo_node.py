import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String
from time import sleep

class MockYoloNode(Node):
    def __init__(self):
        super().__init__('mock_yolo_node')
        self.subscription = self.create_subscription(
            Empty,
            '/trigger_yolo',
            self.trigger_callback,
            10)
        self.publisher = self.create_publisher(String, '/yolo_result', 10)

    def trigger_callback(self, msg):
        self.get_logger().info('Received trigger to perform YOLO detection')
        self.get_logger().info('Simulating YOLO processing...')
        sleep(2.0)  # 2초 대기, 처리 시간 시뮬레이션
        result_msg = String()
        result_msg.data = 'Mock result: items missing: A, B'
        self.publisher.publish(result_msg)
        self.get_logger().info('Published mock YOLO result')

def main(args=None):
    rclpy.init(args=args)
    node = MockYoloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()