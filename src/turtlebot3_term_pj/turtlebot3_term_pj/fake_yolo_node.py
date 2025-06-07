import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
from time import sleep

class FakeYoloNode(Node):
    def __init__(self):
        super().__init__('fake_yolo_node')
        # 퍼블리셔 설정
        self.yolo_detect_pub = self.create_publisher(String, '/yolo_detect', 10)
        # 서브스크라이버 설정
        self.trigger_yolo_sub = self.create_subscription(
            Int32, '/trigger_yolo', self.trigger_yolo_callback, 10)
        self.get_logger().info('Fake YOLO Node is ready')

    def trigger_yolo_callback(self, msg):
        point_number = msg.data
        self.get_logger().info(f'Received trigger for point {point_number}')
        self.get_logger().info('Simulating YOLO processing...')
        sleep(2.0)  # 처리 시간 시뮬레이션
        result = String()
        result.data = f'Fake result for point {point_number}: items missing: A, B'
        self.yolo_detect_pub.publish(result)
        self.get_logger().info('YOLO processing completed')

def main(args=None):
    rclpy.init(args=args)
    node = FakeYoloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()