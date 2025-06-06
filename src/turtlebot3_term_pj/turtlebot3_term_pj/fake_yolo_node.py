import rclpy
from rclpy.node import Node
from turtlebot3_term_pj_interfaces.srv import TriggerYolo
from time import sleep

class MockYoloNode(Node):
    def __init__(self):
        super().__init__('fake_yolo_node')
        self.srv = self.create_service(TriggerYolo, 'trigger_yolo', self.trigger_yolo_callback)
        self.get_logger().info('Mock YOLO service is ready')

    def trigger_yolo_callback(self, request, response):
        self.get_logger().info(f'Received trigger for point {request.point_number}')
        self.get_logger().info('Simulating YOLO processing...')
        sleep(2.0)  # 2초 대기, 처리 시간 시뮬레이션
        response.message = f'Fake result for point {request.point_number}: items missing: A, B'
        self.get_logger().info('YOLO processing completed')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = MockYoloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()