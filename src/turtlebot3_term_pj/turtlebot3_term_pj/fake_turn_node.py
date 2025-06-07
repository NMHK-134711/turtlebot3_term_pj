import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from time import sleep

class FakeTurnNode(Node):
    def __init__(self):
        super().__init__('fake_turn_node')
        # 퍼블리셔 설정
        self.bottle_align_progress_pub = self.create_publisher(Float32, '/bottle_align_progress', 10)
        self.success_bottle_pub = self.create_publisher(Bool, '/success_bottle_align', 10)
        # 서브스크라이버 설정
        self.start_bottle_sub = self.create_subscription(
            Bool, '/start_bottle_align', self.start_bottle_callback, 10)
        # 상태 변수
        self.is_turning = False
        self.get_logger().info('Fake Turn Node is ready')

    def start_bottle_callback(self, msg):
        if msg.data and not self.is_turning:
            self.is_turning = True
            self.get_logger().info('Starting turn simulation')
            # 진행 상황 시뮬레이션 (0.0 ~ 1.0)
            for i in range(10):
                progress = Float32()
                progress.data = i / 10.0
                self.bottle_align_progress_pub.publish(progress)
                sleep(0.1)
            # 완료 신호 발송
            success_msg = Bool()
            success_msg.data = True
            self.success_bottle_pub.publish(success_msg)
            self.get_logger().info('Turn simulation completed')
            self.is_turning = False

def main(args=None):
    rclpy.init(args=args)
    node = FakeTurnNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()