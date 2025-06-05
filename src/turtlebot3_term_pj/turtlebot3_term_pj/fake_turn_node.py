import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from time import sleep
from turtlebot3_term_pj_interfaces.action import TurnToDesk

class MockTurnNode(Node):
    def __init__(self):
        super().__init__('mock_turn_node')
        self._action_server = ActionServer(
            self,
            TurnToDesk,
            'turn_to_desk',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info(f'Received goal with start={goal_handle.request.start}')
        self.get_logger().info('Simulating turning to desk...')
        feedback_msg = TurnToDesk.Feedback()
        for i in range(10):
            feedback_msg.progress = i / 10.0
            goal_handle.publish_feedback(feedback_msg)
            sleep(0.1)  # 0.1초 대기, 총 1초 시뮬레이션
        goal_handle.succeed()
        result = TurnToDesk.Result()
        result.success = True
        self.get_logger().info('Turning completed successfully')
        return result

def main(args=None):
    rclpy.init(args=args)
    node = MockTurnNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()