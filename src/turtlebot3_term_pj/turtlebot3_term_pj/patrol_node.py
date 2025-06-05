import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Empty, String
from turtlebot3_term_pj_interfaces.action import TurnToDesk

class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol_node')
        # Navigation action client (existing)
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # Turn action client (new)
        self.turn_client = ActionClient(self, TurnToDesk, 'turn_to_desk')
        # Publishers and subscribers
        self.sub_stop = self.create_subscription(Empty, '/stop_patrol', self.stop_callback, 10)
        self.sub_patrol_points = self.create_subscription(PoseArray, '/patrol_points', self.patrol_points_callback, 10)
        self.status_pub = self.create_publisher(String, '/patrol_status', 10)
        self.trigger_yolo_pub = self.create_publisher(Empty, '/trigger_yolo', 10)  # New: Trigger yolo_node
        
        # State variables
        self.patrolling = False
        self.stop_flag = False
        self.points = []
        self.current_index = 0
        
        self.get_logger().info("Patrol node started. Waiting for patrol points...")

    def create_pose_stamped_from_pose(self, pose):
        """Convert Pose to PoseStamped (existing method)"""
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'map'
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose = pose
        return pose_stamped

    def stop_callback(self, msg):
        """Handle stop command (existing method)"""
        self.stop_flag = True
        self.patrolling = False
        self.get_logger().info("Patrol stopped by user")

    def patrol_points_callback(self, msg):
        """Receive patrol points (existing method)"""
        self.points = msg.poses
        self.current_index = 0
        self.patrolling = True
        self.get_logger().info(f"Received {len(self.points)} patrol points")
        self.send_next_goal()

    def send_next_goal(self):
        """Send next navigation goal (existing method)"""
        if self.stop_flag or not self.patrolling:
            return
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.create_pose_stamped_from_pose(self.points[self.current_index])
        self.action_client.wait_for_server()
        self._send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle navigation goal response (existing method)"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            self.patrolling = False
            return
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """Log navigation feedback (existing method)"""
        self.get_logger().info(f"Feedback: {feedback_msg.feedback.distance_remaining:.2f}m remaining")

    def get_result_callback(self, future):
        """Handle navigation result"""
        result = future.result().result
        status_msg = String()
        if result.code == 0:  # Success
            status_msg.data = f"Reached point {self.current_index}"
            self.get_logger().info(status_msg.data)
            self.send_turn_goal()  # Trigger turn_node instead of waiting
        else:
            status_msg.data = f"Navigation failed for point {self.current_index}"
            self.get_logger().error(status_msg.data)
            self.patrolling = False
        self.status_pub.publish(status_msg)

    def send_turn_goal(self):
        """Send goal to turn_node"""
        goal_msg = TurnToDesk.Goal()  # No parameters needed
        self.turn_client.wait_for_server()
        self._turn_future = self.turn_client.send_goal_async(goal_msg, feedback_callback=self.turn_feedback_callback)
        self._turn_future.add_done_callback(self.turn_goal_response_callback)

    def turn_goal_response_callback(self, future):
        """Handle turn_node goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Turn goal rejected")
            self.proceed_to_next_point()  # Proceed even if rejected
            return
        self._turn_result_future = goal_handle.get_result_async()
        self._turn_result_future.add_done_callback(self.turn_result_callback)

    def turn_feedback_callback(self, feedback_msg):
        """Log turn_node feedback"""
        self.get_logger().info(f"Turn progress: {feedback_msg.feedback.progress:.2f}")

    def turn_result_callback(self, future):
        """Handle turn_node result"""
        result = future.result().result
        if result.success:
            self.get_logger().info("Turn successful")
            self.trigger_yolo_pub.publish(Empty())  # Trigger yolo_node
            self.get_logger().info("Triggered yolo_node")
        else:
            self.get_logger().error("Turn failed")
        self.proceed_to_next_point()  # Move to next point regardless

    def proceed_to_next_point(self):
        """Proceed to the next patrol point"""
        self.current_index += 1
        if self.current_index < len(self.points):
            self.send_next_goal()
        else:
            self.patrolling = False
            self.get_logger().info("Patrol completed")

def main(args=None):
    rclpy.init(args=args)
    node = PatrolNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()