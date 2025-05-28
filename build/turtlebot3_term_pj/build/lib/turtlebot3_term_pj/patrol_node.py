import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty
from tf_transformations import quaternion_from_euler

class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol_node')
        self.navigator = BasicNavigator()
        self.stop_flag = False
        self.sub_stop = self.create_subscription(Empty, '/stop_patrol', self.stop_callback, 10)
        self.pub_take_photo = self.create_publisher(Empty, '/take_photo', 10)
        
        # 좌표와 세타값 목록 (x, y, theta)
        points = [
            (0.0, 0.5, 0.0),    # 예시 좌표
            (0.5, 0.7, 1.57),   # 예시 좌표
            (0.0, 0.2, 0.5),
        ]
        self.points = [self.create_pose_stamped(*p) for p in points]
        self.current_index = 0
        
        # Nav2 활성화 대기
        self.get_logger().info("Waiting for Nav2 to become active...")
        self.navigator.waitUntilNav2Active()
        self.get_logger().info("Nav2 is active. Starting patrol...")
        
        # 첫 번째 목표 전송
        self.send_next_goal()
        
        # 작업 상태 확인을 위한 타이머 생성
        self.timer = self.create_timer(0.1, self.timer_callback)
    
    def create_pose_stamped(self, x, y, theta):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        q = quaternion_from_euler(0, 0, theta)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        return pose
    
    def stop_callback(self, msg):
        self.stop_flag = True
        self.navigator.cancelTask()
        self.get_logger().info("Received stop command. Patrol stopped.")
    
    def send_next_goal(self):
        if self.stop_flag:
            return
        pose = self.points[self.current_index]
        self.navigator.goToPose(pose)
        self.get_logger().info(f"Sending goal to point {self.current_index}: x={pose.pose.position.x}, y={pose.pose.position.y}")
    
    def timer_callback(self):
        if self.stop_flag:
            return
        if self.navigator.isTaskComplete():
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info(f"Reached point {self.current_index}")
                # 사진 촬영 트리거
                self.pub_take_photo.publish(Empty())
                self.get_logger().info("Triggered camera to take photo")
                # 다음 지점으로 이동
                self.current_index = (self.current_index + 1) % len(self.points)
                self.send_next_goal()
            elif result == TaskResult.CANCELED:
                self.get_logger().info("Patrol canceled")
            else:
                self.get_logger().error("Navigation failed")
                self.stop_flag = True

def main(args=None):
    rclpy.init(args=args)
    node = PatrolNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()