import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, PoseArray
from std_msgs.msg import Empty
from tf_transformations import quaternion_from_euler
import time

class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol_node')
        self.navigator = BasicNavigator()
        self.stop_flag = False
        self.patrolling = False
        self.points = []
        self.current_index = 0
        self.waiting = False
        self.wait_start_time = 0.0
        
        self.sub_stop = self.create_subscription(Empty, '/stop_patrol', self.stop_callback, 10)
        self.sub_patrol_points = self.create_subscription(PoseArray, '/patrol_points', self.patrol_points_callback, 10)
        self.pub_take_photo = self.create_publisher(Empty, '/take_photo', 10)
        
        self.get_logger().info("Waiting for Nav2 to become active...")
        self.navigator.waitUntilNav2Active()
        self.get_logger().info("Nav2 is active. Waiting for patrol points...")
        
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
    
    def create_pose_stamped_from_pose(self, pose):
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'map'
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose = pose
        return pose_stamped
    
    def stop_callback(self, msg):
        self.stop_flag = True
        self.patrolling = False
        self.waiting = False
        self.navigator.cancelTask()
        self.get_logger().info("Received stop command. Patrol stopped.")
    
    def patrol_points_callback(self, msg):
        self.points = [self.create_pose_stamped_from_pose(pose) for pose in msg.poses]
        self.current_index = 0
        self.patrolling = True
        self.stop_flag = False
        self.waiting = False
        self.get_logger().info(f"Received {len(self.points)} patrol points. Starting patrol.")
        self.send_next_goal()
    
    def send_next_goal(self):
        if not self.patrolling or self.stop_flag or not self.points:
            return
        pose = self.points[self.current_index]
        self.navigator.goToPose(pose)
        self.get_logger().info(f"Sending goal to point {self.current_index}: x={pose.pose.position.x}, y={pose.pose.position.y}")
    
    def timer_callback(self):
        if not self.patrolling or self.stop_flag:
            return
        if self.waiting:
            current_time = time.time()
            if current_time - self.wait_start_time >= 2.0:
                self.waiting = False
                self.current_index = (self.current_index + 1) % len(self.points)
                self.send_next_goal()
        else:
            if self.navigator.isTaskComplete():
                result = self.navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info(f"Reached point {self.current_index}")
                    self.pub_take_photo.publish(Empty())
                    self.get_logger().info("Triggered camera to take photo")
                    self.waiting = True
                    self.wait_start_time = time.time()
                elif result == TaskResult.CANCELED:
                    self.get_logger().info("Patrol canceled")
                    self.patrolling = False
                else:
                    self.get_logger().error("Navigation failed")
                    self.patrolling = False

def main(args=None):
    rclpy.init(args=args)
    node = PatrolNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()