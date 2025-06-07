import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseArray, PoseStamped
from std_msgs.msg import Bool, Int32, String, Empty
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from turtlebot3_term_pj_interfaces.action import Patrol
from rclpy.action import ActionServer

class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol_node')
        
        # QoS 설정
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Patrol 액션 서버
        self._action_server = ActionServer(
            self,
            Patrol,
            'patrol',
            self.execute_callback
        )

        # BasicNavigator 초기화
        self.navigator = BasicNavigator()
        self.get_logger().info("Waiting for Nav2 to become active...")
        self.navigator.waitUntilNav2Active()
        self.get_logger().info("Nav2 is active.")

        # 패트롤 포인트 저장 및 상태 변수
        self.patrol_points = []
        self.current_point_index = 0
        self.is_patrol_active = False
        self.is_processing_point = False
        self.is_turning = False
        self.is_yolo_running = False
        self.current_goal_handle = None  # goal_handle 저장용

        # 퍼블리셔와 서브스크라이버
        self.start_bottle_pub = self.create_publisher(Bool, '/start_bottle_align', qos)
        self.success_bottle_sub = self.create_subscription(
            Bool, '/success_bottle_align', self.success_bottle_callback, qos)
        self.trigger_yolo_pub = self.create_publisher(Int32, '/trigger_yolo', qos)
        self.yolo_detect_sub = self.create_subscription(
            String, '/yolo_detect', self.yolo_detect_callback, qos)
        self.status_pub = self.create_publisher(String, '/patrol_status', qos)
        self.current_point_pub = self.create_publisher(String, '/current_patrol_point', qos)
        # 중단 신호 서브스크라이버 추가
        self.stop_patrol_sub = self.create_subscription(
            Empty, '/stop_patrol', self.stop_patrol_callback, qos)

        # 타이머 설정
        self.timer = self.create_timer(0.1, self.timer_callback)

    def stop_patrol_callback(self, msg):
        """중단 신호 수신 시 패트롤 중단"""
        self.is_patrol_active = False
        self.get_logger().info('Received stop patrol signal')

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing patrol goal...')
        self.patrol_points = goal_handle.request.points.poses
        if not self.patrol_points:
            goal_handle.abort()
            result = Patrol.Result()
            result.success = False
            result.message = 'No patrol points provided'
            return result

        self.is_patrol_active = True
        self.current_point_index = 0
        self.current_goal_handle = goal_handle  # goal_handle 저장
        feedback_msg = Patrol.Feedback()
        feedback_msg.status = 'Patrol started'
        feedback_msg.current_point = ''
        goal_handle.publish_feedback(feedback_msg)
        self.send_next_goal()

        while self.is_patrol_active:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.is_patrol_active = False
                result = Patrol.Result()
                result.success = False
                result.message = 'Patrol canceled'
                return result
            rclpy.spin_once(self, timeout_sec=0.1)
            # 패트롤 포인트 끝에 도달하면 인덱스 리셋
            if self.current_point_index >= len(self.patrol_points):
                self.current_point_index = 0

        if not self.is_patrol_active:
            goal_handle.abort()
            result = Patrol.Result()
            result.success = False
            result.message = 'Patrol stopped'
        else:
            goal_handle.succeed()
            result = Patrol.Result()
            result.success = True
            result.message = 'Patrol completed'
        self.current_goal_handle = None
        return result

    def create_pose_stamped_from_pose(self, pose):
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'map'
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose = pose
        return pose_stamped

    def send_next_goal(self):
        if not self.is_patrol_active:
            return
        # 인덱스를 순환하도록 설정
        self.current_point_index = self.current_point_index % len(self.patrol_points)
        pose = self.create_pose_stamped_from_pose(self.patrol_points[self.current_point_index])
        feedback_msg = Patrol.Feedback()
        feedback_msg.current_point = f'Point {self.current_point_index + 1}'
        feedback_msg.status = f'Moving to point {self.current_point_index + 1}'
        self.current_goal_handle.publish_feedback(feedback_msg)
        self.current_point_pub.publish(String(data=f'Point {self.current_point_index + 1}'))
        self.status_pub.publish(String(data=f'Moving to point {self.current_point_index + 1}'))
        self.navigator.goToPose(pose)

    def timer_callback(self):
        if not self.is_patrol_active or self.is_processing_point:
            return
        if self.navigator.isTaskComplete():
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info(f'포인트 {self.current_point_index + 1}에 도착')
                feedback_msg = Patrol.Feedback()
                feedback_msg.status = f'Reached point {self.current_point_index + 1}'
                self.current_goal_handle.publish_feedback(feedback_msg)
                self.status_pub.publish(String(data=f'Reached point {self.current_point_index + 1}'))
                self.is_processing_point = True
                self.turn_to_desk()
            else:
                self.get_logger().error('네비게이션 실패')
                feedback_msg = Patrol.Feedback()
                feedback_msg.status = 'Navigation failed'
                self.current_goal_handle.publish_feedback(feedback_msg)
                self.status_pub.publish(String(data='Navigation failed'))
                self.is_patrol_active = False

    def turn_to_desk(self):
        if not self.is_turning:
            self.is_turning = True
            msg = Bool()
            msg.data = True
            self.start_bottle_pub.publish(msg)
            self.get_logger().info('Sent start signal to bottle_align_node')

    def success_bottle_callback(self, msg):
        if msg.data and self.is_turning:
            self.is_turning = False
            self.get_logger().info('Received success from bottle_align_node')
            self.trigger_yolo()

    def trigger_yolo(self):
        if not self.is_yolo_running:
            self.is_yolo_running = True
            point_number = Int32()
            point_number.data = self.current_point_index + 1
            self.trigger_yolo_pub.publish(point_number)
            self.get_logger().info(f'Sent trigger to yolo_node with point {point_number.data}')

    def yolo_detect_callback(self, msg):
        if self.is_yolo_running:
            self.is_yolo_running = False
            self.get_logger().info(f'Yolo result: {msg.data}')
            feedback_msg = Patrol.Feedback()
            feedback_msg.status = f'Yolo result: {msg.data}'
            self.current_goal_handle.publish_feedback(feedback_msg)
            self.status_pub.publish(String(data=f'Yolo result: {msg.data}'))
            
            if self.is_patrol_active:
                self.current_point_index += 1
                self.is_processing_point = False
                self.send_next_goal()
            else:
                self.get_logger().info('패트롤 중단됨')
                feedback_msg = Patrol.Feedback()
                feedback_msg.status = 'Patrol stopped'
                self.current_goal_handle.publish_feedback(feedback_msg)
                self.status_pub.publish(String(data='Patrol stopped'))
                self.is_processing_point = False

def main(args=None):
    rclpy.init(args=args)
    patrol_node = PatrolNode()
    try:
        rclpy.spin(patrol_node)
    except KeyboardInterrupt:
        pass
    finally:
        patrol_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()