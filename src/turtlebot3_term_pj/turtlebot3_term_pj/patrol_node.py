import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseArray, PoseStamped
from std_msgs.msg import Empty, String
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from turtlebot3_term_pj_interfaces.action import TurnToDesk
from turtlebot3_term_pj_interfaces.srv import TriggerYolo
from rclpy.action import ActionClient

class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol_node')
        
        # QoS 설정
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
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

        # 퍼블리셔
        self.status_pub = self.create_publisher(String, '/patrol_status', qos)
        self.current_point_pub = self.create_publisher(String, '/current_patrol_point', qos)

        # 서브스크라이버
        self.patrol_points_sub = self.create_subscription(
            PoseArray,
            '/patrol_points',
            self.patrol_points_callback,
            qos
        )
        self.start_patrol_sub = self.create_subscription(
            Empty,
            '/start_patrol',
            self.start_patrol_callback,
            qos
        )
        self.stop_patrol_sub = self.create_subscription(
            Empty,
            '/stop_patrol',
            self.stop_patrol_callback,
            qos
        )

        # 액션 클라이언트 (TurnToDesk)
        self.turn_client = ActionClient(self, TurnToDesk, 'turn_to_desk')

        # 서비스 클라이언트 (TriggerYolo)
        self.yolo_client = self.create_client(TriggerYolo, 'trigger_yolo')

        # 타이머 설정 (0.1초마다 실행)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def patrol_points_callback(self, msg):
        """패트롤 포인트 수신 콜백"""
        self.patrol_points = [self.create_pose_stamped_from_pose(pose) for pose in msg.poses]
        self.get_logger().info(f'수신된 패트롤 포인트 수: {len(self.patrol_points)}')

    def create_pose_stamped_from_pose(self, pose):
        """Pose를 PoseStamped로 변환"""
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'map'
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose = pose
        return pose_stamped

    def start_patrol_callback(self, msg):
        """패트롤 시작 콜백"""
        if not self.patrol_points:
            self.get_logger().warn('패트롤 포인트가 없습니다.')
            return
        self.is_patrol_active = True
        self.current_point_index = 0
        self.get_logger().info('패트롤 시작')
        self.status_pub.publish(String(data='Patrol started'))
        self.send_next_goal()

    def stop_patrol_callback(self, msg):
        """패트롤 중단 콜백"""
        self.is_patrol_active = False
        self.navigator.cancelTask()
        self.get_logger().info('패트롤 중단')
        self.status_pub.publish(String(data='Patrol stopped'))

    def send_next_goal(self):
        """다음 패트롤 지점으로 이동"""
        if not self.is_patrol_active or self.current_point_index >= len(self.patrol_points):
            return
        pose = self.patrol_points[self.current_point_index]
        self.current_point_pub.publish(String(data=f'Point {self.current_point_index + 1}'))
        self.status_pub.publish(String(data=f'Moving to point {self.current_point_index + 1}'))
        self.navigator.goToPose(pose)

    def timer_callback(self):
        """타이머 콜백: 내비게이션 상태 확인"""
        if not self.is_patrol_active:
            return
        if self.navigator.isTaskComplete():
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info(f'포인트 {self.current_point_index + 1}에 도착')
                self.status_pub.publish(String(data=f'Reached point {self.current_point_index + 1}'))
                self.turn_to_desk()
            else:
                self.get_logger().error('네비게이션 실패')
                self.status_pub.publish(String(data='Navigation failed'))
                self.is_patrol_active = False

    def turn_to_desk(self):
        """TurnToDesk 액션 실행"""
        if not self.turn_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('TurnToDesk 서버를 찾을 수 없습니다.')
            return

        goal_msg = TurnToDesk.Goal()
        goal_msg.start = True

        future = self.turn_client.send_goal_async(goal_msg, feedback_callback=self.turn_feedback_callback)
        future.add_done_callback(self.turn_goal_response_callback)

    def turn_feedback_callback(self, feedback):
        """TurnToDesk 피드백 콜백"""
        progress = feedback.feedback.progress
        self.status_pub.publish(String(data=f'Turning, progress: {progress:.2f}'))

    def turn_goal_response_callback(self, future):
        """TurnToDesk 목표 응답 콜백"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('TurnToDesk 목표가 거부되었습니다.')
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.turn_result_callback)

    def turn_result_callback(self, future):
        """TurnToDesk 결과 콜백"""
        result = future.result()
        if result.result.success:
            self.get_logger().info('TurnToDesk 완료')
            self.status_pub.publish(String(data='Turn completed'))
            self.trigger_yolo()
        else:
            self.get_logger().error('TurnToDesk 실패')

    def trigger_yolo(self):
        """yolo_node 서비스 호출"""
        if not self.yolo_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('TriggerYolo 서비스를 찾을 수 없습니다.')
            return

        request = TriggerYolo.Request()
        request.point_number = self.current_point_index + 1

        future = self.yolo_client.call_async(request)
        future.add_done_callback(self.yolo_response_callback)

    def yolo_response_callback(self, future):
        """yolo_node 응답 콜백"""
        try:
            response = future.result()
            self.get_logger().info(f'yolo_node 결과: {response.message}')
            self.status_pub.publish(String(data=f'Yolo result: {response.message}'))
            
            # 다음 포인트로 이동
            self.current_point_index += 1
            if self.current_point_index < len(self.patrol_points):
                self.send_next_goal()
            else:
                self.get_logger().info('모든 패트롤 포인트 완료')
                self.status_pub.publish(String(data='Patrol completed'))
                self.is_patrol_active = False
        except Exception as e:
            self.get_logger().error(f'yolo_node 호출 실패: {str(e)}')

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