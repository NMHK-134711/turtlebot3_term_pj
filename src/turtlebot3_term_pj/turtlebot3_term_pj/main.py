import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QGraphicsScene, QLabel
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt5.QtGui import QImage, QPixmap
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, PoseArray, Pose, PoseWithCovarianceStamped
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32, String
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from turtlebot3_term_pj_interfaces.action import Patrol
from turtlebot3_term_pj_interfaces.srv import StopPatrol
from .ui_main import Ui_MainWindow
import numpy as np

linearx = 0.0
angularz = 0.0

class ImageUpdater(QObject):
    image_signal = pyqtSignal(QImage)
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.sub = self.node.create_subscription(
            CompressedImage, '/camera_node/image_raw/compressed', self.image_callback, 10)
    def image_callback(self, msg):
        try:
            qimage = QImage.fromData(msg.data, "jpg")
            self.image_signal.emit(qimage)
        except Exception as e:
            self.node.get_logger().error(f"이미지 변환 오류: {e}")

class MapUpdater(QObject):
    map_signal = pyqtSignal(QImage)
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.sub = self.node.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
    def map_callback(self, msg):
        width = msg.info.width
        height = msg.info.height
        data = msg.data
        byte_array = np.zeros((height, width), dtype=np.uint8)
        for y in range(height):
            for x in range(width):
                idx = y * width + x
                if data[idx] == 0: byte_array[y, x] = 255
                elif data[idx] == 100: byte_array[y, x] = 0
                else: byte_array[y, x] = 128
        qimage = QImage(byte_array.data, width, height, width, QImage.Format_Grayscale8)
        qimage = qimage.mirrored(False, True)
        self.map_signal.emit(qimage)

class MainWindow(QMainWindow, Ui_MainWindow):  
    def __init__(self):
        super(MainWindow, self).__init__()
        self.setupUi(self)
        
        # ROS2 초기화
        rclpy.init()
        self.node = Node('ui_node')

        # 퍼블리셔 설정
        self.pub = self.node.create_publisher(Twist, 'cmd_vel', 10)

        self.stop_patrol_client = self.node.create_client(StopPatrol, '/stop_patrol')

        # 액션 클라이언트 설정
        self._action_client = ActionClient(self.node, Patrol, 'patrol')

        # 구독 설정
        self.sub_amcl_pose = self.node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_pose_callback, 10)
        self.sub_bottle_align_progress = self.node.create_subscription(Float32, '/bottle_align_progress', self.bottle_align_progress_callback, 10)
        self.sub_yolo_detect = self.node.create_subscription(String, '/yolo_detect', self.yolo_detect_callback, 10)
        self.sub_patrol_status = self.node.create_subscription(String, '/patrol_status', self.patrol_status_callback, 10)
        self.sub_current_point = self.node.create_subscription(String, '/current_patrol_point', self.current_point_callback, 10)

        # UI 초기화 및 연결
        self.nav_status.setText("N/A")
        self.dist_status.setText("N/A")
        self.feedback_status.setText("unknown")
        self.image_updater = ImageUpdater(self.node)
        self.image_updater.image_signal.connect(self.update_camera_feed)
        self.scene = QGraphicsScene()
        self.camera_feed.setScene(self.scene)
        self.pixmap_item = self.scene.addPixmap(QPixmap())
        self.map_scene = QGraphicsScene()
        self.graphicsView.setScene(self.map_scene)
        self.map_pixmap_item = self.map_scene.addPixmap(QPixmap())
        self.map_updater = MapUpdater(self.node)
        self.map_updater.map_signal.connect(self.update_map)

        # 버튼 연결
        self.forward.clicked.connect(self.move_forward)
        self.backward.clicked.connect(self.move_backward)
        self.left.clicked.connect(self.turn_left)
        self.right.clicked.connect(self.turn_right)
        self.stop.clicked.connect(self.stop_robot)
        self.start_patrol_button.clicked.connect(self.start_patrol)
        self.stop_patrol_button.clicked.connect(self.stop_patrol)

        # ROS2 스핀 타이머
        self.timer = QTimer()
        self.timer.timeout.connect(self.spin_ros)
        self.timer.start(100)

    def current_point_callback(self, msg):
        self.nav_status.setText(msg.data)

    def patrol_status_callback(self, msg):
        self.debug_text.append(msg.data)

    def amcl_pose_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, theta = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_pose_label.setText(f"Current Pose: x={x:.2f}, y={y:.2f}, θ={theta:.2f}")

    def bottle_align_progress_callback(self, msg):
        self.debug_text.append(f"Turn progress: {msg.data:.2f}")

    def yolo_detect_callback(self, msg):
        self.item_check.append(f"YOLO Result: {msg.data}")

    def update_camera_feed(self, qimage):
        pixmap = QPixmap.fromImage(qimage)
        self.pixmap_item.setPixmap(pixmap)
        self.camera_feed.fitInView(self.scene.sceneRect(), Qt.KeepAspectRatio)

    def update_map(self, qimage):
        pixmap = QPixmap.fromImage(qimage)
        self.map_pixmap_item.setPixmap(pixmap)
        self.graphicsView.fitInView(self.map_scene.sceneRect(), Qt.KeepAspectRatio)

    def move_forward(self):
        msg = Twist()
        msg.linear.x = linearx - 0.1
        self.pub.publish(msg)
        self.debug_text.append("Forward")

    def move_backward(self):
        msg = Twist()
        msg.linear.x = linearx + 0.1
        self.pub.publish(msg)
        self.debug_text.append("Backward")

    def turn_left(self):
        msg = Twist()
        msg.angular.z = angularz + 0.2
        self.pub.publish(msg)
        self.debug_text.append("Left")

    def turn_right(self):
        msg = Twist()
        msg.angular.z = angularz - 0.2
        self.pub.publish(msg)
        self.debug_text.append("Right")

    def stop_robot(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.pub.publish(msg)
        self.debug_text.append("Stop")

    def start_patrol(self):
        self.node.get_logger().info("Start Patrol 버튼이 클릭되었습니다")
        points = []
        for i in range(1, 5):
            text_edit = getattr(self, f'patrol_point_{i}', None)
            if text_edit is None:
                self.node.get_logger().error(f"patrol_point_{i} 텍스트 필드가 존재하지 않습니다")
                continue
            text = text_edit.toPlainText().strip()
            self.node.get_logger().info(f"Point {i} 입력값: {text}")
            if text:
                try:
                    x, y, theta = map(float, text.split())
                    pose = Pose()
                    pose.position.x = x
                    pose.position.y = y
                    pose.position.z = 0.0
                    q = quaternion_from_euler(0, 0, theta)
                    pose.orientation.x = q[0]
                    pose.orientation.y = q[1]
                    pose.orientation.z = q[2]
                    pose.orientation.w = q[3]
                    points.append(pose)
                except ValueError:
                    self.node.get_logger().error(f"Point {i}의 입력이 잘못되었습니다: {text}")
                    return

        if points:
            pose_array = PoseArray()
            pose_array.header.frame_id = 'map'
            pose_array.header.stamp = self.node.get_clock().now().to_msg()
            pose_array.poses = points
            goal_msg = Patrol.Goal()
            goal_msg.points = pose_array
            self._action_client.wait_for_server()
            self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
            self._send_goal_future.add_done_callback(self.goal_response_callback)
        else:
            self.node.get_logger().warning("패트롤 좌표가 입력되지 않았습니다")

    def stop_patrol(self):
        if not self.stop_patrol_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().error('Stop patrol service not available')
            self.debug_text.append("Stop patrol service unavailable")
            return
        request = StopPatrol.Request()  # 요청은 비어 있음
        future = self.stop_patrol_client.call_async(request)
        future.add_done_callback(self.stop_patrol_response_callback)

    def stop_patrol_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.node.get_logger().info('Patrol stopped successfully')
                self.debug_text.append("Patrol stopped successfully")
            else:
                self.node.get_logger().info(f'Patrol stop request: {response.message}')
                self.debug_text.append(f"Patrol stop: {response.message}")
        except Exception as e:
            self.node.get_logger().error(f'Error in stop patrol service call: {e}')
            self.debug_text.append(f"Error: {e}")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error('Patrol goal rejected')
            return
        self._goal_handle = goal_handle
        self.node.get_logger().info('Patrol goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.debug_text.append(feedback.status)
        self.nav_status.setText(feedback.current_point)

    def get_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.node.get_logger().info('Patrol completed successfully')
        else:
            self.node.get_logger().error('Patrol failed')
        self._goal_handle = None

    def spin_ros(self):
        rclpy.spin_once(self.node, timeout_sec=0)

    def closeEvent(self, event):
        rclpy.shutdown()
        super().closeEvent(event)

def main(args=None):
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    app.exec_()

if __name__ == '__main__':
    main()