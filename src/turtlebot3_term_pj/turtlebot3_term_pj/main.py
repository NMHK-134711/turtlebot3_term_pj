import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QGraphicsScene, QLabel
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt5.QtGui import QImage, QPixmap
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray, Pose, PoseWithCovarianceStamped
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Empty, String
from nav2_msgs.action import NavigateToPose
from nav2_msgs.action._navigate_to_pose import NavigateToPose_Feedback
from action_msgs.msg import GoalStatus, GoalStatusArray
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from .ui_main import Ui_MainWindow
import numpy as np

linearx = 0.0
angularz = 0.0

# ImageUpdater와 MapUpdater 클래스는 변경 없으므로 생략
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
        self.patrol_points_pub = self.node.create_publisher(PoseArray, '/patrol_points', 10)
        self.stop_patrol_pub = self.node.create_publisher(Empty, '/stop_patrol', 10)
        self.start_patrol_pub = self.node.create_publisher(Empty, '/start_patrol', 10)

        # 타이머 설정 (추가됨)
        self.start_patrol_timer = QTimer(self)
        self.start_patrol_timer.setSingleShot(True)
        self.start_patrol_timer.timeout.connect(self.publish_start_patrol)

        # 구독 설정 (기존 코드 유지, 생략)
        self.sub_amcl_pose = self.node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_pose_callback, 10)
        self.sub_patrol_status = self.node.create_subscription(String, '/patrol_status', self.patrol_status_callback, 10)
        self.sub_current_point = self.node.create_subscription(String, '/current_patrol_point', self.current_point_callback, 10)
        self.sub_nav_status = self.node.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.nav_status_callback, 10)
        self.sub_nav_feedback = self.node.create_subscription(NavigateToPose_Feedback, '/navigate_to_pose/_action/feedback', self.nav_feedback_callback, 10)
        self.sub_yolo_result = self.node.create_subscription(String, '/yolo_result', self.yolo_result_callback, 10)

        # UI 초기화 및 연결 (기존 코드 유지, 생략)
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

    # 콜백 및 이동 메서드 (변경 없음, 생략)
    def current_point_callback(self, msg): self.nav_status.setText(msg.data)
    def nav_status_callback(self, msg): pass  # 생략
    def nav_feedback_callback(self, msg): self.dist_status.setText(f"{msg.feedback.distance_remaining:.2f} m")
    def patrol_status_callback(self, msg): self.feedback_status.setText(msg.data)
    def amcl_pose_callback(self, msg): pass  # 생략
    def yolo_result_callback(self, msg): self.debug_text.append(f"YOLO Result: {msg.data}")
    def update_camera_feed(self, qimage): pass  # 생략
    def update_map(self, qimage): pass  # 생략
    def move_forward(self): pass  # 생략
    def move_backward(self): pass  # 생략
    def turn_left(self): pass  # 생략
    def turn_right(self): pass  # 생략
    def stop_robot(self): pass  # 생략

    def start_patrol(self):
        self.node.get_logger().info("Start Patrol 버튼이 클릭되었습니다")  # 디버그 로그 추가
        points = []
        for i in range(1, 5):
            text_edit = getattr(self, f'patrol_point_{i}', None)
            if text_edit is None:
                self.node.get_logger().error(f"patrol_point_{i} 텍스트 필드가 존재하지 않습니다")
                continue
            text = text_edit.toPlainText().strip()
            self.node.get_logger().info(f"Point {i} 입력값: {text}")  # 입력값 확인
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
            self.patrol_points_pub.publish(pose_array)
            self.node.get_logger().info("패트롤 좌표를 발행했습니다")

            # 0.5초 후 /start_patrol 메시지 발행
            self.start_patrol_timer.start(500)
        else:
            self.node.get_logger().warning("패트롤 좌표가 입력되지 않았습니다")

    def publish_start_patrol(self):
        start_patrol_msg = Empty()
        self.start_patrol_pub.publish(start_patrol_msg)
        self.node.get_logger().info("패트롤 시작 메시지를 발행했습니다")

    def stop_patrol(self):
        self.stop_patrol_pub.publish(Empty())
        self.node.get_logger().info("Stop patrol 명령을 보냈습니다")

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