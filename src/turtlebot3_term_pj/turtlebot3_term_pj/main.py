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
from tf_transformations import quaternion_from_euler, euler_from_quaternion
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
            CompressedImage,
            '/camera_node/image_raw/compressed',  # real turtlebot
            #'camera/image_raw/compressed',  # sim turtlebot_cam
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        try:
            qimage = QImage.fromData(msg.data, "jpg")  # 압축 이미지 처리
            self.image_signal.emit(qimage)
        except Exception as e:
            self.node.get_logger().error(f"이미지 변환 오류: {e}")

class MapUpdater(QObject):
    map_signal = pyqtSignal(QImage)

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.sub = self.node.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

    def map_callback(self, msg):
        width = msg.info.width
        height = msg.info.height
        data = msg.data

        byte_array = np.zeros((height, width), dtype=np.uint8)
        for y in range(height):
            for x in range(width):
                idx = y * width + x
                if data[idx] == 0:
                    byte_array[y, x] = 255
                elif data[idx] == 100:
                    byte_array[y, x] = 0
                else:
                    byte_array[y, x] = 128

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

        # /amcl_pose 토픽 구독
        self.sub_amcl_pose = self.node.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            10
        )

        self.sub_patrol_status = self.node.create_subscription(
            String,
            '/patrol_status',
            self.patrol_status_callback,
            10
        )

        # /yolo_result 토픽 구독 추가
        self.sub_yolo_result = self.node.create_subscription(
            String,
            '/yolo_result',
            self.yolo_result_callback,
            10
        )

        # UI 라벨 초기화
        self.nav_status.setText("N/A")
        self.dist_status.setText("N/A")
        self.feedback_status.setText("unknown")
        
        # cmd_vel 퍼블리셔 설정
        self.pub = self.node.create_publisher(Twist, 'cmd_vel', 10)
        self.patrol_points_pub = self.node.create_publisher(PoseArray, '/patrol_points', 10)
        self.stop_patrol_pub = self.node.create_publisher(Empty, '/stop_patrol', 10)
        
        # 카메라 피드 구독 설정
        self.image_updater = ImageUpdater(self.node)
        self.image_updater.image_signal.connect(self.update_camera_feed)
        
        # QGraphicsScene 설정
        self.scene = QGraphicsScene()
        self.camera_feed.setScene(self.scene)

        # QGraphicsPixmapItem 추가
        self.pixmap_item = self.scene.addPixmap(QPixmap())
        
        self.map_scene = QGraphicsScene()
        self.graphicsView.setScene(self.map_scene)
        self.map_pixmap_item = self.map_scene.addPixmap(QPixmap())

        self.map_updater = MapUpdater(self.node)
        self.map_updater.map_signal.connect(self.update_map)

        # UI 버튼 연결
        self.forward.clicked.connect(self.move_forward)
        self.backward.clicked.connect(self.move_backward)
        self.left.clicked.connect(self.turn_left)
        self.right.clicked.connect(self.turn_right)
        self.stop.clicked.connect(self.stop_robot)
        self.start_patrol_button.clicked.connect(self.start_patrol)
        self.stop_patrol_button.clicked.connect(self.stop_patrol)

        # ROS2 스핀을 위한 타이머 설정
        self.timer = QTimer()
        self.timer.timeout.connect(self.spin_ros)
        self.timer.start(100)  # 100ms마다 스핀

    def patrol_status_callback(self, msg):
        self.debug_text.append(msg.data)  # 디버깅용 텍스트 유지
        text = msg.data
        if text.startswith("Moving to point"):
            # "Moving to point X, remaining distance: Y.YY m" 파싱
            parts = text.split(",")
            point_part = parts[0].split("point ")[1]  # "X" 추출
            distance_part = parts[1].split(": ")[1].split(" m")[0]  # "Y.YY" 추출
            self.nav_status.setText(f"Point{point_part}")
            self.dist_status.setText(f"{distance_part} m")
            self.feedback_status.setText("moving")
        elif text.startswith("Reached point"):
            # "Reached point X" 파싱
            point_part = text.split("point ")[1]  # "X" 추출
            self.nav_status.setText(f"Point{point_part}")
            self.dist_status.setText("0.00 m")
            self.feedback_status.setText("reached")
        else:
            # 그 외 메시지 (예: "Navigation goal accepted", "Navigation failed", "Patrol stopped")
            self.nav_status.setText("N/A")
            self.dist_status.setText("N/A")
            self.feedback_status.setText("unknown")

    def amcl_pose_callback(self, msg):
        # 위치 (x, y) 추출
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        # 쿼터니언에서 θ(yaw) 추출
        q = msg.pose.pose.orientation
        _, _, theta = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        # UI에 현재 위치 업데이트
        self.current_pose_label.setText(f"Current Pose: x={x:.2f}, y={y:.2f}, θ={theta:.2f}")

    # /yolo_result 콜백 함수 추가
    def yolo_result_callback(self, msg):
        self.debug_text.append(f"YOLO Result: {msg.data}")

    def update_camera_feed(self, qimage):
        # QImage를 QPixmap으로 변환
        pixmap = QPixmap.fromImage(qimage)
        
        # 기존 QGraphicsPixmapItem의 픽스맵을 업데이트
        self.pixmap_item.setPixmap(pixmap)
        
        # 뷰의 크기에 맞게 씬을 조정 (화면에 꽉 차게 표시)
        self.camera_feed.fitInView(self.scene.sceneRect(), Qt.KeepAspectRatio)

    def update_map(self, qimage):
        pixmap = QPixmap.fromImage(qimage)
        self.map_pixmap_item.setPixmap(pixmap)
        self.graphicsView.fitInView(self.map_scene.sceneRect(), Qt.KeepAspectRatio)

    def move_forward(self):
        msg = Twist()
        msg.linear.x = linearx - 0.05  # real turtlebot
        #msg.linear.x = linearx + 0.05  # sim turtlebot
        self.pub.publish(msg)
        self.debug_text.append("Forward")

    def move_backward(self):
        msg = Twist()
        msg.linear.x = linearx + 0.05  # real turtlebot
        #msg.linear.x = linearx + 0.05  # sim turtlebot
        self.pub.publish(msg)
        self.debug_text.append("Backward")

    def turn_left(self):
        msg = Twist()
        msg.angular.z = angularz + 0.2  # 좌회전 각속도
        self.pub.publish(msg)
        self.debug_text.append("Left")

    def turn_right(self):
        msg = Twist()
        msg.angular.z = angularz - 0.2  # 우회전 각속도
        self.pub.publish(msg)
        self.debug_text.append("Right")

    def stop_robot(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.pub.publish(msg)
        self.debug_text.append("Stop")

    def start_patrol(self):
        points = []
        for i in range(1, 5):
            text_edit = getattr(self, f'patrol_point_{i}')
            text = text_edit.toPlainText().strip()
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
        else:
            self.node.get_logger().warning("패트롤 좌표가 입력되지 않았습니다")

    def stop_patrol(self):
        self.stop_patrol_pub.publish(Empty())
        self.node.get_logger().info("Stop patrol command sent")

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