import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QGraphicsScene
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt5.QtGui import QImage, QPixmap
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import OccupancyGrid
from ui_main import Ui_MainWindow  # .ui 파일에서 생성된 클래스
import numpy as np
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

linearx = 0.0
angularz = 0.0

class ImageUpdater(QObject):
    image_signal = pyqtSignal(QImage)

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.sub = self.node.create_subscription(
            CompressedImage,
            '/camera_node/image_raw/compressed', #real turtlebot
            #'camera/image_raw/compressed', #sim turtlebot_cam
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
        
        # cmd_vel 퍼블리셔 설정
        self.pub = self.node.create_publisher(Twist, 'cmd_vel', 10)
        
        # 카메라 피드 구독 설정
        self.image_updater = ImageUpdater(self.node)
        self.image_updater.image_signal.connect(self.update_camera_feed)
        
        # QGraphicsScene 설정
        self.scene = QGraphicsScene()
        self.camera_feed.setScene(self.scene)

        # QGraphicsPixmapItem을 미리 추가 (재사용할 아이템)
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

        # ROS2 스핀을 위한 타이머 설정
        self.timer = QTimer()
        self.timer.timeout.connect(self.spin_ros)
        self.timer.start(100)  # 100ms마다 스핀

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
        msg.linear.x = linearx - 0.05  #real turtlebot
        #msg.linear.x = linearx + 0.05  #sim turtlebot
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

    def spin_ros(self):
        rclpy.spin_once(self.node, timeout_sec=0)

    def closeEvent(self, event):
        rclpy.shutdown()
        super().closeEvent(event)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())