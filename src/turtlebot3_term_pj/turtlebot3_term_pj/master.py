import sys
from PyQt5.QtWidgets import QApplication, QMainWindow
from turtlebot3_term_pj.ui_master import Ui_MainWindow
import subprocess
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class EmergencyStopNode(Node):
    def __init__(self):
        super().__init__('emergency_stop_node')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)
        self.get_logger().info('Emergency stop command sent to the robot')

class MasterWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.cartographer_process = None
        self.navigation_process = None
        self.patrol_process = None
        self.main_ui_process = None

        rclpy.init()
        self.emergency_stop_node = EmergencyStopNode()

        # 버튼 연결
        self.ui.start_ui_main_node.clicked.connect(self.start_main_ui)
        self.ui.start_cartographer.clicked.connect(self.start_cartographer)
        self.ui.save_map.clicked.connect(self.save_map)
        self.ui.kill_cartographer.clicked.connect(self.kill_cartographer)
        self.ui.start_nav2.clicked.connect(self.start_navigation)
        self.ui.emergency_stop.clicked.connect(self.emergency_stop)

    def start_main_ui(self):
        try:
            subprocess.Popen(['ros2', 'run', 'turtlebot3_term_pj', 'ui_main_node'])
            self.ui.status_checker_ui_main.setText("실행 중")
        except Exception as e:
            self.ui.status_checker_ui_main.setText(f"오류: {str(e)}")

    def start_cartographer(self):
        try:
            self.cartographer_process = subprocess.Popen(
                ['ros2', 'launch', 'turtlebot3_cartographer', 'cartographer.launch.py', 'use_sim_time:=True'],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE
            )
            self.ui.status_checker_cartographer.setText("실행 중")
        except Exception as e:
            self.ui.status_checker_cartographer.setText(f"오류: {str(e)}")

    def save_map(self):
        try:
            home_dir = os.path.expanduser("~")
            subprocess.run(['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', f'{home_dir}/map'], check=True)
            self.ui.status_checker_save_map.setText("맵 저장 완료")
        except subprocess.CalledProcessError as e:
            self.ui.status_checker_save_map.setText(f"오류: {str(e)}")

    def kill_cartographer(self):
        try:
            # Cartographer 관련 모든 프로세스 종료
            subprocess.run(["pkill", "-f", "cartographer_node"], check=True)
            subprocess.run(["pkill", "-f", "cartographer_occupancy_grid_node"], check=True)
            # 혹시 모를 추가적인 Cartographer 관련 프로세스 종료
            subprocess.run(["pkill", "-f", "cartographer"], check=True)
            if self.cartographer_process:
                self.cartographer_process.terminate()
                self.cartographer_process.wait(timeout=5)
                self.cartographer_process = None
            self.ui.status_checker_cartographer.setText("종료됨")
            self.ui.status_checker_kill_cartographer.setText("종료됨")
        except subprocess.CalledProcessError as e:
            self.ui.status_checker_kill_cartographer.setText(f"오류: {str(e)}")
        except Exception as e:
            self.ui.status_checker_kill_cartographer.setText(f"프로세스 종료 오류: {str(e)}")

    def start_navigation(self):
        try:
            home_dir = os.path.expanduser("~")
            self.navigation_process = subprocess.Popen(
                ['ros2', 'launch', 'turtlebot3_navigation2', 'navigation2.launch.py', f'map:={home_dir}/map.yaml'],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE
            )
            self.patrol_process = subprocess.Popen(
                ['ros2', 'run', 'turtlebot3_term_pj', 'patrol_node'],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE
            )
            self.ui.status_checker_nav2.setText("실행 중")
        except Exception as e:
            self.ui.status_checker_nav2.setText(f"오류: {str(e)}")

    def emergency_stop(self):
        # 모든 프로세스 중지
        processes = [self.cartographer_process, self.navigation_process, self.patrol_process, self.main_ui_process]
        for process in processes:
            if process:
                try:
                    process.terminate()
                    process.wait(timeout=5)
                except Exception as e:
                    self.ui.status_checker_ui_main.setText(f"프로세스 종료 오류: {str(e)}")

        # 터틀봇 정지 명령 전송
        self.emergency_stop_node.stop_robot()

        # UI 업데이트
        self.ui.status_checker_ui_main.setText("비상 정지")
        self.ui.status_checker_cartographer.setText("비상 정지")
        self.ui.status_checker_save_map.setText("비상 정지")
        self.ui.status_checker_kill_cartographer.setText("비상 정지")
        self.ui.status_checker_nav2.setText("비상 정지")

        # 프로세스 변수 초기화
        self.cartographer_process = None
        self.navigation_process = None
        self.patrol_process = None
        self.main_ui_process = None

def main():
    app = QApplication(sys.argv)
    window = MasterWindow()
    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
