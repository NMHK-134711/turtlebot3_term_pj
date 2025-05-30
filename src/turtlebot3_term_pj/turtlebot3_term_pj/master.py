import sys
from PyQt5.QtWidgets import QApplication, QMainWindow
from ui_master import Ui_MainWindow
import subprocess
import os

class MasterWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.cartographer_process = None
        self.navigation_process = None
        self.patrol_process = None

        # 버튼 연결
        self.ui.start_ui_main_node.clicked.connect(self.start_main_ui)
        self.ui.start_cartographer.clicked.connect(self.start_cartographer)
        self.ui.save_map.clicked.connect(self.save_map)
        self.ui.kill_cartographer.clicked.connect(self.kill_cartographer)
        self.ui.start_nav2.clicked.connect(self.start_navigation)

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
            if self.cartographer_process:
                self.cartographer_process.terminate()
                self.cartographer_process.wait(timeout=5)
                self.cartographer_process = None
                self.ui.status_checker_kill_cartographer.setText("종료됨")
            else:
                self.ui.status_checker_kill_cartographer.setText("Cartographer가 실행 중이 아님")
        except Exception as e:
            self.ui.status_checker_kill_cartographer.setText(f"오류: {str(e)}")

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

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MasterWindow()
    window.show()
    sys.exit(app.exec_())