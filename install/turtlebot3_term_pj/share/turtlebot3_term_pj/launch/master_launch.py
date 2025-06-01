from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    # 현재 파일(master_launch.py)의 디렉토리 가져오기
    current_dir = os.path.dirname(__file__)
    # master.py로의 상대경로 계산
    master_script = os.path.join(current_dir, '..', 'turtlebot3_term_pj', 'master.py')
    
    return LaunchDescription([
        ExecuteProcess(
            cmd=['python3', master_script],
            output='screen'
        )
    ])