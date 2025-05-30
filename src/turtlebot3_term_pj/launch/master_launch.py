from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    # master.py의 경로를 프로젝트 구조에 맞게 설정
    package_path = '/home/hk/turtlebot3_term_pj/src/turtlebot3_term_pj/turtlebot3_term_pj'
    master_script = os.path.join(package_path, 'master.py')
    
    return LaunchDescription([
        ExecuteProcess(
            cmd=['python3', master_script],
            output='screen'
        )
    ])