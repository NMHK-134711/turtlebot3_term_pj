import launch
import launch_ros.actions
from pathlib import Path

def generate_launch_description():
    # 현재 launch 파일 기준으로 상위 폴더 탐색
    launch_dir = Path(__file__).parent  # .../yolov5_ros/launch
    pkg_dir = launch_dir.parent         # .../yolov5_ros

    data_path = str(pkg_dir / 'data' / 'coco128.yaml')  # 소스 폴더 기준 상대경로

    yolov5_ros_node = launch_ros.actions.Node(
        package='yolov5_ros',
        executable='yolov5_ros',
        name='yolov5_ros',
        output='screen',
        parameters=[
            {"view_img": True},
            {"data": data_path},
        ],
        remappings=[
            ("/image_raw", "/camera_node/image_raw")
        ]
    )

    zone_checker_node = launch_ros.actions.Node(
        package='yolov5_ros',
        executable='detected_object_checker',
        name='detected_object_checker',
        output='screen',
    )

    return launch.LaunchDescription([
        yolov5_ros_node,
        zone_checker_node,
    ])

