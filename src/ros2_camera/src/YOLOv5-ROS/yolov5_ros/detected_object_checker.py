#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from bboxes_ex_msgs.msg import BoundingBoxes

ZONE1_KEYWORDS = {'teddy bear', 'bottle', 'laptop', 'mouse'}
ZONE2_KEYWORDS = {'apple', 'cell phone'}

REQUIRED_ZONE1 = {'teddy bear', 'bottle','laptop', 'mouse'}
REQUIRED_ZONE2 = {'apple', 'cell phone'}

class LogicalZoneChecker(Node):
    def __init__(self):
        super().__init__('logical_zone_checker')
        self.subscription = self.create_subscription(
            BoundingBoxes,
            '/yolov5/bounding_boxes',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        detected = set(box.class_id for box in msg.bounding_boxes)

        in_zone1 = bool(detected & ZONE1_KEYWORDS)
        in_zone2 = bool(detected & ZONE2_KEYWORDS)

        if in_zone1 and not in_zone2:
            self.check_zone(detected, REQUIRED_ZONE1, "Zone 1")
        elif in_zone2 and not in_zone1:
            self.check_zone(detected, REQUIRED_ZONE2, "Zone 2")
        elif in_zone1 and in_zone2:
            self.get_logger().info("⚠️ 물체가 두 구역에 걸쳐 인식됨. 둘 다 체크합니다.")
            self.check_zone(detected, REQUIRED_ZONE1, "Zone 1")
            self.check_zone(detected, REQUIRED_ZONE2, "Zone 2")
        else:
            self.get_logger().info("❓ 구역을 판단할 수 없음. 기준 물체가 감지되지 않았습니다.")

    def check_zone(self, detected, required_set, zone_name):
        missing = required_set - detected
        if missing:
            self.get_logger().info(f'[{zone_name}] ❌ Missing: {", ".join(missing)}')
        else:
            self.get_logger().info(f'[{zone_name}] ✅ All required objects detected!')

def main(args=None):
    rclpy.init(args=args)
    node = LogicalZoneChecker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

