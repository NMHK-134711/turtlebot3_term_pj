#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import torch
import numpy as np
import math

class CameraLidarFusionNode(Node):
    def __init__(self):
        super().__init__('camera_lidar_fusion_node')
        
        # ROS2 통신 설정
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.camera_sub = self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # OpenCV 브리지
        self.bridge = CvBridge()
        
        # YOLO v5 모델 로드
        try:
            self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
            self.model.eval()
            self.get_logger().info("✅ YOLO v5 모델 로드 성공!")
        except Exception as e:
            self.get_logger().error(f"❌ YOLO v5 모델 로드 실패: {e}")
            self.model = None
        
        # 센서 데이터
        self.front_distance = float('inf')
        self.latest_image = None
        
        # bottle 감지 정보
        self.bottle_detected = False
        self.bottle_center_x = None
        self.bottle_angle = None
        self.bottle_distance = None
        
        # 카메라 파라미터
        self.image_width = 640
        self.image_height = 480
        self.camera_fov = 62.2  # 시야각 (도)
        
        # 상태 관리
        self.state = "SEARCH"
        self.step_timer = 0
        self.count = 0
        
        # 타이머
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("🎯 카메라-LiDAR 퓨전 bottle 추적 시스템 시작!")

    def camera_callback(self, msg):
        """카메라 이미지 처리 및 YOLO bottle 감지"""
        if self.model is None:
            return
            
        try:
            # ROS 이미지를 OpenCV 이미지로 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_image = cv_image.copy()
            self.image_height, self.image_width = cv_image.shape[:2]
            
            # YOLO v5로 객체 감지
            results = self.model(cv_image)
            detections = results.pandas().xyxy[0]
            
            # bottle 감지 확인
            bottles = detections[detections['name'] == 'bottle']
            
            if len(bottles) > 0:
                # 가장 큰 bottle 선택
                largest_bottle = bottles.loc[bottles['confidence'].idxmax()]
                
                # bottle 중심 좌표 계산
                x1, y1, x2, y2 = largest_bottle['xmin'], largest_bottle['ymin'], largest_bottle['xmax'], largest_bottle['ymax']
                self.bottle_center_x = int((x1 + x2) / 2)
                bottle_center_y = int((y1 + y2) / 2)
                
                # 각도 계산
                pixel_offset = self.bottle_center_x - (self.image_width / 2)
                self.bottle_angle = (pixel_offset / (self.image_width / 2)) * (self.camera_fov / 2)
                
                self.bottle_detected = True
                
                # 시각화
                cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.circle(cv_image, (self.bottle_center_x, bottle_center_y), 5, (0, 0, 255), -1)
                cv2.putText(cv_image, f'Bottle: {largest_bottle["confidence"]:.2f}', 
                           (int(x1), int(y1-10)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(cv_image, f'Angle: {self.bottle_angle:.1f}°', 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
                
                if self.count % 20 == 0:
                    self.get_logger().info(f"🍾 Bottle 감지! 각도: {self.bottle_angle:.1f}°, 신뢰도: {largest_bottle['confidence']:.2f}")
            else:
                self.bottle_detected = False
                self.bottle_center_x = None
                self.bottle_angle = None
            
            # 결과 이미지 표시
            cv2.imshow('YOLO Bottle Detection', cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"카메라 콜백 오류: {e}")

    def lidar_callback(self, msg):
        """LiDAR 데이터 처리"""
        if len(msg.ranges) == 0:
            return
        
        # 정면 거리 계산
        front_ranges = []
        for i, distance in enumerate(msg.ranges):
            if 0.1 <= distance <= 10.0:
                angle_rad = msg.angle_min + i * msg.angle_increment
                angle_deg = math.degrees(angle_rad)
                
                # 각도 정규화
                while angle_deg > 180:
                    angle_deg -= 360
                while angle_deg < -180:
                    angle_deg += 360
                
                # 정면 거리
                if -15 <= angle_deg <= 15:
                    front_ranges.append(distance)
        
        self.front_distance = min(front_ranges) if front_ranges else float('inf')
        
        # bottle 방향 거리 측정
        if self.bottle_detected and self.bottle_angle is not None:
            self.bottle_distance = self.get_distance_at_angle(msg, self.bottle_angle)

    def get_distance_at_angle(self, scan_msg, target_angle):
        """특정 각도에서의 거리 측정"""
        best_distance = float('inf')
        angle_tolerance = 5.0
        
        for i, distance in enumerate(scan_msg.ranges):
            if 0.1 <= distance <= 10.0:
                angle_rad = scan_msg.angle_min + i * scan_msg.angle_increment
                angle_deg = math.degrees(angle_rad)
                
                while angle_deg > 180:
                    angle_deg -= 360
                while angle_deg < -180:
                    angle_deg += 360
                
                angle_diff = abs(angle_deg - target_angle)
                if angle_diff > 180:
                    angle_diff = 360 - angle_diff
                
                if angle_diff <= angle_tolerance and distance < best_distance:
                    best_distance = distance
        
        return best_distance if best_distance != float('inf') else None

    def control_loop(self):
        """제어 루프"""
        self.count += 1
        self.step_timer += 1
        
        if self.state == "SEARCH":
            self.search_phase()
        elif self.state == "TURN":
            self.turn_phase()
        elif self.state == "APPROACH":
            self.approach_phase()
        elif self.state == "ADJUST":
            self.adjust_phase()
        elif self.state == "DONE":
            self.done_phase()

    def search_phase(self):
        """bottle 탐색"""
        if self.count % 30 == 0:
            self.get_logger().info(f"🔍 Bottle 탐색 중... {self.step_timer//10}초")
        
        if self.bottle_detected and self.bottle_angle is not None:
            self.stop_robot()
            self.state = "TURN"
            self.step_timer = 0
            self.get_logger().info(f"✅ Bottle 발견! 각도: {self.bottle_angle:.1f}°")
        else:
            twist = Twist()
            twist.angular.z = 0.3
            self.cmd_pub.publish(twist)

    def turn_phase(self):
        """bottle 방향으로 회전"""
        if not self.bottle_detected:
            self.state = "SEARCH"
            self.step_timer = 0
            return
        
        angle_error = self.bottle_angle
        
        if self.count % 10 == 0:
            self.get_logger().info(f"🔄 회전 중... 오차: {angle_error:.1f}°")
        
        if abs(angle_error) > 2:
            twist = Twist()
            rotation_speed = max(0.1, min(0.4, abs(angle_error) * 0.05))
            
            if angle_error > 0:
                twist.angular.z = rotation_speed
            else:
                twist.angular.z = -rotation_speed
            
            self.cmd_pub.publish(twist)
        else:
            self.stop_robot()
            self.state = "APPROACH"
            self.step_timer = 0
            self.get_logger().info("✅ 회전 완료! 접근 시작")

    def approach_phase(self):
        """접근"""
        target_distance = 0.25
        
        if self.count % 10 == 0:
            self.get_logger().info(f"➡️ 접근 중... 거리: {self.front_distance:.2f}m")
        
        if not self.bottle_detected:
            self.stop_robot()
            self.state = "SEARCH"
            self.step_timer = 0
            return
        
        if self.front_distance > target_distance + 0.05:
            twist = Twist()
            distance_error = self.front_distance - target_distance
            move_speed = max(0.05, min(0.2, distance_error * 0.3))
            twist.linear.x = move_speed
            
            # 방향 유지
            if abs(self.bottle_angle) > 1:
                twist.angular.z = self.bottle_angle * 0.02
            
            self.cmd_pub.publish(twist)
        else:
            self.stop_robot()
            self.state = "ADJUST"
            self.step_timer = 0
            self.get_logger().info("✅ 접근 완료! 정밀 조정 시작")

    def adjust_phase(self):
        """15cm 정밀 조정"""
        target_distance = 0.15
        tolerance = 0.02
        
        distance_error = self.front_distance - target_distance
        
        if self.count % 10 == 0:
            self.get_logger().info(f"🎯 정밀 조정... 현재: {self.front_distance:.3f}m, 목표: 0.150m")
        
        if abs(distance_error) > tolerance:
            twist = Twist()
            adjust_speed = max(0.01, min(0.05, abs(distance_error) * 0.5))
            
            if distance_error > 0:
                twist.linear.x = adjust_speed
            else:
                twist.linear.x = -adjust_speed
            
            self.cmd_pub.publish(twist)
        else:
            self.stop_robot()
            self.state = "DONE"
            self.get_logger().info("🎉 15cm 정밀 정렬 완료!")

    def done_phase(self):
        """완료"""
        self.stop_robot()
        if self.count % 50 == 0:
            self.get_logger().info("🏆 Bottle 추적 미션 완료!")

    def stop_robot(self):
        twist = Twist()
        self.cmd_pub.publish(twist)

    def __del__(self):
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = CameraLidarFusionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 미션 중단됨")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
