#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from rclpy.qos import QoSProfile
from cv_bridge import CvBridge
import cv2
import torch
import signal
import sys
import atexit

class BottleTracker(Node):
    def __init__(self):
        super().__init__('bottle_tracker')
        
        # 정리 상태 플래그
        self.cleanup_done = False
        
        # 시그널 핸들러 등록
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        atexit.register(self.cleanup)

        qos = QoSProfile(depth=10)
        self.camera_sub = self.create_subscription(Image, '/camera_node/image_raw', self.camera_callback, qos)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 정렬 상태 발행을 위한 Publisher 추가
        self.alignment_status_pub = self.create_publisher(String, '/bottle_alignment_status', 10)
        
        self.bridge = CvBridge()

        try:
            self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
            self.model.eval()
            self.get_logger().info("✅ YOLO 모델 로드 성공")
        except Exception as e:
            self.get_logger().error(f"❌ YOLO 모델 로드 실패: {e}")
            self.model = None

        self.image_height = 480
        self.image_width = 640
        self.bottle_angle = None
        self.bottle_detected = False
        self.state = "SEARCHING"
        self.turn_speed = 0.1
        self.timer = self.create_timer(0.1, self.control_loop)
        self.alignment_completed = False

    def signal_handler(self, signum, frame):
        """시그널 핸들러 - Ctrl+C 등으로 종료될 때 호출"""
        self.get_logger().info(f"시그널 {signum} 수신 - 정리 시작")
        self.cleanup()
        sys.exit(0)

    def cleanup(self):
        """정리 작업"""
        if self.cleanup_done:
            return
        
        self.cleanup_done = True
        
        try:
            self.get_logger().info("🧹 BottleTracker 정리 시작...")
            
            # 로봇 정지
            try:
                stop_cmd = Twist()
                stop_cmd.linear.x = 0.0
                stop_cmd.angular.z = 0.0
                self.cmd_pub.publish(stop_cmd)
            except Exception as e:
                self.get_logger().warning(f"로봇 정지 명령 실패: {e}")
            
            # OpenCV 윈도우 모두 닫기
            try:
                cv2.destroyAllWindows()
                cv2.waitKey(1)  # 윈도우가 완전히 닫힐 때까지 대기
            except Exception as e:
                self.get_logger().warning(f"OpenCV 윈도우 정리 실패: {e}")
            
            # 타이머 정리
            try:
                if hasattr(self, 'timer') and self.timer:
                    self.timer.cancel()
            except Exception as e:
                self.get_logger().warning(f"타이머 정리 실패: {e}")
            
            self.get_logger().info("✅ BottleTracker 정리 완료")
        except Exception as e:
            self.get_logger().error(f"정리 중 오류: {e}")

    def camera_callback(self, msg):
        if self.model is None or self.cleanup_done:
            return
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.image_height, self.image_width = img.shape[:2]
            results = self.model(img)
            detections = results.pandas().xyxy[0]
            bottles = detections[(detections['name'] == 'bottle') & (detections['confidence'] > 0.3)]

            if not bottles.empty:
                bottles = bottles.copy()
                bottles['area'] = (bottles['xmax'] - bottles['xmin']) * (bottles['ymax'] - bottles['ymin'])
                best = bottles.loc[bottles['area'].idxmax()]

                x1, y1, x2, y2 = best['xmin'], best['ymin'], best['xmax'], best['ymax']
                center_x = (x1 + x2) / 2
                pixel_offset = center_x - (self.image_width / 2)
                angle = (pixel_offset / (self.image_width / 2)) * (62.2 / 2)

                self.bottle_angle = angle
                self.bottle_detected = True

                # 시각화
                cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.circle(img, (int(center_x), int((y1 + y2) / 2)), 5, (0, 0, 255), -1)
                cv2.putText(img, f"Angle: {angle:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.putText(img, f"State: {self.state}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            else:
                self.bottle_detected = False
                self.bottle_angle = None

            # cleanup_done이 True가 아닐 때만 윈도우 표시
            if not self.cleanup_done:
                cv2.imshow("Bottle Tracker", img)
                cv2.waitKey(1)
                
        except Exception as e:
            self.get_logger().error(f"카메라 처리 오류: {e}")

    def control_loop(self):
        if self.cleanup_done:
            return
            
        cmd = Twist()
        status_msg = String()

        if self.state == "SEARCHING":
            status_msg.data = "SEARCHING"
            if self.bottle_detected and self.bottle_angle is not None:
                self.state = "TURNING"
            else:
                cmd.angular.z = 0.15  # 계속 회전

        elif self.state == "TURNING":
            status_msg.data = "TURNING"
            if not self.bottle_detected:
                self.state = "SEARCHING"
            elif abs(self.bottle_angle) > 5.0:
                cmd.angular.z = -self.turn_speed if self.bottle_angle > 0 else self.turn_speed
            else:
                self.get_logger().info("🛑 정면 정렬 완료 - 정지")
                self.state = "DONE"
                if not self.alignment_completed:
                    self.alignment_completed = True

        elif self.state == "DONE":
            status_msg.data = "ALIGNMENT_COMPLETED"
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        try:
            self.cmd_pub.publish(cmd)
            self.alignment_status_pub.publish(status_msg)
        except Exception as e:
            self.get_logger().warning(f"메시지 발행 실패: {e}")

    def __del__(self):
        """소멸자에서 정리"""
        self.cleanup()

def main(args=None):
    try:
        rclpy.init(args=args)
        node = BottleTracker()
        
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info("🛑 KeyboardInterrupt 수신")
        except Exception as e:
            node.get_logger().error(f"스핀 중 오류: {e}")
        finally:
            # 정리 작업
            node.cleanup()
            try:
                node.destroy_node()
            except Exception as e:
                print(f"노드 정리 실패: {e}")
                
    except Exception as e:
        print(f"메인 함수 오류: {e}")
    finally:
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as e:
            print(f"RCLPy 종료 실패: {e}")

if __name__ == '__main__':
    main()
