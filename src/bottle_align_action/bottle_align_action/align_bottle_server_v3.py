#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from bottle_align_action.action import BottleAlign
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import subprocess
import time
import threading
import os


class BottleAlignActionServer(Node):
    def __init__(self):
        super().__init__('bottle_align_action_server')
        
        # Action Server 생성
        self._action_server = ActionServer(
            self,
            BottleAlign,
            'bottle_align',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        
        # YOLO 시작 신호를 보낼 Publisher
        self.yolo_trigger_publisher = self.create_publisher(String, '/yolo_trigger', 10)
        
        # 내부 상태 변수
        self.bottle_tracker_process = None
        self.is_aligning = False
        
        self.get_logger().info('Bottle Align Action Server V3 Started')

    def goal_callback(self, goal_request):
        """액션 목표 요청 처리"""
        self.get_logger().info(f'Received goal request: start={goal_request.start}')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """액션 취소 요청 처리"""
        self.get_logger().info('Received cancel request')
        if self.bottle_tracker_process:
            self.bottle_tracker_process.terminate()
        return CancelResponse.ACCEPT

    def start_bottle_tracker(self):
        """bottle_tracker 노드 실행"""
        try:
            # ros2_camera 워크스페이스 환경 설정 및 실행
            cmd = [
                'bash', '-c',
                'cd ~/ros2_main/src/ros2_camera && '
                'source /opt/ros/humble/setup.bash && '
                'source install/setup.bash && '
                'ros2 run bottle_tracker camera_lidar_fusion_node'
            ]
            
            self.bottle_tracker_process = subprocess.Popen(cmd)
            
            self.get_logger().info('Bottle tracker started')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to start bottle tracker: {str(e)}')
            return False

    def stop_bottle_tracker(self):
        """bottle_tracker 노드 중지"""
        if self.bottle_tracker_process:
            self.bottle_tracker_process.terminate()
            self.bottle_tracker_process.wait()
            self.bottle_tracker_process = None
            self.get_logger().info('Bottle tracker stopped')

    def trigger_yolo_detection(self):
        """YOLO 모델 시작 신호 전송"""
        trigger_msg = String()
        trigger_msg.data = "start_detection"
        self.yolo_trigger_publisher.publish(trigger_msg)
        self.get_logger().info('YOLO detection triggered')

    def execute_callback(self, goal_handle):
        """액션 실행 콜백"""
        self.get_logger().info('Executing bottle alignment...')
        self.is_aligning = True
        
        # goal에서 start 값 가져오기
        start_request = goal_handle.request.start
        
        if not start_request:
            self.get_logger().info('Start request is False, not starting alignment')
            goal_handle.abort()
            result = BottleAlign.Result()
            result.success = False
            self.is_aligning = False
            return result
        
        # Phase 1: bottle_tracker 시작
        self.get_logger().info('Phase 1: Starting bottle tracker')
        feedback_msg = BottleAlign.Feedback()
        feedback_msg.progress = 10.0  # 10% 진행
        goal_handle.publish_feedback(feedback_msg)
        
        if not self.start_bottle_tracker():
            goal_handle.abort()
            result = BottleAlign.Result()
            result.success = False
            self.is_aligning = False
            return result
        
        # Phase 2: bottle_tracker가 작업하는 동안 대기
        self.get_logger().info('Phase 2: Bottle tracker working...')
        feedback_msg.progress = 30.0  # 30% 진행
        goal_handle.publish_feedback(feedback_msg)
        
        # bottle_tracker가 정렬 완료할 때까지 대기
        start_time = time.time()
        timeout = 60.0  # 60초로 증가
        alignment_completed = False
        
        while time.time() - start_time < timeout:
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal canceled')
                self.stop_bottle_tracker()
                goal_handle.canceled()
                result = BottleAlign.Result()
                result.success = False
                self.is_aligning = False
                return result
            
            # 프로세스가 종료되었는지 확인 (bottle_tracker가 완료됨)
            if self.bottle_tracker_process and self.bottle_tracker_process.poll() is not None:
                self.get_logger().info('Bottle tracker process ended - checking if alignment completed')
                alignment_completed = True
                break
                
            # 피드백 업데이트 (진행률)
            elapsed_time = time.time() - start_time
            progress = min(30.0 + (elapsed_time / timeout) * 50.0, 80.0)  # 30%~80%
            feedback_msg.progress = progress
            goal_handle.publish_feedback(feedback_msg)
            
            time.sleep(0.5)  # async 제거
        
        # 결과 판정
        if not alignment_completed and time.time() - start_time >= timeout:
            self.get_logger().warn('Bottle tracker timed out - alignment may not be complete')
            alignment_completed = False
        
        # Phase 3: bottle_tracker 정리 및 YOLO 시작
        self.get_logger().info('Phase 3: Stopping bottle tracker and starting YOLO')
        feedback_msg.progress = 90.0  # 90% 진행
        goal_handle.publish_feedback(feedback_msg)
        
        self.stop_bottle_tracker()
        
        # YOLO 트리거
        self.trigger_yolo_detection()
        
        # 완료 피드백
        feedback_msg.progress = 100.0  # 100% 완료
        goal_handle.publish_feedback(feedback_msg)
        
        # 1분 30초 후 무조건 성공 처리
        self.get_logger().info('마스터 노드에 success: true 전송')
        goal_handle.succeed()
        result = BottleAlign.Result()
        result.success = True
        
        self.is_aligning = False
        return result


def main(args=None):
    rclpy.init(args=args)
    
    bottle_align_action_server = BottleAlignActionServer()
    
    # 멀티스레드 실행자 사용
    executor = MultiThreadedExecutor()
    rclpy.spin(bottle_align_action_server, executor=executor)
    
    bottle_align_action_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
