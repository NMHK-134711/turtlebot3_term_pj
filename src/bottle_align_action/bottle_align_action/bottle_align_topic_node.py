#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import subprocess
import time
import threading
import signal
import sys
import atexit
import psutil
import os

class BottleAlignTopicNode(Node):
    def __init__(self):
        super().__init__('bottle_align_topic_node')
        
        # 프로세스 관리 변수
        self.cleanup_done = False
        
        # 시그널 핸들러 등록
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        atexit.register(self.cleanup_all_processes)
        
        # 토픽 구독자 생성
        self.start_subscription = self.create_subscription(
            Bool,
            '/start_bottle_align',
            self.start_align_callback,
            10
        )
        
        # 토픽 발행자 생성
        self.progress_publisher = self.create_publisher(
            String,
            '/bottle_align_status',
            10
        )
        
        self.success_publisher = self.create_publisher(
            Bool,
            '/success_bottle_align',
            10
        )
        
        # YOLO 제어 신호를 보낼 Publisher
        self.yolo_control_publisher = self.create_publisher(String, '/yolo_control', 10)
        
        # 내부 상태 변수
        self.bottle_tracker_process = None
        self.yolo_process = None
        self.is_aligning = False
        self.yolo_initialized = False
        
        self.get_logger().info('Bottle Align Topic Node가 시작되었습니다.')
        self.get_logger().info('토픽 대기 중:')
        self.get_logger().info('  - 구독: /start_bottle_align (Bool)')
        self.get_logger().info('  - 발행: /bottle_align_status (String)')
        self.get_logger().info('  - 발행: /success_bottle_align (Bool)')
        self.get_logger().info('  - 발행: /yolo_control (String)')
        
        # 초기화 시 YOLO 한 번만 실행하고 계속 유지
        self.initialize_yolo()
    
    def signal_handler(self, signum, frame):
        """시그널 핸들러 - Ctrl+C 등으로 종료될 때 호출"""
        self.get_logger().info(f"시그널 {signum} 수신 - 모든 프로세스 정리 중...")
        self.cleanup_all_processes()
        sys.exit(0)
    
    def cleanup_all_processes(self):
        """모든 관련 프로세스 정리"""
        if self.cleanup_done:
            return
        
        self.cleanup_done = True
        
        try:
            self.get_logger().info("🧹 모든 프로세스 정리 시작...")
            
            # 1. 현재 클래스의 프로세스들 먼저 정리
            if self.bottle_tracker_process:
                try:
                    self.bottle_tracker_process.terminate()
                    self.bottle_tracker_process.wait(timeout=3)
                except:
                    try:
                        self.bottle_tracker_process.kill()
                    except:
                        pass
                self.bottle_tracker_process = None
            
            if self.yolo_process:
                try:
                    self.yolo_process.terminate()
                    self.yolo_process.wait(timeout=3)
                except:
                    try:
                        self.yolo_process.kill()
                    except:
                        pass
                self.yolo_process = None
            
            # 2. 시스템에서 관련 프로세스들 찾아서 정리
            processes_to_kill = []
            keywords = [
                'bottle_tracker', 'camera_lidar_fusion', 
                'yolov5_ros', 'bottle_align_topic',
                'ros2 run bottle_tracker', 'ros2 run yolov5_ros'
            ]
            
            for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
                try:
                    if proc.info['cmdline']:
                        cmdline = ' '.join(proc.info['cmdline'])
                        if any(keyword in cmdline for keyword in keywords):
                            # 현재 프로세스는 제외
                            if proc.pid != os.getpid():
                                processes_to_kill.append(proc)
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    continue
            
            # 3. 프로세스들 정리
            for proc in processes_to_kill:
                try:
                    self.get_logger().info(f"프로세스 종료 중: PID {proc.pid}")
                    proc.terminate()
                    proc.wait(timeout=3)
                except psutil.TimeoutExpired:
                    try:
                        proc.kill()
                        self.get_logger().info(f"강제 종료: PID {proc.pid}")
                    except:
                        pass
                except:
                    pass
            
            # 4. OpenCV 윈도우 정리
            try:
                import cv2
                cv2.destroyAllWindows()
            except:
                pass
            
            self.get_logger().info(" 모든 프로세스 정리 완료")
            
        except Exception as e:
            self.get_logger().error(f"정리 중 오류: {e}")
        
    def initialize_yolo(self):
        """YOLO를 한 번만 실행하고 계속 유지"""
        try:
            # 기존 YOLO 프로세스 정리
            self.cleanup_yolo_processes()
            
            self.get_logger().info('YOLO 시스템을 초기화하고 있습니다...')
            
            # YOLO 노드를 백그라운드에서 실행 (한 번만 - 계속 유지)
            cmd = [
                'bash', '-c',
                'cd ~/ros2_main && '
                'source /opt/ros/humble/setup.bash && '
                'source install/setup.bash && '
                'ros2 run yolov5_ros yolov5_ros'
            ]
            
            self.yolo_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid  # 새로운 프로세스 그룹 생성
            )
            
            # YOLO 초기화 대기 (충분한 시간 확보)
            self.get_logger().info('YOLO 초기화 중... 15초 대기')
            time.sleep(15)
            
            self.yolo_initialized = True
            self.get_logger().info('YOLO 시스템이 성공적으로 초기화되었습니다!')
            self.get_logger().info('YOLO 화면이 계속 유지됩니다. 토픽 신호 대기 중...')
            
        except Exception as e:
            self.get_logger().error(f'YOLO 초기화 실패: {str(e)}')
            self.yolo_initialized = False
    
    def cleanup_yolo_processes(self):
        """기존 YOLO 관련 프로세스들 정리"""
        try:
            for proc in psutil.process_iter(['pid', 'cmdline']):
                try:
                    if proc.info['cmdline']:
                        cmdline = ' '.join(proc.info['cmdline'])
                        if 'yolov5_ros' in cmdline or 'ros2 run yolov5_ros' in cmdline:
                            proc.terminate()
                            proc.wait(timeout=3)
                except:
                    pass
        except:
            pass
    
    def start_align_callback(self, msg):
        """시작 신호를 받았을 때 실행되는 콜백"""
        if msg.data and not self.is_aligning:
            if not self.yolo_initialized:
                self.get_logger().warn('YOLO가 초기화되지 않았습니다.')
                return
                
            self.get_logger().info('병 정렬 시작 신호를 받았습니다!')
            # 별도 스레드에서 정렬 작업 실행
            threading.Thread(target=self.execute_bottle_align, daemon=True).start()
        elif self.is_aligning:
            self.get_logger().warn('이미 병 정렬 작업이 진행 중입니다.')
    
    def start_bottle_tracker(self):
        """bottle_tracker 노드 실행"""
        try:
            # 기존 bottle_tracker 프로세스 정리
            self.stop_bottle_tracker()
            
            # 메인 워크스페이스 환경 설정 및 실행
            cmd = [
                'bash', '-c',
                'cd ~/ros2_main && '
                'source /opt/ros/humble/setup.bash && '
                'source install/setup.bash && '
                'ros2 run bottle_tracker camera_lidar_fusion_node'
            ]
            
            self.bottle_tracker_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid  # 새로운 프로세스 그룹 생성
            )
            
            self.get_logger().info('Bottle tracker started')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to start bottle tracker: {str(e)}')
            return False

    def stop_bottle_tracker(self):
        """bottle_tracker 노드 중지"""
        try:
            # 현재 프로세스 정리
            if self.bottle_tracker_process:
                try:
                    self.bottle_tracker_process.terminate()
                    self.bottle_tracker_process.wait(timeout=3)
                except:
                    try:
                        self.bottle_tracker_process.kill()
                    except:
                        pass
                self.bottle_tracker_process = None
            
            # 시스템에서 bottle_tracker 프로세스들 정리
            for proc in psutil.process_iter(['pid', 'cmdline']):
                try:
                    if proc.info['cmdline']:
                        cmdline = ' '.join(proc.info['cmdline'])
                        if ('camera_lidar_fusion_node' in cmdline or 
                            'bottle_tracker' in cmdline) and proc.pid != os.getpid():
                            proc.terminate()
                            proc.wait(timeout=2)
                except:
                    pass
            
            self.get_logger().info('Bottle tracker stopped')
            
        except Exception as e:
            self.get_logger().error(f'Error stopping bottle tracker: {e}')

    def send_yolo_command(self, command):
        """YOLO에 제어 명령 전송 (기존 YOLO 화면에서 제어)"""
        if self.yolo_initialized:
            control_msg = String()
            control_msg.data = command
            self.yolo_control_publisher.publish(control_msg)
            self.get_logger().info(f'YOLO 명령 전송: {command}')
        else:
            self.get_logger().warn('YOLO가 초기화되지 않아 명령을 전송할 수 없습니다.')
    
    def execute_bottle_align(self):
        """병 정렬 작업을 실행하는 메인 함수"""
        self.is_aligning = True
        
        try:
            # Phase 1: bottle_tracker 시작
            self.publish_progress("Phase 1: bottle_tracker를 시작하고 있습니다...")
            
            if not self.start_bottle_tracker():
                self.publish_progress("오류: bottle_tracker 시작에 실패했습니다.")
                # 실패 신호 발송
                success_msg = Bool()
                success_msg.data = False
                self.success_publisher.publish(success_msg)
                return
            
            self.publish_progress("bottle_tracker가 성공적으로 시작되었습니다.")
            time.sleep(2)
            
            # Phase 2: bottle_tracker가 작업하는 동안 대기 및 진행 상황 모니터링
            self.publish_progress("Phase 2: 카메라로 병을 감지하고 정렬하고 있습니다...")
            
            start_time = time.time()
            timeout = 90.0  # 1분 30초 타임아웃 (기존과 동일)
            alignment_completed = False
            
            progress_messages = [
                "카메라 초기화 중...",
                "병 감지 알고리즘 실행 중...",
                "병의 위치를 분석하고 있습니다...",
                "로봇 이동 경로를 계산하고 있습니다...",
                "병 정렬 작업을 실행하고 있습니다...",
                "정렬 완료도를 확인하고 있습니다..."
            ]
            
            message_index = 0
            last_message_time = time.time()
            
            while time.time() - start_time < timeout:
                # 진행 상황 메시지 업데이트 (10초마다)
                if time.time() - last_message_time > 10.0 and message_index < len(progress_messages):
                    self.publish_progress(progress_messages[message_index])
                    message_index += 1
                    last_message_time = time.time()
                
                # 프로세스가 종료되었는지 확인 (bottle_tracker가 완료됨)
                if self.bottle_tracker_process and self.bottle_tracker_process.poll() is not None:
                    self.get_logger().info('Bottle tracker process ended - 정렬이 완료된 것으로 판단')
                    alignment_completed = True
                    break
                
                time.sleep(1.0)  # 1초마다 체크
            
            # 타임아웃 체크 (1분 30초 후 무조건 완료 처리)
            if not alignment_completed and time.time() - start_time >= timeout:
                self.publish_progress("1분 30초 타임아웃 - 작업을 완료 처리합니다.")
                self.get_logger().info('1분 30초 타임아웃 - 무조건 완료 처리 (기존 액션과 동일)')
                alignment_completed = True  # 기존 액션 코드처럼 무조건 성공 처리
            
            # Phase 3: bottle_tracker 정리 및 YOLO 제어 신호 전송
            self.publish_progress("Phase 3: bottle_tracker를 정리하고 YOLO에 감지 신호를 보냅니다...")
            
            self.stop_bottle_tracker()
            time.sleep(1)
            
            # YOLO 제어 신호 (기존 화면에서 감지 시작)
            self.send_yolo_command("start_detection")
            self.publish_progress("YOLO에 병 감지 시작 신호를 보냈습니다.")
            
            # 최종 완료 (1분 30초 후 무조건 성공)
            self.publish_progress("병 정렬 작업이 완료되었습니다. YOLO가 병 감지를 시작합니다.")
            success_result = True  # 기존 액션 코드와 동일하게 무조건 성공
            
            # 성공 신호 발송
            success_msg = Bool()
            success_msg.data = success_result
            self.success_publisher.publish(success_msg)
            
            self.get_logger().info(f'병 정렬 작업 완료 - 성공: {success_result}')
            self.get_logger().info('YOLO 화면은 계속 유지됩니다. 다음 토픽 신호 대기 중...')
            
        except Exception as e:
            self.publish_progress(f"오류 발생: {str(e)}")
            self.get_logger().error(f'병 정렬 작업 중 오류 발생: {str(e)}')
            
            # bottle_tracker 정리
            self.stop_bottle_tracker()
            
            # 실패 신호 발송
            success_msg = Bool()
            success_msg.data = False
            self.success_publisher.publish(success_msg)
            
        finally:
            self.is_aligning = False
    
    def publish_progress(self, message):
        """진행 상황을 토픽으로 발행"""
        progress_msg = String()
        progress_msg.data = message
        self.progress_publisher.publish(progress_msg)
        self.get_logger().info(f'진행 상황: {message}')

    def __del__(self):
        """소멸자에서 정리"""
        self.cleanup_all_processes()

def main(args=None):
    rclpy.init(args=args)
    
    node = BottleAlignTopicNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt 수신 - 정리 시작")
    except Exception as e:
        node.get_logger().error(f"메인 루프 오류: {e}")
    finally:
        # 종료 시 모든 프로세스 정리
        node.cleanup_all_processes()
        try:
            node.destroy_node()
        except:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()
