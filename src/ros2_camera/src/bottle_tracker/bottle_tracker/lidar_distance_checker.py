#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import math

class LidarDistanceChecker(Node):
    def __init__(self):
        super().__init__('lidar_distance_checker')

        # QoS 설정: BEST_EFFORT로 설정해야 /scan과 호환
        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT

        # 라이다 구독
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            qos
        )

        self.count = 0
        self.get_logger().info("라이다 정면 거리 측정기 시작!")

    def lidar_callback(self, msg):
        """라이다로 정면 거리 측정"""
        self.count += 1

        if len(msg.ranges) == 0:
            self.get_logger().info("라이다 데이터 없음")
            return

        # 처음 5번 라이다 정보 출력
        if self.count <= 5:
            self.get_logger().info("라이다 정보:")
            self.get_logger().info(f"  - 총 포인트: {len(msg.ranges)}")
            self.get_logger().info(f"  - 각도 범위: {math.degrees(msg.angle_min):.1f}도 ~ {math.degrees(msg.angle_max):.1f}도")
            self.get_logger().info(f"  - 각도 증분: {math.degrees(msg.angle_increment):.2f}도")
            self.get_logger().info(f"  - 거리 범위: {msg.range_min:.2f}m ~ {msg.range_max:.2f}m")

        # 정면 거리 (약 -15도 ~ +15도)
        front_distances = []
        front_data = []

        for i, distance in enumerate(msg.ranges):
            angle_rad = msg.angle_min + i * msg.angle_increment
            angle_deg = math.degrees(angle_rad)

            # 각도 정규화
            while angle_deg > 180:
                angle_deg -= 360
            while angle_deg < -180:
                angle_deg += 360

            # 유효 거리 & 정면 범위 필터링
            if msg.range_min <= distance <= msg.range_max:
                if 165 <= angle_deg <= 180:
                    front_distances.append(distance)
                    front_data.append(f"{angle_deg:.0f}도:{distance:.3f}m")

        # 결과 출력
        if front_distances:
            min_distance = min(front_distances)
            avg_distance = sum(front_distances) / len(front_distances)

            # 10회마다 일부 상세 정보 출력
            if self.count % 10 == 0:
                self.get_logger().info(f"정면 스캔 일부 데이터: {front_data[:5]}")
                self.get_logger().info(f"정면 포인트 개수: {len(front_distances)}")

            self.get_logger().info(f"정면 거리 - 최소: {min_distance:.3f}m, 평균: {avg_distance:.3f}m")
        else:
            self.get_logger().info("정면에 유효한 라이다 데이터 없음")

        # 전체 라이다 통계 (30회마다)
        if self.count % 30 == 0:
            valid_distances = [d for d in msg.ranges if msg.range_min <= d <= msg.range_max]
            if valid_distances:
                closest_overall = min(valid_distances)
                self.get_logger().info(f"전체 통계 - 유효 포인트: {len(valid_distances)}/{len(msg.ranges)}, 가장 가까운 물체: {closest_overall:.3f}m")
            else:
                self.get_logger().info("전체 라이다에서 유효한 데이터 없음")

def main(args=None):
    rclpy.init(args=args)
    node = LidarDistanceChecker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("종료")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

