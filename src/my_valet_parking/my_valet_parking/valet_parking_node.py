import rclpy
import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class ValetParkingNode(Node):
    def __init__(self):
        super().__init__("valet_parking_node")

        # 구독 설정
        self.cmd_sub = self.create_subscription(
            Twist, "/key_vel", self.key_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )

        # 상태 변수
        self.current_x = 0.0
        self.current_y = 0.0
        self.saved_x = 0.0
        self.saved_y = 0.0
        self.is_saved = False  # 위치가 저장되었는지 확인하는 플래그

        self.get_logger().info("4단계: 위치 기억 및 거리 측정 노드 가동!")

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # 위치가 저장된 상태라면, 저장된 지점과의 거리를 실시간 계산합니다.
        if self.is_saved:
            # 유클리드 거리 계산: sqrt((x2-x1)^2 + (y2-y1)^2)
            distance = math.sqrt(
                (self.saved_x - self.current_x) ** 2
                + (self.saved_y - self.current_y) ** 2
            )
            # 1초에 한 번 정도만 출력되도록 로깅 (너무 많이 찍히지 않게)
            if int(self.current_x * 10) % 10 == 0:  # 간단한 출력 조절 예시
                self.get_logger().info(f"원래 위치로부터 거리: {distance:.2f}m")

    def key_callback(self, msg):
        # 전진 키(linear.x > 0)를 누르는 순간을 '위치 저장' 시점으로 잡습니다.
        if msg.linear.x > 0.1 and not self.is_saved:
            self.saved_x = self.current_x
            self.saved_y = self.current_y
            self.is_saved = True
            self.get_logger().info(
                f"★★★ 주차 위치 저장 완료! ({self.saved_x:.2f}, {self.saved_y:.2f})"
            )


def main(args=None):
    rclpy.init(args=args)
    node = ValetParkingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
