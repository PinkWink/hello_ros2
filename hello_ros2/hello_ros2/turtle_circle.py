# 1번 실습
# cmd_vel 토픽을 발행하는 코드

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class TurtleCirclePublisher(Node):
    def __init__(self):
        super().__init__('turtle_circle_publisher')

        # cmd_vel Publisher (TurtleBot 기본 속도 명령)
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # 0.1초마다 publish하는 timer
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("Turtle circle publisher started!")

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 1.0     # 전진 속도
        msg.angular.z = 1.0    # 회전 속도 → 원을 그리게 됨

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleCirclePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
