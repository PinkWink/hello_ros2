# 1번 실습
# turtle1/pose 토픽을 구독

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose


class TurtlePoseSubscriber(Node):
    def __init__(self):
        super().__init__('turtle_pose_subscriber')

        # pose 토픽 구독
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        self.get_logger().info("Turtle pose subscriber started!")

    def pose_callback(self, msg: Pose):
        # Pose 메시지 출력
        print(
            f"Pose → x: {msg.x:.2f}, y: {msg.y:.2f}, "
            f"theta: {msg.theta:.2f}, linear_velocity: {msg.linear_velocity:.2f}, "
            f"angular_velocity: {msg.angular_velocity:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = TurtlePoseSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
