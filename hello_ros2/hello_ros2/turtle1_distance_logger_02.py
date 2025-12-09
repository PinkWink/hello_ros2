import math

import rclpy
from rclpy.node import Node

from turtlesim.msg import Pose
from hello_ros2_msgs.msg import TurtleDist   
from std_srvs.srv import Empty              


class TurtleDistanceLogger(Node):
    def __init__(self):
        super().__init__("turtle1_distance_logger")

        self.prev_x = None
        self.prev_y = None

        self.total_dist = 0.0

        self.pose_sub = self.create_subscription(
            Pose,
            "/turtle1/pose",
            self.pose_callback,
            10,
        )

        self.dist_pub = self.create_publisher(
            TurtleDist,
            "/turtle1/dist",
            10,
        )

        self.reset_srv = self.create_service(
            Empty,
            "/turtle1/reset_distance",
            self.reset_distance_callback,
        )

        self.clear_client = self.create_client(Empty, "/clear")

        self.get_logger().info("🐢 Turtle1 Distance Logger node started")

    def pose_callback(self, msg: Pose):
        x = msg.x
        y = msg.y

        if self.prev_x is None:
            self.prev_x = x
            self.prev_y = y
            self.total_dist = 0.0

            self.get_logger().info(
                f"Initial position set: x={x:.3f}, y={y:.3f}"
            )
            return

        dx = x - self.prev_x
        dy = y - self.prev_y
        step_dist = math.sqrt(dx * dx + dy * dy)

        self.total_dist += step_dist

        self.prev_x = x
        self.prev_y = y

        self.get_logger().info(
            f"Current x={x:.3f}, y={y:.3f}, "
            f"step_dist={step_dist:.4f}, total_dist={self.total_dist:.4f}"
        )

        dist_msg = TurtleDist()
        dist_msg.x = float(x)                   # 현재 x
        dist_msg.y = float(y)                   # 현재 y
        dist_msg.dist = float(self.total_dist)  # 시작 이후 누적 거리

        self.dist_pub.publish(dist_msg)

    def reset_distance_callback(self, request, response):
        if self.prev_x is None or self.prev_y is None:
            self.get_logger().warn(
                "Reset requested before any pose received. Just zeroing total_dist."
            )
            self.total_dist = 0.0

        else:
            self.total_dist = 0.0
            self.get_logger().info(
                f"Distance reset. New initial position: "
                f"x={self.prev_x:.3f}, y={self.prev_y:.3f}"
            )

        if not self.clear_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                "/clear service not available. Cannot clear turtlesim background."
            )
        else:
            self.get_logger().info("Calling /clear service to erase path...")
            req = Empty.Request()
            self.clear_client.call_async(req)

        self.get_logger().info("Total distance reset service call completed.")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TurtleDistanceLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
