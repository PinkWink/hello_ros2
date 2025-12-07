import math

import rclpy
from rclpy.node import Node

from turtlesim.msg import Pose
from hello_ros2_msgs.msg import TurtleDist  


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
        dist_msg.x = float(x)                 
        dist_msg.y = float(y)                
        dist_msg.dist = float(self.total_dist)  

        self.dist_pub.publish(dist_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleDistanceLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
