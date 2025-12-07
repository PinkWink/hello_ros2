# 3번 파일
# turtlesim의 거북이 위치를 실시간으로 모니터링하는 코드

import math

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


class TurtlePoseNode(Node):
    def __init__(self):
        super().__init__("turtle_pose_live_node")

        self.subscription = self.create_subscription(
            Pose,
            "/turtle1/pose",
            self.pose_callback,
            10,
        )

        # 최신 pose 값을 저장할 변수
        self.x = 5.5
        self.y = 5.5
        self.theta = 0.0

        self.get_logger().info("TurtlePoseNode started! (subscribing /turtle1/pose)")

    def pose_callback(self, msg: Pose):
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta  # 라디안 단위

        # 디버그용 로그 (너무 시끄러우면 주석 처리)
        # self.get_logger().info(f"Pose: x={self.x:.2f}, y={self.y:.2f}, theta={self.theta:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = TurtlePoseNode()

    # --- Matplotlib Figure 세팅 ---
    fig, ax = plt.subplots()
    ax.set_title("TurtleSim Pose (Live Arrow)")
    ax.set_xlim(0, 11)   # turtlesim 기본 월드 크기
    ax.set_ylim(0, 11)
    ax.set_aspect("equal")
    ax.grid(True)

    # 처음에 화살표 하나 만들어놓고, 애니메이션에서 계속 업데이트
    # quiver(시작 x, 시작 y, 방향 x, 방향 y)
    arrow = ax.quiver(
        node.x,
        node.y,
        math.cos(node.theta),
        math.sin(node.theta),
        angles="xy",
        scale_units="xy",
        scale=1.0,
    )

    # 거북이 위치(점)도 같이 찍어보기 (선택)
    point, = ax.plot(node.x, node.y, "o")

    def update(frame):
        # ROS 콜백 한 번 처리 (spin_once)
        rclpy.spin_once(node, timeout_sec=0.01)

        # 현재 pose 값 읽기
        x = node.x
        y = node.y
        theta = node.theta

        # 방향 벡터 (길이는 1, 보기 좋게 scale로 나중에 조절 가능)
        dx = math.cos(theta)
        dy = math.sin(theta)

        # quiver 업데이트: 위치, 방향 갱신
        arrow.set_offsets([[x, y]])      # 화살표 시작점
        arrow.set_UVC(dx, dy)           # 방향 벡터

        # 점(현재 위치)도 이동
        point.set_data([x], [y])

        return arrow, point

    # FuncAnimation으로 주기적으로 update 호출
    ani = FuncAnimation(
        fig,
        update,
        interval=50,   # ms 단위 (50ms ≒ 20 FPS)
        blit=True,
    )

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("종료합니다.")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
