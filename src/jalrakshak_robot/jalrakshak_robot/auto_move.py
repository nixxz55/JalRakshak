import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class AutoMove(Node):

    def __init__(self):
        super().__init__('auto_move')

        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.start_time = self.get_clock().now()

        self.timer = self.create_timer(
            0.1,
            self.move_robot
        )

        self.started = False

        self.get_logger().info(
            'JalRakshak Auto Move Started!'
        )

        self.get_logger().info(
            'Waiting 5 seconds before starting...'
        )

    def move_robot(self):

        elapsed = (
            self.get_clock().now() - self.start_time
        ).nanoseconds / 1e9

        msg = Twist()

        if elapsed < 5.0:

            msg.linear.x = 0.0
            msg.angular.z = 0.0

        else:

            if not self.started:
                self.started = True

                self.get_logger().info(
                    '5 seconds completed!'
                )

                self.get_logger().info(
                    'JalRakshak rover moving forward...'
                )

            msg.linear.x = 0.15
            msg.angular.z = 0.0

        self.publisher.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = AutoMove()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        stop_msg = Twist()

        node.publisher.publish(stop_msg)

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()
