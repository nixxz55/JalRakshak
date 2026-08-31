import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry

from cv_bridge import CvBridge

import cv2
import numpy as np
import math


class JalRakshakSystem(Node):

    def __init__(self):
        super().__init__('jalrakshak_system')

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.camera_sub = self.create_subscription(
            Image,
            '/jalrakshak_camera/camera/image_raw',
            self.camera_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.bridge = CvBridge()

        self.start_time = self.get_clock().now()

        self.start_x = None
        self.start_y = None

        self.current_x = 0.0
        self.current_y = 0.0
        self.distance_travelled = 0.0

        self.leak_detected = False
        self.started = False

        self.timer = self.create_timer(
            0.1,
            self.control_loop
        )

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            'JALRAKSHAK SYSTEM STARTED'
        )

        self.get_logger().info(
            'Rover will wait for 5 seconds...'
        )

        self.get_logger().info(
            '========================================'
        )


    def odom_callback(self, msg):

        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        if self.start_x is None:

            self.start_x = self.current_x
            self.start_y = self.current_y

        self.distance_travelled = math.sqrt(
            (self.current_x - self.start_x) ** 2 +
            (self.current_y - self.start_y) ** 2
        )


    def camera_callback(self, msg):

        try:

            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )

        except Exception as e:

            self.get_logger().error(
                f'Camera error: {e}'
            )

            return


        hsv = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2HSV
        )

        lower_blue = np.array(
            [90, 80, 50]
        )

        upper_blue = np.array(
            [140, 255, 255]
        )

        mask = cv2.inRange(
            hsv,
            lower_blue,
            upper_blue
        )

        kernel = np.ones(
            (5, 5),
            np.uint8
        )

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            kernel
        )

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            kernel
        )

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        biggest_area = 0
        biggest_contour = None

        for contour in contours:

            area = cv2.contourArea(
                contour
            )

            if area > biggest_area:

                biggest_area = area
                biggest_contour = contour


        if biggest_area > 1500:

            x, y, w, h = cv2.boundingRect(
                biggest_contour
            )

            cv2.rectangle(
                frame,
                (x, y),
                (x + w, y + h),
                (0, 0, 255),
                3
            )

            cv2.putText(
                frame,
                'WATER LEAK DETECTED!',
                (x, max(y - 10, 30)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2
            )

            if not self.leak_detected:

                self.leak_detected = True

                self.get_logger().warn(
                    '========================================'
                )

                self.get_logger().warn(
                    '!!! WATER LEAK DETECTED !!!'
                )

                self.get_logger().warn(
                    f'LEAK LOCATION: '
                    f'{self.distance_travelled:.2f} meters'
                )

                self.get_logger().warn(
                    'ROVER STOPPED'
                )

                self.get_logger().warn(
                    '========================================'
                )


        cv2.imshow(
            'JalRakshak Camera',
            frame
        )

        cv2.waitKey(1)


    def control_loop(self):

        elapsed = (
            self.get_clock().now()
            - self.start_time
        ).nanoseconds / 1e9

        msg = Twist()

        if elapsed < 5.0:

            msg.linear.x = 0.0
            msg.angular.z = 0.0

            self.cmd_pub.publish(
                msg
            )

            return


        if self.leak_detected:

            msg.linear.x = 0.0
            msg.angular.z = 0.0

            self.cmd_pub.publish(
                msg
            )

            return


        if not self.started:

            self.started = True

            self.get_logger().info(
                '5 seconds completed!'
            )

            self.get_logger().info(
                'JalRakshak rover is moving...'
            )


        msg.linear.x = 0.15
        msg.angular.z = 0.0

        self.cmd_pub.publish(
            msg
        )


    def stop_robot(self):

        msg = Twist()

        msg.linear.x = 0.0
        msg.angular.z = 0.0

        self.cmd_pub.publish(
            msg
        )


def main(args=None):

    rclpy.init(
        args=args
    )

    node = JalRakshakSystem()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        node.get_logger().info(
            'JalRakshak system stopped by user.'
        )

    finally:

        node.stop_robot()

        cv2.destroyAllWindows()

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()
