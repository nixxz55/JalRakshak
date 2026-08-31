import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry

from cv_bridge import CvBridge

import cv2
import numpy as np
import math
import time


class JalRakshakSystem(Node):

    def __init__(self):
        super().__init__('jalrakshak_system')

        # ========================================
        # ROS PUBLISHER
        # ========================================

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # ========================================
        # CAMERA SUBSCRIBER
        # ========================================

        self.camera_sub = self.create_subscription(
            Image,
            '/jalrakshak_camera/camera/image_raw',
            self.camera_callback,
            10
        )

        # ========================================
        # ODOMETRY SUBSCRIBER
        # ========================================

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # ========================================
        # OPENCV
        # ========================================

        self.bridge = CvBridge()

        # ========================================
        # START POSITION / TIME
        # ========================================

        self.start_time = self.get_clock().now()

        self.start_x = None
        self.start_y = None

        self.current_x = 0.0
        self.current_y = 0.0

        self.distance_travelled = 0.0

        # ========================================
        # MISSION STATE
        # ========================================

        self.started = False

        self.leak_detected = False

        # ========================================
        # LEAK INFORMATION
        # ========================================

        self.leak_count = 0

        self.leak_positions = [
            10.0,
            20.0,
            30.0,
            40.0,
            50.0
        ]

        self.last_leak_distance = -999.0

        # Minimum distance between two leak detections
        self.minimum_leak_distance = 3.0

        # ========================================
        # INSPECTION TIMER
        # ========================================

        self.inspection_start_time = None

        self.inspection_duration = 5.0

        # ========================================
        # MISSION COMPLETION
        # ========================================

        self.mission_completed = False

        # ========================================
        # TIMER
        # ========================================

        self.timer = self.create_timer(
            0.1,
            self.control_loop
        )

        # ========================================
        # STARTUP LOG
        # ========================================

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            '        JALRAKSHAK SYSTEM STARTED'
        )

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            'Rover will wait for 5 seconds...'
        )

        self.get_logger().info(
            'Automatic leak inspection mode enabled.'
        )

        self.get_logger().info(
            '========================================'
        )


    # ==================================================
    # ODOMETRY CALLBACK
    # ==================================================

    def odom_callback(self, msg):

        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Store initial rover position
        if self.start_x is None:

            self.start_x = self.current_x
            self.start_y = self.current_y

        # Calculate distance travelled from start
        self.distance_travelled = math.sqrt(
            (self.current_x - self.start_x) ** 2
            +
            (self.current_y - self.start_y) ** 2
        )


    # ==================================================
    # CAMERA CALLBACK
    # ==================================================

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

        # ========================================
        # BGR → HSV
        # ========================================

        hsv = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2HSV
        )

        # ========================================
        # BLUE COLOR RANGE
        # ========================================

        lower_blue = np.array([
            90,
            80,
            50
        ])

        upper_blue = np.array([
            140,
            255,
            255
        ])

        # ========================================
        # BLUE MASK
        # ========================================

        mask = cv2.inRange(
            hsv,
            lower_blue,
            upper_blue
        )

        # ========================================
        # REMOVE NOISE
        # ========================================

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

        # ========================================
        # FIND BLUE CONTOURS
        # ========================================

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

        # ========================================
        # WATER LEAK DETECTION
        # ========================================

        if biggest_area > 1500:

            x, y, w, h = cv2.boundingRect(
                biggest_contour
            )

            # Draw bounding box
            cv2.rectangle(
                frame,
                (x, y),
                (x + w, y + h),
                (0, 0, 255),
                3
            )

            # Display detection text
            cv2.putText(
                frame,
                'WATER LEAK DETECTED!',
                (x, max(y - 10, 30)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2
            )

            # ====================================
            # NEW LEAK EVENT
            # ====================================

            if not self.leak_detected:

                distance_from_last_leak = (
                    self.distance_travelled
                    -
                    self.last_leak_distance
                )

                # Prevent repeated detection
                # of the same leak
                if (
                    distance_from_last_leak
                    >
                    self.minimum_leak_distance
                ):

                    self.leak_count += 1

                    self.leak_detected = True

                    self.inspection_start_time = time.time()

                    self.last_leak_distance = (
                        self.distance_travelled
                    )

                    # ====================================
                    # EXPECTED LOCATION
                    # ====================================

                    if self.leak_count <= len(
                        self.leak_positions
                    ):

                        expected_location = (
                            self.leak_positions[
                                self.leak_count - 1
                            ]
                        )

                    else:

                        expected_location = (
                            self.distance_travelled
                        )

                    # ====================================
                    # POSITION ERROR
                    # ====================================

                    position_error = abs(
                        self.distance_travelled
                        -
                        expected_location
                    )

                    # ====================================
                    # PROFESSIONAL ALERT
                    # ====================================

                    self.get_logger().warn(
                        '========================================'
                    )

                    self.get_logger().warn(
                        '          JALRAKSHAK ALERT'
                    )

                    self.get_logger().warn(
                        '========================================'
                    )

                    self.get_logger().warn(
                        f'LEAK NUMBER       : '
                        f'{self.leak_count}'
                    )

                    self.get_logger().warn(
                        f'EXPECTED LOCATION: '
                        f'{expected_location:.2f} meters'
                    )

                    self.get_logger().warn(
                        f'DETECTED LOCATION: '
                        f'{self.distance_travelled:.2f} meters'
                    )

                    self.get_logger().warn(
                        f'POSITION ERROR    : '
                        f'{position_error:.2f} meters'
                    )

                    self.get_logger().warn(
                        'STATUS            : LEAK CONFIRMED'
                    )

                    self.get_logger().warn(
                        'ROVER ACTION      : STOPPED'
                    )

                    self.get_logger().warn(
                        'INSPECTION TIME   : 5 seconds'
                    )

                    self.get_logger().warn(
                        '========================================'
                    )

        # ========================================
        # CAMERA WINDOW
        # ========================================

        cv2.imshow(
            'JalRakshak Camera',
            frame
        )

        cv2.waitKey(1)


    # ==================================================
    # CONTROL LOOP
    # ==================================================

    def control_loop(self):

        # ========================================
        # ELAPSED TIME
        # ========================================

        elapsed = (
            self.get_clock().now()
            -
            self.start_time
        ).nanoseconds / 1e9

        msg = Twist()

        # ========================================
        # 5 SECOND START DELAY
        # ========================================

        if elapsed < 5.0:

            msg.linear.x = 0.0
            msg.angular.z = 0.0

            self.cmd_pub.publish(
                msg
            )

            return

        # ========================================
        # START MISSION
        # ========================================

        if not self.started:

            self.started = True

            self.get_logger().info(
                '5 seconds completed!'
            )

            self.get_logger().info(
                'JalRakshak rover is moving...'
            )

            self.get_logger().info(
                'Automatic leak detection active.'
            )

        # ========================================
        # MISSION COMPLETED
        # ========================================

        if self.mission_completed:

            msg.linear.x = 0.0
            msg.angular.z = 0.0

            self.cmd_pub.publish(
                msg
            )

            return

        # ========================================
        # LEAK INSPECTION MODE
        # ========================================

        if self.leak_detected:

            # Stop rover
            msg.linear.x = 0.0
            msg.angular.z = 0.0

            self.cmd_pub.publish(
                msg
            )

            # ====================================
            # INSPECTION TIMER
            # ====================================

            if (
                self.inspection_start_time
                is not None
            ):

                inspection_elapsed = (
                    time.time()
                    -
                    self.inspection_start_time
                )

                if (
                    inspection_elapsed
                    >=
                    self.inspection_duration
                ):

                    self.get_logger().info(
                        '========================================'
                    )

                    self.get_logger().info(
                        'Leak inspection completed.'
                    )

                    self.get_logger().info(
                        f'Leaks detected so far: '
                        f'{self.leak_count}'
                    )

                    self.get_logger().info(
                        'Rover continuing mission...'
                    )

                    self.get_logger().info(
                        '========================================'
                    )

                    self.leak_detected = False

                    self.inspection_start_time = None

                    # ====================================
                    # CHECK FINAL LEAK
                    # ====================================

                    if self.leak_count >= len(
                        self.leak_positions
                    ):

                        self.mission_completed = True

                        self.get_logger().info(
                            '========================================'
                        )

                        self.get_logger().info(
                            '       JALRAKSHAK MISSION COMPLETE'
                        )

                        self.get_logger().info(
                            '========================================'
                        )

                        self.get_logger().info(
                            f'Total leaks detected: '
                            f'{self.leak_count}'
                        )

                        self.get_logger().info(
                            f'Total distance travelled: '
                            f'{self.distance_travelled:.2f} m'
                        )

                        self.get_logger().info(
                            'All pipeline leak points inspected.'
                        )

                        self.get_logger().info(
                            'Rover stopped safely.'
                        )

                        self.get_logger().info(
                            '========================================'
                        )

            return

        # ========================================
        # NORMAL FORWARD MOVEMENT
        # ========================================

        msg.linear.x = 0.15
        msg.angular.z = 0.0

        self.cmd_pub.publish(
            msg
        )


    # ==================================================
    # STOP ROBOT
    # ==================================================

    def stop_robot(self):

        msg = Twist()

        msg.linear.x = 0.0
        msg.angular.z = 0.0

        self.cmd_pub.publish(
            msg
        )


# ======================================================
# MAIN
# ======================================================

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


# ======================================================
# PROGRAM ENTRY
# ======================================================

if __name__ == '__main__':

    main()
