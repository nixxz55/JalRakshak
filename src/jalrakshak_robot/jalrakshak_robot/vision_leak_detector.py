import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np


class WaterLeakDetector(Node):

    def __init__(self):
        super().__init__('water_leak_detector')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/jalrakshak_camera/camera/image_raw',
            self.image_callback,
            10
        )

        self.get_logger().info('Water Leak Detector Started!')
        self.get_logger().info('Waiting for camera image...')

    def image_callback(self, msg):

        try:
            # ROS Image -> OpenCV Image
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )

        except Exception as e:
            self.get_logger().error(
                f'Image conversion failed: {e}'
            )
            return

        # Convert BGR -> HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # BLUE color range
        lower_blue = np.array([90, 80, 50])
        upper_blue = np.array([140, 255, 255])

        # Create blue mask
        mask = cv2.inRange(
            hsv,
            lower_blue,
            upper_blue
        )

        # Remove small noise
        kernel = np.ones((5, 5), np.uint8)

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

        # Find blue objects
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        leak_detected = False

        for contour in contours:

            area = cv2.contourArea(contour)

            # Minimum blue area
            if area > 1500:

                x, y, w, h = cv2.boundingRect(contour)

                # Red bounding box
                cv2.rectangle(
                    frame,
                    (x, y),
                    (x + w, y + h),
                    (0, 0, 255),
                    3
                )

                # Detection text
                cv2.putText(
                    frame,
                    'WATER LEAK DETECTED!',
                    (x, max(y - 10, 30)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 0, 255),
                    2
                )

                leak_detected = True

        # Terminal notification
        if leak_detected:

            self.get_logger().warn(
                '!!! WATER LEAK DETECTED !!!'
            )

        # Display camera
        cv2.imshow(
            'JalRakshak - Water Leak Detection',
            frame
        )

        # OpenCV window response
        cv2.waitKey(1)


def main(args=None):

    rclpy.init(args=args)

    node = WaterLeakDetector()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        cv2.destroyAllWindows()

        rclpy.shutdown()


if __name__ == '__main__':
    main()
