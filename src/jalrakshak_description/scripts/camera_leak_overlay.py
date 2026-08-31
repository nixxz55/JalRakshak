import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraLeakViewer(Node):
    def __init__(self):
        super().__init__('camera_leak_viewer')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10)
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Draw Red Alert Box and Text like in the screenshot
        cv2.rectangle(frame, (150, 200), (650, 280), (0, 0, 255), -1)
        cv2.putText(frame, "WATER LEAK DETECTED!", (170, 250), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 3, cv2.LINE_AA)

        cv2.imshow("JalRakshak Camera Feed", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    viewer = CameraLeakViewer()
    rclpy.spin(viewer)
    viewer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
