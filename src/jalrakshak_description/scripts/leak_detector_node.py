import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class LeakDetectorNode(Node):
    def __init__(self):
        super().__init__('leak_detector_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info(' Jal Raksha Pipeline Inspection Initialized (Infinity Pipeline Mode)...')
        self.get_logger().info(' Waiting for system stabilization (5 seconds)...')
        
        # 5 Seconds Delay before moving
        time.sleep(5.0)
        
        self.get_logger().info(' Rover Starting Movement Inside Endless Pipeline...')
        self.timer = self.create_timer(1.0, self.detect_leaks_callback)
        self.elapsed_time = 0

    def detect_leaks_callback(self):
        # Move forward command
        msg = Twist()
        msg.linear.x = 0.25  # Speed
        self.publisher_.publish(msg)

        self.elapsed_time += 1
        
        # Calculate simulated distance (Speed 0.25 m/s * time)
        current_distance = self.elapsed_time * 0.25

        if 9.5 <= current_distance <= 10.5:
            self.get_logger().warn('🚨 [CRITICAL LEAK ALERT]: Heavy Water Leakage Detected at 10.0 Meters! (Pipeline Joint Crack)')
        elif current_distance >= 25.0:
            self.get_logger().info('🏁 Endless Pipeline Inspection Milestone Reached successfully.')

def main(args=None):
    rclpy.init(args=args)
    node = LeakDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
