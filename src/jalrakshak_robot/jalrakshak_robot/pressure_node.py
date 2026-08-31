import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry

class PressureNode(Node):
    def __init__(self):
        super().__init__('pressure_node')
        
        # Publisher for Pressure topic
        self.publisher = self.create_publisher(Float32, '/pressure', 10)
        
        # Subscriber for Odometry (to track rover distance)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Timer to check and publish every 1 second
        self.timer = self.create_timer(1.0, self.publish_pressure)
        
        self.pressure = 5.0  # Normal pipe pressure in Bar
        self.current_x = 0.0 # Rover starting position
        self.leak_detected = False

    def odom_callback(self, msg):
        # Update current X distance from Gazebo
        self.current_x = msg.pose.pose.position.x

    def publish_pressure(self):
        # Simulate Leak: If rover travels beyond 2.0 meters, pressure drops!
        if self.current_x > 2.0 and not self.leak_detected:
            self.pressure = 2.0  # Pressure drop due to leak
            self.leak_detected = True
            self.get_logger().warn(f'⚠️ LEAK DETECTED at distance X: {self.current_x:.2f} meters! Pressure dropped to {self.pressure} Bar ⚠️')
        
        elif not self.leak_detected:
            self.get_logger().info(f'Normal Pipe... Distance: {self.current_x:.2f} m, Pressure: {self.pressure} Bar')
        
        else:
            self.get_logger().warn(f'⚠️ CONTINUOUS LEAK! Location: {self.current_x:.2f} m, Pressure: {self.pressure} Bar')

        # Publish the pressure value
        msg = Float32()
        msg.data = self.pressure
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PressureNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
