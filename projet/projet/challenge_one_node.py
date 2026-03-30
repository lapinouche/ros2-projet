import rclpy
from rclpy.node import Node
import cv2
import numpy as np  # Added missing import
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist # Fixed import path
from rclpy.duration import Duration

from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# Define this outside or as a static method
def get_centroid(mask):
    M = cv2.moments(mask)
    if M['m00'] > 0:
        return int(M['m10'] / M['m00'])
    return None

class ChallangeOneNode(Node):
    def __init__(self):
        super().__init__('challange_one')
        
        # Subscriber & Publisher
        self.subscription = self.create_subscription(CompressedImage, '/image_raw/compressed', self.listener_callback, 10)
        #self.publisher_vel = self.create_publisher(Twist, '/cmd_vel', 10) # Fixed topic name to standard /cmd_vel

        # Replace your publisher line with this:
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.publisher_vel = self.create_publisher(Twist, '/cmd_vel', qos_profile)

        # Timer setup (ONLY set these once)
        self.start_time = self.get_clock().now()
        self.blind_duration = 3.0 
        self.get_logger().info("Robot is ready to move!")

        self.declare_parameter('use_sim_time', True)

    def listener_callback(self, msg):
        # FORCE MOVE TEST
        twist = Twist()
        twist.linear.x = 0.2  # Move forward
        twist.angular.z = 0.0
        self.publisher_vel.publish(twist)
        
        self.get_logger().info("I am sending a move command!")
        
def main(args=None):
    rclpy.init(args=args)
    node = ChallangeOneNode() # Match class name
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()