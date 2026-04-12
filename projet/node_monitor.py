import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class NodeMonitor(Node):
    def __init__(self):
        super().__init__('node_monitor')
        
        # Subscriptions
        self.line_sub = self.create_subscription(Twist, '/cmd_vel_line', self.line_callback, 10)
        self.obs_sub = self.create_subscription(Twist, '/cmd_vel_obstacle', self.obs_callback, 10)
        
        # Publisher to the actual robot hardware/simulator
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # State variables
        self.latest_line_twist = Twist()
        self.latest_obs_twist = Twist()
        
        # Timer to publish at a fixed rate (e.g., 10Hz)
        self.timer = self.create_timer(0.1, self.publish_decision) # 0.1 for 10Hz

    def line_callback(self, msg):
        self.latest_line_twist = msg

    def obs_callback(self, msg):
        self.latest_obs_twist = msg

    def publish_decision(self):
        out_msg = Twist()
        
        # PRIORITY LOGIC: 
        # If obstacle twist has linear velocity or angular velocity (meaning it wants to move/steer)
        # we prioritize it. Otherwise, follow the line.
        if abs(self.latest_obs_twist.linear.x) > 0.01 or abs(self.latest_obs_twist.angular.z) > 0.01:
            out_msg = self.latest_obs_twist
            # Optional: self.get_logger().info("Priority: Obstacle Avoidance")
        else:
            out_msg = self.latest_line_twist
            
        self.cmd_pub.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = NodeMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()