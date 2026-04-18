import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan # The specific message type for LiDAR

class CorridorNode(Node):
    def __init__(self):
        super().__init__('corridor_node')
        
        # Create the subscriber for LIDAR data
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',           # The default topic for TurtleBot LiDAR
            self.scan_callback,
            10                 # QoS history depth
        )

        # 3. Talk to the actual robot
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)


    def scan_callback(self, msg):
        max_range = 0.8 # Meters
        min_range = 0.3 # was 0.1

        left_side = msg.ranges[30:60] # was 85:95
        valid_left = [l for l in left_side if min_range < l < max_range]

        right_side = msg.ranges[300:330] # was 265:275
        valid_right = [r for r in right_side if min_range < r < max_range]

        """
        if not valid_left or not valid_right:
            self.get_logger().warn("wall out of visibility")
            return 
        """

        # If the LiDAR sees something very close (the cylinder), 
        # the Corridor node should SHUT UP and let the Obstacle node work.
        front_check = [f for f in msg.ranges[340:359] + msg.ranges[0:20] if f < 0.5]

        if len(front_check) > 5:
            # There is something right in front (the cylinder!)
            # Send 0.0 so the Monitor ignores the Corridor node
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            return

        twist = Twist()

        if valid_right or valid_left:
            if len(valid_left) != 0:
                dist_left = sum(valid_left) / len(valid_left)
            else:
                pass
                
            if len(valid_right) != 0:
                dist_right = sum(valid_right) / len(valid_right)
            else:
                pass

            twist.linear.x = 0.06
            # Proportional Control 
            error = dist_left - dist_right # If error is positive, we turn left. If negative, we turn right.
            kp = 0.6  # Gain: adjust this to make turning more or less aggressive
            twist.angular.z = error*kp
            
        else:
            twist.angular.z = 0.0
            twist.linear.x = 0.0
        
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = CorridorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
