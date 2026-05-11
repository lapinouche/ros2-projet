import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
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

        # Make node check that it's her time to publish
        self.current_state = -1
        self.state_robot = self.create_subscription(Int32, '/robot_state', self.state_callback, 10)
        self.start = True

        # 3. Talk to the actual robot
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # PD Variables
        self.prev_error = 0.0
        # Gains
        self.kp = 1.5
        self.kd = 0.5

    def state_callback(self, msg):
        self.current_state = msg.data

    def scan_callback(self, msg):
        if self.current_state not in [2]:
            return
        
        twist = Twist()
        max_range = 0.5 # 0.8
        min_range = 0.1

        # 1. Standard Side Distances
        # We look slightly forward-left and forward-right
        left_side = [l for l in msg.ranges[30:70] if min_range < l < max_range]
        right_side = [r for r in msg.ranges[290:330] if min_range < r < max_range]

        dist_left = sum(left_side) / len(left_side) if left_side else max_range
        dist_right = sum(right_side) / len(right_side) if right_side else max_range

        # 2. Front Check (The "Wall is coming" sensor)
        front_check = [f for f in msg.ranges[350:359] + msg.ranges[0:10] if min_range < f < 0.25]

        #self.get_logger().info(f"fc : {len(front_check)}; dl : {dist_left}; dr : {dist_right}")
        
        #if dist_left == max_range and dist_right == max_range :
        if self.start == True:
            twist.linear.x = 0.1
            error = 0.0

            if len(front_check) > 0:
                self.start = False

        elif len(front_check) > 10: # 3
            # We see a wall ahead! Reduce forward speed and increase the turning 'error' manually
            twist.linear.x = 0.03  # Slow down to navigate the turn
            error = -(dist_right - dist_left) + 0.2 # We force a left turn bias because we know the corner is left 
            #kp = 1.8 # (1.8) Stronger gain for the corner

        else:
            # Standard corridor following
            twist.linear.x = 0.05
            error = dist_left - dist_right # dist_left - dist_right
            #kp = 1.2
        
        # --- PD Logic ---
        derivative = error - self.prev_error
        twist.angular.z = (self.kp * error) + (self.kd * derivative)
        self.prev_error = error
        # 3. Apply movement
        self.get_logger().info(f"sending value to cmd_vel : x = {twist.linear.x} y = {twist.angular.z}")
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = CorridorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
