import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32
import subprocess
import math

class NodeMonitor(Node):
    def __init__(self):
        super().__init__('node_monitor')

        # Define your sequence of checkpoints and corresponding nodes
        # Format: (x, y, yaw, "node_script.py")
        self.challenges = [
            (1.0, 1.4, "LINE"), # 1.244, 1.5
            (0.1, 1.6, "OBSTACLE"), #   -0.35, 1.5 // 0.3, 1.9
            (-0.2, 1.4, "CORRIDOR"),  # -0.4, 1.5 Example placeholder
            (0.05, 0.9, "LINE"),
            (99.0, 99.0, "MOTION") # X and Y to change
        ]
        # (1.216, 1.300, 0.010, "obstacle_avoidance.py"),
        #(-0.2, 0.5, None, "MOTION") # X and Y to change

        self.current_step = 0
        self.active_process = None
        self.in_exit_zone = False
        self.in_entrance_zone = False
        #self.threshold = 0.3  # (0.15) Distance tolerance in meters

        # State variables
        self.latest_line_twist = Twist()
        self.latest_obs_twist = Twist()
        self.latest_corr_twist = Twist()
        self.latest_hmc_twist = Twist()

        # Subscriptions
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10) # Subscribe to Odometry to track robot position
        self.lidar_sub = self.create_subscription(LaserScan, '/scan',  self.lidar_callback, 10)
        self.line_sub = self.create_subscription(Twist, '/cmd_vel_line', self.line_callback, 10)
        self.obs_sub = self.create_subscription(Twist, '/cmd_vel_obstacle', self.obs_callback, 10)
        self.corr_sub = self.create_subscription(Twist, '/cmd_vel_corridor', self.corr_callback, 10)
        self.hmc_sub = self.create_subscription(Twist, '/cmd_vel_hmc', self.hmc_callback, 10)
        self.cam_sub = self.create_subscription(CompressedImage, '/camera/image_raw/compressed', self.cam_callback, 10)

        # Publisher to the actual robot hardware/simulator
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_pub = self.create_publisher(Int32, '/robot_state', 10)
        
        # Timer to publish at a fixed rate (e.g., 10Hz)
        self.timer = self.create_timer(0.1, self.publish_decision) # 0.1 for 10Hz

    def lidar_callback(self, msg):
        if self.current_step not in [1, 2]:
            return
        
        if self.in_entrance_zone:
            left_side = [l for l in msg.ranges[30:70] if 0.1 < l < 0.5]
            right_side = [r for r in msg.ranges[290:330] if 0.1 < r < 0.5]

            if left_side and right_side:
                self.current_step = 2
                self.in_entrance_zone = False

        if self.in_exit_zone:
            left_side = [l for l in msg.ranges[30:70] if 0.1 < l < 0.5]
            right_side = [r for r in msg.ranges[290:330] if 0.1 < r < 0.5]

            if not left_side and not right_side:
                self.current_step = 3
                self.in_exit_zone = False

    def odom_callback(self, msg):
        if self.current_step >= len(self.challenges):
            return

        pos = msg.pose.pose.position
        next_step = False
        target_x, target_y, current_script = self.challenges[self.current_step]

        # Calculate Euclidean distance to the target coordinate
        #distance = math.sqrt((pos.x - target_x)**2 + (pos.y - target_y)**2)

        if "LINE" in current_script and self.current_step == 0:
            if pos.y > target_y :
                self.current_step = 1
                # Filter helper: returns the shortest valid distance in a zone
                '''
                # Is not working (the turtlebot miss the corner !!!)
                def get_min_dist(zone):
                    valid = [d for d in zone if 0.1 < d < 0.8] # 0.8m threshold
                    return min(valid) if valid else float('inf')

                # Define strict detection zones (indices for TurtleBot3)
                front_ranges = msg.ranges[0:20] + msg.ranges[340:360] # Front: -20 to +20 degrees
                left_ranges = msg.ranges[20:70] # Left: 20 to 70 degrees

                dist_front = get_min_dist(front_ranges)
                dist_left = get_min_dist(left_ranges)

                if dist_front != 'inf' or dist_left != 'inf':
                    self.current_step = 1
                '''

        elif "OBSTACLE" in current_script:
            if pos.x < target_x :
                self.in_entrance_zone = True
                self.get_logger().info("start corridor entrance mode")
            
        elif "CORRIDOR" in current_script:
            if pos.x > target_x and pos.y < target_y:
                self.in_exit_zone = True
                self.get_logger().info("start corridor exit mode")

        elif "LINE" in current_script and self.current_step == 3:
            if pos.y < target_y :
                self.get_logger().info("last challanges !!!")
                #self.current_step = 4

    def line_callback(self, msg):
        self.latest_line_twist = msg

    def obs_callback(self, msg):
        self.latest_obs_twist = msg

    def corr_callback(self, msg):
        self.latest_corr_twist = msg

    def hmc_callback(self, msg):
        self.latest_hmc_twist = msg

    def cam_callback(self, msg): # camera callback to save the image
        self.latest_raw_image = msg

    def publish_decision(self):
        # ----- Movement Logic -----
        out_msg = Twist()

        # ----- State ----- (go on ros2 topic echo /robot_state to see it)
        state_msg = Int32()
        state_msg.data = self.current_step # accept only integer !
        self.state_pub.publish(state_msg)

        if self.current_step < len(self.challenges):
            current_script = self.challenges[self.current_step][2]

            if "LINE" in current_script:
                #self.get_logger().info("Priority: Line Following")
                out_msg = self.latest_line_twist

            elif "OBSTACLE" in current_script:
                #self.get_logger().info("Priority: Obstacle Avoidance")
                out_msg = self.latest_obs_twist
            
            elif "CORRIDOR" in current_script:
                #self.get_logger().info("Priority: Corridor")
                out_msg = self.latest_corr_twist
            
            elif "MOTION" in current_script:
                #self.get_logger().info("Last challange !")
                out_msg = self.latest_hmc_twist

            # Log only occasionally to avoid flooding the terminal
            self.get_logger().info(f"Step {self.current_step} active: {current_script}", once=True)

        else:
            # Safety stop if out of bounds
            out_msg = Twist()

        # publish the message
        self.state_pub.publish(state_msg)
        self.cmd_pub.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = NodeMonitor()

    # Using MultiThreadedExecutor is great for keeping the UI and Logic separate
    #executor = MultiThreadedExecutor()
    #executor.add_node(node)

    try: 
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
