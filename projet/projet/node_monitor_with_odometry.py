import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
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
            (1.0, 1.5, 0.010, "LINE"), # 1.244, 1.5
            (-0.4, 1.5, 0.010, "OBSTACLE"), #  0.3, 1.9
            (-0.4, 0.6, None, "CORRIDOR"),  # -0.4, 1.5 Example placeholder
            (-0.2, 0.9, None, "LINE"),
            (99.0, 99.0, None, "MOTION") # X and Y to change
        ]
        # (1.216, 1.300, 0.010, "obstacle_avoidance.py"),
        #(-0.2, 0.5, None, "MOTION") # X and Y to change

        self.current_step = 0
        self.active_process = None
        self.threshold = 0.3  # (0.15) Distance tolerance in meters

        # State variables
        self.latest_line_twist = Twist()
        self.latest_obs_twist = Twist()
        self.latest_corr_twist = Twist()
        self.latest_hmc_twist = Twist()

        # Subscriptions
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10) # Subscribe to Odometry to track robot position
        self.line_sub = self.create_subscription(Twist, '/cmd_vel_line', self.line_callback, 10)
        self.obs_sub = self.create_subscription(Twist, '/cmd_vel_obstacle', self.obs_callback, 10)
        self.corr_sub = self.create_subscription(Twist, '/cmd_vel_corridor', self.corr_callback, 10)
        self.hmc_sub = self.create_subscription(Twist, '/cmd_vel_hmc', self.hmc_callback, 10)
        self.cam_sub = self.create_subscription(CompressedImage, '/image_raw/compressed', self.cam_callback, 10)

        # Publisher to the actual robot hardware/simulator
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_pub = self.create_publisher(Int32, '/robot_state', 10)
        
        # Timer to publish at a fixed rate (e.g., 10Hz)
        self.timer = self.create_timer(0.1, self.publish_decision) # 0.1 for 10Hz

    def odom_callback(self, msg):
        if self.current_step >= len(self.challenges):
            return

        pos = msg.pose.pose.position
        target_x, target_y, target_yaw, _ = self.challenges[self.current_step]

        # Calculate Euclidean distance to the target coordinate
        distance = math.sqrt((pos.x - target_x)**2 + (pos.y - target_y)**2)

        if distance < self.threshold:
            self.get_logger().info(f"Transitioning at distance: {distance:.2f}")
            self.current_step += 1

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

        if self.current_step < len(self.challenges):
            current_script = self.challenges[self.current_step][3]

            if "LINE" in current_script:
                self.get_logger().info("Priority: Line Following")
                out_msg = self.latest_line_twist

            elif "OBSTACLE" in current_script:
                self.get_logger().info("Priority: Obstacle Avoidance")
                out_msg = self.latest_obs_twist
            
            elif "CORRIDOR" in current_script:
                self.get_logger().info("Priority: Corridor")
                out_msg = self.latest_corr_twist
            
            elif "MOTION" in current_script:
                self.get_logger().info("Last challange !")
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
