import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import numpy as np
import cv2

# Use "ros2 launch projet2025 projet.launch.py x_pose:=-0.8 y_pose:=0.9 yaw_angle:=-1.57"

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        # Subscriptions
        self.sub_lidar = self.create_subscription(LaserScan, '/scan', self.lidar_cb, 10)
        self.sub_cam = self.create_subscription(Image, '/image_raw', self.cam_cb, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # State Variables (The "Bridge" between sensors)
        self.lidar_steering_adj = 0.0
        self.lidar_linear = 0.0
        self.line_steering = 0.0
        self.speed = 0.1
        self.obstacle_active = False
        self.avoidance_timeout = self.get_clock().now().nanoseconds / 1e9 + 1.5

        # line follow parameters
        self.last_state = "RV"
        self.last_error = 0
        self.bridge = CvBridge()

        # Timer: This runs the logic 10 times per second
        self.timer = self.create_timer(0.1, self.control_loop)

    def lidar_cb(self, msg):
        # Filter helper: returns the shortest valid distance in a zone
        def get_min_dist(zone):
            valid = [d for d in zone if 0.1 < d < 0.8] # 0.8m threshold
            return min(valid) if valid else float('inf')

        # Define strict detection zones (indices for TurtleBot3)
        front_ranges = msg.ranges[0:20] + msg.ranges[340:360] # Front: -20 to +20 degrees
        left_ranges = msg.ranges[20:70] # Left: 20 to 70 degrees
        right_ranges = msg.ranges[290:340] # Right: 290 to 340 degrees

        dist_front = get_min_dist(front_ranges)
        dist_left = get_min_dist(left_ranges)
        dist_right = get_min_dist(right_ranges)

        self.get_logger().info(f"dist_front = {dist_front}")
        self.get_logger().info(f"dist_left = {dist_left}")
        self.get_logger().info(f"dist_right = {dist_right}")

        # 3. Logic Priority (Emergency Stop -> Turn -> Cruise)
        if dist_front < 0.2:
            self.obstacle_active = True
            self.lidar_linear = -0.05
            # Decide to steer away
            self.lidar_steering_adj = 0.8 if dist_right < dist_left else -0.8 # Steer slightly left

        elif dist_front < 0.8:
            self.obstacle_active = True
            self.lidar_linear = 0.02
            self.lidar_steering_adj = 0.7 if dist_right < dist_left else -0.7
        
        elif dist_left < 0.5:
            self.obstacle_active = True
            self.lidar_linear = 0.05
            self.lidar_steering_adj = -0.4

        elif dist_right < 0.5:
            self.obstacle_active = True
            self.lidar_linear = 0.05
            self.lidar_steering_adj = 0.4

        else:
            self.obstacle_active = False
            self.lidar_steering_adj = 0.0
        
    def cam_cb(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w, _ = cv_image.shape
        #hsv_full = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # ROI : On prend une zone un peu plus haute pour voir venir la ligne
        roi = cv_image[int(h * 0.6):h, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # --- MASQUES HSV ---
        mask_red1 = cv2.inRange(hsv, (0, 100, 50), (10, 255, 255))
        mask_red2 = cv2.inRange(hsv, (160, 100, 50), (180, 255, 255))
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)

        mask_green = cv2.inRange(hsv, (35, 50, 50), (90, 255, 255))

        # Moments
        M_red = cv2.moments(mask_red)
        M_green = cv2.moments(mask_green)

        kp = 0.0015 # 0.02 Proportional gain
        kd = 0.025 # 0.01 Derivate gain

        # CAS 1 : On voit les deux lignes
        if M_red['m00'] > 2000 and M_green['m00'] > 2000:
            cx_red = int(M_red['m10'] / M_red['m00'])
            cx_green = int(M_green['m10'] / M_green['m00'])
            cible = (cx_red + cx_green) / 2
            error = cible - (w / 2)
                
            self.line_steering = -(float(error) / 100.0) # 100.0

        # CAS 2 : On voit seulement la ligne rouge (Le vert a disparu)
        elif M_red['m00'] > 2000:
            cx_red = int(M_red['m10'] / M_red['m00'])
            cy_red = int(M_red['m01'] / M_red['m00'])

            if self.last_state != "R":
                self.last_error = cx_red - 80

            self.last_state = "R"

            error = cx_red - 80
            derivative = error - self.last_error

            if cy_red > 40:
                severity = (cy_red - 38)/20.0
                self.line_steering = 0.7 + (0.5*severity)
            else:
                pd_steering = -(error*kp + derivative*kd)
                self.line_steering = pd_steering 
                self.last_error = error
            
        # CAS 3 : On voit seulement la ligne verte (Le rouge a disparu)
        elif M_green['m00'] > 2000:
            cx_green = int(M_green['m10'] / M_green['m00'])
            cy_green = int(M_green['m01'] / M_green['m00'])

            if self.last_state != "V":
                self.last_error = cx_green - 80

            self.last_state = "V"
            error = cx_green - 80
            derivative = error - self.last_error

            if cy_green > 40:
                severity = (cy_green - 38)/20.0
                self.line_steering = -(0.7 + (0.5*severity))
            else:
                pd_steering = -(error*kp + derivative*kd)
                self.line_steering = pd_steering 
                self.last_error = error
            

        else:
            self.get_logger().info("Lines lost - searching")
            self.line_steering *= 0.9

    def control_loop(self):
        twist = Twist()
        now = self.get_clock().now().nanoseconds / 1e9
        max_turn_speed = 0.5

        is_avoiding = self.obstacle_active or (now < self.avoidance_timeout)

        if is_avoiding: #self.obstacle_active
            # PRIORITY: Obstacle avoidance
            # We combine the line following AND a nudge from LiDAR
            combined_steering = (self.line_steering*0.5) + self.lidar_steering_adj
            twist.angular.z = max(min(combined_steering, max_turn_speed), -max_turn_speed)
            if self.lidar_linear:
                twist.linear.x = self.lidar_linear
            else:
                twist.linear.x = 0.05 
            
        else:
            if self.last_state == "R":
                twist.linear.x = 0.05   # 0.15
                twist.angular.z = self.line_steering - 0.1
            elif self.last_state == "V":
                twist.linear.x = 0.05   # 0.15
                twist.angular.z = self.line_steering + 0.1
            else:                       #self.last_state == "RV"
                twist.linear.x = 0.08
                twist.angular.z = self.line_steering

        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
