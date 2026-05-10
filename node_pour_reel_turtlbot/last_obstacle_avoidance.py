import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import numpy as np
import cv2

# Use "ros2 launch projet2025 projet.launch.py x_pose:=-0.8 y_pose:=0.9 yaw_angle:=-1.57"

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        # Subscriptions
        self.sub_cam = self.create_subscription(CompressedImage, '/image_raw/compressed', self.cam_cb, 10)
        #self.sub_cam = self.create_subscription(CompressedImage, '/camera/image_raw/compressed',  self.cam_cb, 10)

        self.sub_lidar = self.create_subscription(LaserScan, '/scan', self.lidar_cb, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # State Variables (The "Bridge" between sensors)
        self.lidar_linear = 0.0
        self.lidar_angular = 0.0
        self.line_angular = 0.0
        self.speed = 0.1
        self.obstacle_active = False
        self.avoidance_timeout = self.get_clock().now().nanoseconds / 1e9 + 1.5

        # line follow parameters
        self.last_state = "RV"
        self.last_error = 1

        # Make node check that it's her time to publish
        self.current_state = 1
        self.state_robot = self.create_subscription(Int32, '/robot_state', self.state_callback, 10)

        self.target_dist = 0.1 # 0.1 (good esquive on the first cylinder for 0.3) The ideal distance from the cylinder in meters
        self.kp_lidar = 1.5      # Proportional gain for lidar steering

        self.smoothed_lidar_angular = 0.0
        self.alpha = 0.3 # Smoothing factor (0.0 to 1.0). Lower is smoother/slower.
        
        self.between_obstacle = False
        self.once = False # to get in between_obstacle mode only once
        self.last_dist_left = float('inf')

        self.end_obstacle = False

        # Timer: This runs the logic 10 times per second
        self.timer = self.create_timer(0.1, self.control_loop)

    def state_callback(self, msg):
        self.current_state = msg.data

    def lidar_cb(self, msg):
        if self.current_state not in [1]:
            #self.get_logger().info("exit current state not valid")
            return
        # Filter helper: returns the shortest valid distance in a zone
        def get_min_dist(zone):
            valid = [d for d in zone if 0.1 < d < 0.6] # 0.6m / 0.8m threshold
            if not valid:
                return float('inf')
            # Sort and take the average of the 5 closest points
            valid.sort()
            n_points = min(len(valid), 5) 
            avg_min = sum(valid[:n_points]) / n_points
            return avg_min

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
        if dist_front < 0.08 or dist_left < 0.08 or dist_right < 0.08 :
            #self.get_logger().info("too close")
            self.lidar_linear = -0.02
            self.lidar_angular = 0.0
        
        elif dist_front < 0.2:
            #self.get_logger().info("dist_front < 0.2")
            self.lidar_linear = 0.02
            self.lidar_angular = 1.2 if dist_right < dist_left else -1.2 

        elif dist_front < 0.4:
            #self.get_logger().info("dist_front < 0.6")
            self.lidar_linear = 0.04
            self.lidar_angular = 0.6 if dist_right < dist_left else -0.6
        
        elif dist_left < 0.6:  # Detecting cylinder on the left
            #self.get_logger().info(f"Centering from Left: {dist_left:.2f}")
            self.lidar_linear = 0.05
            # Error is (Actual - Target). 
            # If dist_left is 0.2 (too close), error is -0.15 -> steer right (negative)
            # If dist_left is 0.5 (too far), error is +0.15 -> steer left (positive)
            error = dist_left - self.target_dist
            self.lidar_angular = error * self.kp_lidar

        elif dist_right < 0.6: # Detecting cylinder on the right
            #self.get_logger().info(f"Centering from Right: {dist_right:.2f}")
            self.lidar_linear = 0.05
            # Error is (Actual - Target).
            # If dist_right is 0.2 (too close), error is -0.15 -> steer left (positive)
            error = dist_right - self.target_dist
            #self.lidar_angular = -(error * self.kp_lidar)

            target_angular = error * self.kp_lidar # Your calculated steering
            self.smoothed_lidar_angular = (self.alpha * target_angular) + ((1 - self.alpha) * self.smoothed_lidar_angular)

            # Then use the smoothed value for your state
            self.lidar_angular = self.smoothed_lidar_angular

        else:
            self.obstacle_active = False
            #self.get_logger().info(f"lidar_linear: {self.lidar_linear:.2f}")
            #self.get_logger().info(f"lidar_angular : {self.lidar_angular:.2f}")
            return

        self.obstacle_active = True

        if self.last_dist_left != float('inf') and dist_left == float('inf') and not self.once:
            self.get_logger().info("between obstcale true !")
            self.between_obstacle = True
            self.once = True

        if self.between_obstacle and (dist_right < 0.25 or dist_front < 0.25): # (dist_right < 0.2 or dist_front < 0.2):  # dist_front < 0.2 :
            self.get_logger().info("between obstcale false !")
            self.between_obstacle = False
            # Give the camera a moment to stabilize before lidar takes over
            #self.avoidance_timeout = self.get_clock().now().nanoseconds / 1e9 + 0.5

        if self.once and not self.between_obstacle and dist_right < 0.15:
            self.end_obstacle = True
            self.get_logger().info("ending obstacle avoidance")

        self.last_dist_left = dist_left
        #self.get_logger().info(f"lidar_linear: {self.lidar_linear:.2f}")
        #self.get_logger().info(f"lidar_angular : {self.lidar_angular:.2f}")
        
    def cam_cb(self, msg):
        if self.current_state not in [1]:
            return

        #cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        h, w, _ = cv_image.shape
        center_view = w / 2
        #hsv_full = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # ROI : On prend une zone un peu plus haute pour voir venir la ligne
        roi = cv_image[int(h * 0.6):h, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # --- MASQUES HSV ---
        # 1. Mask for the "Lower" Red (starts at 0)
        lower_red_low = np.array([0, 100, 70])    # Tighten S and V to block brown
        upper_red_low = np.array([10, 255, 255])
        # 2. Mask for the "Upper" Red (ends at 179)
        lower_red_high = np.array([160, 100, 70]) 
        upper_red_high = np.array([179, 255, 255])
        # 3. Combine them
        mask_low = cv2.inRange(hsv, lower_red_low, upper_red_low)
        mask_high = cv2.inRange(hsv, lower_red_high, upper_red_high)
        mask_red = cv2.bitwise_or(mask_low, mask_high)
        # Apply the combined mask
        # green mask : L - H : 60 / S : 30 / V : 80 and U - H : 140 / S : 255 / V : 160
        lower_green = np.array([60, 30, 80]) 
        upper_green = np.array([90, 255, 255])
        mask_green = cv2.inRange(hsv, lower_green, upper_green)

        # For the real robot : 
        #mask_red_upper = cv2.inRange(hsv, (100, 64, 50), (179, 255, 255)) # Mask for the "End" of the spectrum (where your values were)
        #mask_red_lower = cv2.inRange(hsv, (0, 64, 50), (10, 255, 255)) # Mask for the "Beginning" of the spectrum (Standard Red)            
        #mask_red = cv2.bitwise_or(mask_red_lower, mask_red_upper) # Combine them
        #mask_green = cv2.inRange(hsv, (11, 67, 55), (95, 255, 255))

        # Moments
        M_red = cv2.moments(mask_red)
        M_green = cv2.moments(mask_green)

        # Camera Proportional Gain
        # You'll need to tune this. Start small (e.g., 0.002 to 0.005)
        kp_cam = 0.003 

        # CAS 1: Both lines visible (Stay in the exact middle)
        if M_red['m00'] > 2000 and M_green['m00'] > 2000:
            cx_red = int(M_red['m10'] / M_red['m00'])
            cx_green = int(M_green['m10'] / M_green['m00'])
            
            # The "ideal" center is halfway between the two lines
            lane_center = (cx_red + cx_green) / 2
            error = center_view - lane_center
            self.line_angular = error * kp_cam
            self.last_state = "RV"

        # CAS 2: Only Red visible (Robot needs to stay a fixed distance from Red)
        elif M_red['m00'] > 2000:
            cx_red = int(M_red['m10'] / M_red['m00'])
            if not self.between_obstacle:
                target_red = w * 0.8 # Goal: Keep red line on the right side (e.g., 80% of width)
            else :
                target_red = w * 0.99
            error = target_red - cx_red
            self.line_angular = error * kp_cam
            self.last_state = "R"

        # CAS 3: Only Green visible (Robot needs to stay a fixed distance from Green)
        elif M_green['m00'] > 2000:
            cx_green = int(M_green['m10'] / M_green['m00'])
            target_green = w * 0.05 # Goal: Keep green line on the left side (e.g., 20% of width)
            #if self.between_obstacle:
            #    target_green = w * 0.1 # 0.3 # Goal: Keep green line on the left side (e.g., 20% of width)
            #else :
            #    target_green = w * 0.2 # 0.05
            error = target_green - cx_green
            self.line_angular = error * kp_cam * 2.5
            self.last_state = "V"

        # Affichage Debug
        cv2.imshow("Camera oa", cv_image)
        cv2.imshow("Masques", cv2.bitwise_or(mask_red, mask_green))
        cv2.waitKey(1)

    def control_loop(self):
        if self.current_state not in [1]:
            return
        
        twist = Twist()
        now = self.get_clock().now().nanoseconds / 1e9
        max_turn_speed = 0.8 

        is_avoiding = self.obstacle_active or (now < self.avoidance_timeout)

        if is_avoiding: # PRIORITY: Obstacle avoidance | self.obstacle_active
            #self.get_logger().info("is avoiding obstacle")
            # We combine the line following and a correction from Lidar

            #weight_cam = 0.4 if self.obstacle_active else 1.0
            #weight_lidar = 1.2

            #weight_cam = 1.6 if self.between_obstacle else 0.4

            #weight_cam = 1.2 if self.between_obstacle else 0.4
            #weight_lidar = 0.0 if self.between_obstacle else 1.2

            if not self.end_obstacle:
                weight_cam = 1.8 if self.between_obstacle else 0.5
                weight_lidar = 0.3 if self.between_obstacle else 1.2
            else:
                weight_cam = 1.0
                weight_lidar = 0.0

            combined_steering = (self.line_angular * weight_cam) + (self.lidar_angular * weight_lidar)
            twist.angular.z = max(min(combined_steering, max_turn_speed), -max_turn_speed)
            
            #self.get_logger().info(f"lidar : {self.lidar_angular:.2f}")
            #self.get_logger().info(f"line : {self.line_angular:.2f}")
            #self.get_logger().info(f"vitesse angulaire : {twist.angular.z:.2f}")
            if self.lidar_linear:
                twist.linear.x = self.lidar_linear
                #self.get_logger().info(f"vitesse linear : {self.lidar_linear:.2f}")
            else:
                twist.linear.x = 0.05 
                #self.get_logger().info("vitesse linear : 0.05")
            
        else:
            #self.get_logger().info("No obstacle in view")
            if self.last_state == "R":
                twist.linear.x = 0.05  # 0.15 
                twist.angular.z = self.line_angular # - 0.1
                #self.get_logger().info(f"Red -vitesse angulaire : {twist.angular.z:.2f}")
            elif self.last_state == "V":
                twist.linear.x = 0.05  # 0.15
                twist.angular.z = self.line_angular # + 0.1
                #self.get_logger().info(f"Green - vitesse angulaire : {twist.angular.z:.2f}")
            else:  # self.last_state == "RV"
                twist.linear.x = 0.08
                twist.angular.z = self.line_angular

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
