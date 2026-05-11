import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, LaserScan
from geometry_msgs.msg import Twist
import cv2
import numpy as np
import time

class BallChallengeNode(Node):
    def __init__(self):
        super().__init__('ball_challenge_node')
        
        # Publishers & Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_cam = self.create_subscription(CompressedImage, '/camera/image_raw/compressed', self.image_callback, 10)

        # State Machine: 1 to 5
        self.state = 1
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # Movement logic variables
        self.twist = Twist()
        self.ball_detected = False
        self.ball_x = 0
        self.red_detected = False
        self.red_x = 0 # Center of red objects
        self.scan_direction = 1 # 1 for Left, -1 for Right
        self.start_time = self.get_clock().now()

        # HSV Thresholds (Tune these for your environment)
        self.yellow_lower = np.array([20, 100, 100])
        self.yellow_upper = np.array([30, 255, 255])
        self.red_lower1 = np.array([0, 100, 100])
        self.red_upper1 = np.array([10, 255, 255])
        self.red_lower2 = np.array([160, 100, 100])
        self.red_upper2 = np.array([179, 255, 255])

    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        h, w, _ = frame.shape
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Yellow Mask (Tennis Ball)
        mask_y = cv2.inRange(hsv, self.yellow_lower, self.yellow_upper)
        M_y = cv2.moments(mask_y)
        if M_y['m00'] > 500:
            self.ball_detected = True
            self.ball_x = int(M_y['m10'] / M_y['m00'])
            self.ball_center_error = (w / 2) - self.ball_x
        else:
            self.ball_detected = False

        # Red Mask (Cylinders)
        m1 = cv2.inRange(hsv, self.red_lower1, self.red_upper1)
        m2 = cv2.inRange(hsv, self.red_lower2, self.red_upper2)
        mask_r = cv2.bitwise_or(m1, m2)
        M_r = cv2.moments(mask_r)
        if M_r['m00'] > 1000:
            self.red_detected = True
            self.red_x = int(M_r['m10'] / M_r['m00'])
            self.red_center_error = (w / 2) - self.red_x
        else:
            self.red_detected = False

    def control_loop(self):
        self.twist = Twist()
        
        # State 1: Scan -60 to 60 for Ball
        if self.state == 1:
            if self.ball_detected:
                self.state = 2
            else:
                # Basic oscillating search logic
                elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
                self.twist.angular.z = 0.4 if np.sin(elapsed) > 0 else -0.4

        # State 2: Offset ball 10 degrees to the right
        elif self.state == 2:
            # Assuming ~60deg FOV, 10deg right is roughly at 66% of image width
            target_x_offset = -0.3 # Normalized offset
            error = (self.ball_center_error / 320.0) - target_x_offset
            if abs(error) < 0.05:
                self.state = 3
            else:
                self.twist.angular.z = error * 0.8

        # State 3: Drive forward until ball is gone
        elif self.state == 3:
            if not self.ball_detected:
                self.state = 4
            else:
                self.twist.linear.x = 0.1
                # Slight correction to keep it in sight while moving
                self.twist.angular.z = self.ball_center_error * 0.002

        # State 4: Turn right until red cylinders visible
        elif self.state == 4:
            if self.red_detected:
                self.state = 5
            else:
                self.twist.angular.z = -0.3 # Turn Right

        # State 5: Center between red cylinders
        elif self.state == 5:
            if self.red_detected:
                # Aim for the center of the red mass
                self.twist.angular.z = self.red_center_error * 0.005
                self.twist.linear.x = 0.05
                if self.red_center_error < 10: # Close enough
                    self.get_logger().info("Target Reached")
            else:
                self.twist.linear.x = 0.0

        self.cmd_vel_pub.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    node = BallChallengeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
