import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2

class NodeMonitor(Node):
    def __init__(self):
        super().__init__('node_monitor')
        
        # Subscriptions
        self.line_sub = self.create_subscription(Twist, '/cmd_vel_line', self.line_callback, 10)
        self.obs_sub = self.create_subscription(Twist, '/cmd_vel_obstacle', self.obs_callback, 10)
        self.corr_sub = self.create_subscription(Twist, '/cmd_vel_corridor', self.corr_callback, 10)
        self.hmc_sub = self.create_subscription(Twist, '/cmd_vel_hmc', self.hmc_callback, 10)
        
        self.cam_sub = self.create_subscription(
            CompressedImage, 
            '/camera/image/compressed', # Double check this topic name with 'ros2 topic list'
            self.cam_callback, 
            10)

        self.latest_raw_image = None

        # Publisher to the actual robot hardware/simulator
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # State variables
        self.latest_line_twist = Twist()
        self.latest_obs_twist = Twist()
        self.latest_corr_twist = Twist()
        self.latest_hmc_twist = Twist()

        
        # Timer to publish at a fixed rate (e.g., 10Hz)
        self.timer = self.create_timer(0.1, self.publish_decision) # 0.1 for 10Hz
        
        # challange state of the robot
        self.state = 4
        self.state_pub = self.create_publisher(Int32, '/robot_state', 10)
        self.tape_detected = False # Gate (to increase state only one time by tape)

    def line_callback(self, msg):
        self.latest_line_twist = msg

    def obs_callback(self, msg):
        self.latest_obs_twist = msg

    def corr_callback(self, msg):
        self.latest_corr_twist = msg

    def cam_callback(self, msg): # camera callback to save the image
        self.latest_raw_image = msg

    def hmc_callback(self, msg):
        self.latest_hmc_twist = msg

    def publish_decision(self):
        # ----- Vision and state logic -----
        if self.latest_raw_image is not None :
            try:
                np_arr = np.frombuffer(self.latest_raw_image.data, np.uint8)
                image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                
                if image is not None : 
                    # ---- to see the state on the cv2 window ----
                    display_img = image.copy()
                    text = f"state : {self.state} | tape detected : {self.tape_detected}"
                    font = cv2.FONT_HERSHEV_SIMPLEX
                    color = (0, 255, 0) if self.tape_detected else (255, 255, 255) # Green when tape detected, white if not
                    cv2.putText(display_img, text, (20, 50), font, 0.8, color, 2, cv2.LINE_AA) # Draw the text on the image
                    cv2.imshow("Robot Dashboard", display_img)
                    cv2.waitKey(1)
                    
                    # ---- image analysis
                    height, width, _ = image.shape
                    # On regarde 60% de l'image pour esquiver les plots
                    roi = image[int(height*0.3):height, :]
                    roi_height = roi.shape[0] # Get the height of the cropped image
                    #roi = image[int(height*0.4):int(height*0.8), :] # to get a better view
                    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

                    # 2. Masques de couleur (Cylindre) 
                    # Ajustez ces seuils si les couleurs sur le terrain réel diffèrent
                    lower_blue = np.array([100, 150, 50]) 
                    upper_blue = np.array([130, 255, 255])
                    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
                    # filtrage de la couleur bleu
                    kernel = np.ones((5, 5), np.uint8)
                    mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_CLOSE, kernel) # This fills small holes inside the object
                    mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel) # Optional: This removes tiny speckles of noise
                    mask_blue = cv2.dilate(mask_blue, kernel, iterations=1) # This makes the blue object appear thicker to the algorithm

                    # 1. Find all blue shapes
                    contours, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                    # 1. Find the biggest blue contour
                    if contours:
                        largest_contour = max(contours, key=cv2.contourArea)
                        largest_area = cv2.contourArea(largest_contour)

                        # Get the center of the contour
                        M = cv2.moments(largest_contour)
                        if M["m00"] != 0:
                            cx = int(M["m10"] / M["m00"])
                            cy = int(M["m01"] / M["m00"])

                        # 2. Check if this is the "Floor" trigger (Example: 50 < area < 250)
                        if 50 < largest_area < 250 and cy > (roi_height*0.8):
                            if not self.tape_detected:
                                self.state += 1
                                self.get_logger().info(f"New state : {self.state}")
                                self.tape_detected = True
                        else:
                            self.tape_detected = False
            except Exception as e:
                self.get_logger().error(f"Vision error: {e}")
        
        # ----- Movement Logic -----
        out_msg = Twist()
        state_msg = Int32()
        state_msg.data = self.state

        if self.state == 0:
            out_msg = self.latest_line_twist

        elif self.state == 1:
            out_msg = self.latest_obs_twist

        elif self.state == 2:
            out_msg = self.latest_corr_twist

        elif self.state == 3:
            out_msg = self.latest_line_twist
        
        else:
            out_msg = self.latest_line_twist # to be change at the end, should be the ball mode

        # PRIORITY LOGIC: 
        # If obstacle twist has linear velocity or angular velocity (meaning it wants to move/steer)
        # we prioritize it. Otherwise, follow the line.   
        #elif abs(self.latest_corr_twist.linear.x) > 0.01 or abs(self.latest_corr_twist.angular.z) > 0.01 : 
        #    out_msg = self.latest_corr_twist
        self.state_pub.publish(state_msg)
        self.cmd_pub.publish(out_msg)

        cv2.imshow("masque scotch", image)
        cv2.waitKey(1) 

def main(args=None):
    rclpy.init(args=args)
    node = NodeMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
