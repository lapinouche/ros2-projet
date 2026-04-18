import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist

from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2

from std_msgs.msg import Int32

class NodeMonitor(Node):
    def __init__(self):
        super().__init__('node_monitor')

        #self.group = ReentrantCallbackGroup()

        # Subscriptions
        self.line_sub = self.create_subscription(Twist, '/cmd_vel_line', self.line_callback, 10)
        self.obs_sub = self.create_subscription(Twist, '/cmd_vel_obstacle', self.obs_callback, 10)
        self.corr_sub = self.create_subscription(Twist, '/cmd_vel_corridor', self.corr_callback, 10)
        self.cam_sub = self.create_subscription(CompressedImage, '/image_raw/compressed', self.cam_callback, 10)

        self.latest_raw_image = None

        # Publisher to the actual robot hardware/simulator
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # State variables
        self.latest_line_twist = Twist()
        self.latest_obs_twist = Twist()
        self.latest_corr_twist = Twist()
        
        # Timer to publish at a fixed rate (e.g., 10Hz)
        self.timer = self.create_timer(0.1, self.publish_decision) # 0.1 for 10Hz

        # challange state of the robot
        self.state = 0
        self.tape_detected = False # Gate (to increase state only one time by tape)

        self.state_pub = self.create_publisher(Int32, '/robot_state', 10)

    def line_callback(self, msg):
        self.latest_line_twist = msg

    def obs_callback(self, msg):
        self.latest_obs_twist = msg

    def corr_callback(self, msg):
        self.latest_corr_twist = msg

    def cam_callback(self, msg): # camera callback to save the image
        self.latest_raw_image = msg

    def publish_decision(self):
        #print("DEBUG: Logic is running!") # Plain python print (hard to miss)
        # ----- Vision and state logic -----
        if self.latest_raw_image is not None :
            try:
                np_arr = np.frombuffer(self.latest_raw_image.data, np.uint8)
                image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                
                if image is not None :  
                    # ---- image analysis
                    height, width, _ = image.shape
                    # On regarde 60% de l'image
                    if self.state == 0:
                        roi = image[int(height*0.4):height, :]
                    else : 
                        roi = image[int(height*0.6):height, :]
                    
                    roi_height = roi.shape[0] # Get the height of the cropped image
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
                        area = cv2.contourArea(largest_contour)

                        # Get bounding box dimensions
                        x, y, w, h = cv2.boundingRect(largest_contour)
                        
                        # Tape logic : should be wide and relatively small
                        #is_tape = (w > 1.5*h) and (area < 350)
                        
                        aspect_ratio = float(w)/h

                        """
                        # Get the center of the contour
                        M = cv2.moments(largest_contour)
                        if M["m00"] != 0:
                            cx = int(M["m10"] / M["m00"])
                            cy = int(M["m01"] / M["m00"])
                        
                        #print(f"DEBUG: Area={largest_area}, Y={cy}, Threshold={roi_height*0.8}")
                        really_tape = 50 < largest_area < 350 and cy > (roi_height*0.8):
                        """

                        # 2. Check if this is the "Floor" trigger (Example: 50 < area < 250)
                        #if is_tape:
                        #self.get_logger().info(f"aspect_ratio: {aspect_ratio}")
                        #self.get_logger().info(f"area: {aspect_ratio}")
                        
                        if (0.8 < aspect_ratio < 1.5) and (20 < area < 400): # 1.2
                            if not self.tape_detected:
                                if self.state < 1:
                                    self.state += 1
                                self.get_logger().info(f"New state : {self.state}")
                                self.tape_detected = True

                        else:
                            self.tape_detected = False
            
            except Exception as e:
                self.get_logger().error(f"Vision error: {e}")
        
        # ----- Movement Logic -----
        out_msg = Twist()

        # ----- State ----- (go on ros2 topic echo /robot_state to see it)
        state_msg = Int32()
        state_msg.data = self.state # accept only integer !

        if self.state == 0:
            out_msg = self.latest_line_twist

        elif self.state == 1:
            if abs(self.latest_obs_twist.linear.x) > 0.01 or abs(self.latest_obs_twist.angular.z) > 0.01:
                out_msg = self.latest_obs_twist
                self.get_logger().info("Priority: Obstacle Avoidance")
            else:
                out_msg = self.latest_line_twist

        elif self.state == 2:
            self.get_logger().info("Priority: Corridor")
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
        #executor.spin()
        rclpy.spin(node)
    
    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
