import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from sensor_msgs.msg import CompressedImage
from rclpy.executors import MultiThreadedExecutor
import numpy as np
import cv2

class NodeMonitor(Node):
    def __init__(self):
        super().__init__('node_monitor')
        
        # Subscriptions
        self.line_sub = self.create_subscription(Twist, '/cmd_vel_line', self.line_callback, 10)
        self.obs_sub = self.create_subscription(Twist, '/cmd_vel_obstacle', self.obs_callback, 10)
        self.corr_sub = self.create_subscription(Twist, '/cmd_vel_corridor', self.corr_callback, 10)
        self.scoring_sub = self.create_subscription(Twist, '/cmd_vel_scoring', self.scoring_callback, 10)
        
        self.cam_sub = self.create_subscription(CompressedImage, 'camera/image_raw/compressed', self.cam_callback, 10) # Only for real robot
        #self.cam_sub = self.create_subscription(CompressedImage, '/image_raw/compressed', self.cam_callback, 10)

        self.latest_raw_image = None

        # Publisher to the actual robot hardware/simulator
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # State variables
        self.latest_line_twist = Twist()
        self.latest_obs_twist = Twist()
        self.latest_corr_twist = Twist()
        #self.latest_hmc_twist = Twist()

        
        # Timer to publish at a fixed rate (e.g., 10Hz)
        self.timer = self.create_timer(0.1, self.publish_decision) # 0.1 for 10Hz
        
        # challange state of the robot
        self.state = 0
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

    def scoring_callback(self, msg):
        self.latest_scoring_twist = msg

    def publish_decision(self):
        state_msg = Int32()
        state_msg.data = self.state
        self.state_pub.publish(state_msg)
        # ----- Vision and state logic -----
        if self.latest_raw_image is None :
            self.state_pub.publish(Int32(data=self.state))
            return
        
        try:
            np_arr = np.frombuffer(self.latest_raw_image.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                
            if image is not None : 
                # ---- image analysis
                height, width, _ = image.shape
                # On regarde 60% de l'image pour esquiver les plots
                roi = image[int(height*0.6):height, :] # 0.3
                roi_h, roi_w = roi.shape[:2]
                #roi_height = roi.shape[0] # Get the height of the cropped image
                #roi = image[int(height*0.4):int(height*0.8), :] # to get a better view
                hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

                # 2. Masques de couleur (Cylindre) 
                #lower_blue = np.array([97, 107, 0]) 
                #upper_blue = np.array([130, 255, 255])
                #mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

                # For real turtlebot : 
                mask_blue = cv2.inRange(hsv, (96, 137, 80), (179, 255, 255))

                # filtrage de la couleur bleu (on simulation, only ?)
                kernel = np.ones((5, 5), np.uint8)
                mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_CLOSE, kernel) # This fills small holes inside the object
                mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel) # Optional: This removes tiny speckles of noise
                mask_blue = cv2.dilate(mask_blue, kernel, iterations=1) # This makes the blue object appear thicker to the algorithm
                
                # 1. Find all blue shapes
                contours, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                # 1. Find the biggest blue contour
                current_frame_tape = False

                # For real turtlebot :
                if contours:
                    # Get total number of pixels in the image
                    total_pixels = frame.shape[0] * frame.shape[1]

                    # Calculate what percentage the detected object takes up
                    area_percentage = (area / total_pixels) * 100

                    if area_percentage > 5.0: # If the blue object takes up more than 5% of the screen
                        current_frame_tape = True

                #if contours:
                #    largest_contour = max(contours, key=cv2.contourArea)
                #    area = cv2.contourArea(largest_contour)

                #    # 2. Check if this is the "Floor" trigger 
                #    if 25000 < area < 30 000:
                #        current_frame_tape = True

                # For simulation (cylinder in blue !!!)
                '''
                if contours:
                    # Get the largest blue object
                    largest_contour = max(contours, key=cv2.contourArea)
                    area = cv2.contourArea(largest_contour)

                    # Calculate the centroid (center of the shape)
                    M = cv2.moments(largest_contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        
                        # Get bounding box dimensions
                        x, y, w, h = cv2.boundingRect(largest_contour)
                        aspect_ratio = float(w) / h

                        # 2. THE LOGIC CHECK
                        # We want something that is:
                        # - Large enough (Area > 15000 based on your 26k measurement)
                        # - In the bottom half of the screen (cy > frame_height / 2)
                        # - Wider than it is tall (aspect_ratio > 1.2)
                        
                        height, width = mask_blue.shape
                        
                        if area > 15000 and cy > (height * 0.6):
                            # This filters out far-away cylinders (small area) 
                            # and close-up cylinders (cy would be higher up, and aspect_ratio < 1.0)
                            current_frame_tape = True
                
                if current_frame_tape and not self.tape_detected:
                    self.state+=1
                    self.get_logger().info(f"!!! STATE CHANGED TO: {self.state} !!!")
                    self.tape_detected = True
                
                elif not current_frame_tape:
                    self.tape_detected = False
                '''

        except Exception as e:
            self.get_logger().error(f"Vision error: {e}")
        
        # ----- Movement Logic -----
        out_msg = Twist()

        if self.state == 0:
            out_msg = self.latest_line_twist

        elif self.state == 1:
            out_msg = self.latest_obs_twist

        elif self.state == 2:
            out_msg = self.latest_corr_twist

        elif self.state == 3:
            out_msg = self.latest_line_twist
        
        else:
            #out_msg = self.latest_line_twist
            out_msg = self.latest_scoring_twist 

        self.cmd_pub.publish(out_msg)

        # PRIORITY LOGIC: 
        # If obstacle twist has linear velocity or angular velocity (meaning it wants to move/steer)
        # we prioritize it. Otherwise, follow the line.   
        #elif abs(self.latest_corr_twist.linear.x) > 0.01 or abs(self.latest_corr_twist.angular.z) > 0.01 : 
        #    out_msg = self.latest_corr_twist

def main(args=None):
    rclpy.init(args=args)
    node = NodeMonitor()
    
    # Use the MultiThreadedExecutor to prevent one callback from blocking others
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
