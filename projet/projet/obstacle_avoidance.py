import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
import numpy as np
import cv2

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        
        # Subscriber pour l'image du projet 2025
        self.subscription = self.create_subscription(
            CompressedImage, 
            '/image_raw/compressed', # Make sure the topic name is correct!
            self.camera_callback, 
            10)

        self.line_cmd_sub = self.create_subscription(Twist, '/cmd_vel_line', self.line_callback, 10)

        # 3. Talk to the actual robot
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Publisher pour le mouvement
        self.latest_line_twist = Twist()

    def line_callback(self, msg):
        # Just store the line following command for later
        self.latest_line_twist = msg

    def camera_callback(self, msg):
        # 1. Décodage de l'image
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if image is None: return

        height, width, _ = image.shape
        # On regarde 60% de l'image pour esquiver les plots
        roi = image[int(height*0.4):height, :]
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

        is_obstacle_detected = False

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 400:  # Only look at significant objects
                x, y, w, h = cv2.boundingRect(cnt)
                
                # 2. Shape check: Cylinders are usually taller than they are wide 
                # or at least not "flat" like tape.
                aspect_ratio = float(h) / w
                
                # Tape on floor usually has h < w (aspect ratio < 1)
                # Cylinder usually has h > w (aspect ratio > 1)
                if aspect_ratio > 0.8: 
                    is_obstacle_detected = True
                    
                    # Use moments for THIS specific contour
                    m = cv2.moments(cnt)
                    cx = int(m['m10'] / m['m00'])
                    
                    twist = Twist()
                    if cx < 320:
                        twist.angular.z = -0.5 # Turn Right
                    else:
                        twist.angular.z = 0.5  # Turn Left
                    
                    self.cmd_vel_pub.publish(twist)
                    break # Exit the loop, we found our obstacle!

        # 3. If no standing cylinders were found, follow the line
        if not is_obstacle_detected:
            self.cmd_vel_pub.publish(self.latest_line_twist)

        cv2.imshow("Masque Bleu", mask_blue)
        cv2.waitKey(1)

        """
        bc = cv2.moments(mask_blue)
        twist = Twist()

        # 4. Logique de suivi (Rester entre les deux lignes) 
        if bc['m00'] > 500:  # (500 = nuber of pixel to make the robot turn) -> Threshold: only react if the blue object is big enough
            # Calculate the center of the blue object
            cx = int(bc['m10'] / bc['m00'])
            
            # Get the center of the camera frame (assuming 640px wide)
            screen_center = 320 
            
            if cx < screen_center:
                # Obstacle is on the LEFT -> Turn RIGHT
                #print("Blue on Left! Steering Right.")
                twist.angular.z = -0.5
            
            else:
                # Obstacle is on the RIGHT -> Turn LEFT
                #print("Blue on Right! Steering Left.")
                twist.angular.z = 0.5

            # Publish when we are actually dodging
            self.cmd_vel_pub.publish(twist)
            
        else:
            # NO OBSTACLE: Just repeat what the line follower said
            self.cmd_vel_pub.publish(self.latest_line_twist)
        """
        cv2.imshow("Masque Bleu", mask_blue)
        cv2.waitKey(1)
        

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