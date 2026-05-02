import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import numpy as np
from cv_bridge import CvBridge
import cv2

class LineFollowerNode(Node):
    def __init__(self):
        super().__init__('line_follower_node')

        self.bridge = CvBridge()
        
        # Subscriber pour l'image
        self.subscription = self.create_subscription(Image, '/image_raw', self.listener_callback, 10)

        # PARAMÈTRE : Pour choisir la direction au rond-point
        self.declare_parameter('direction', 'left')
        
        # Publisher pour le mouvement
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # chequer la derniere ligne visible
        self.last_state = "RV" # ligne rouge et vert visible
        self.in_roundabout = False # passe au mode rond point une fois detecter
        self.timer = 0 # stopper la rotation après un laps de "temps" 
        
        self.get_logger().info("Nœud de suivi de ligne opérationnel avec contrôle CY !")

        # dimensionnement de la fenetre de la camera
        cv2.namedWindow("Vision", cv2.WINDOW_NORMAL) # nomee la fenetre
        cv2.resizeWindow("Vision", 300, 300) # la redimensionner

    def check_roundabout_entry(self, mask_combined, mask_red_full, mask_green_full, h, w):
        # Slice the combined mask to look at the "horizon"
        top_zone = mask_combined[int(h*0.3):int(h*0.6), :]
        
        contours, _ = cv2.findContours(top_zone, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 400:
                # Get the bounding box for the specific blod found
                x, y, w_box, h_box = cv2.boundingRect(cnt)
                if len(cnt) >= 5:
                    # 1. Shape Detection (Ellipse)
                    ellipse = cv2.fitEllipse(cnt)
                    (cx, cy), (w_e, h_e), angle = ellipse
                    
                    # 2. Centering
                    is_centered = abs(cx - (w / 2)) < (w * 0.2)
                    
                    if is_centered:
                        # Targeted color verification (crop the red and green masks to ONLY the area where the blod is)
                        y_offset = int(h*0.3)
                    
                        # Create a small ROI exactly around the detected contour
                        roi_red_local = mask_red_full[y + y_offset : y + y_offset + h_box, x : x + w_box]
                        roi_green_local = mask_green_full[y + y_offset : y + y_offset + h_box, x : x + w_box] # Using h for green just in case

                        #Check for color density inside this specific object
                        red_pixels = cv2.countNonZero(roi_red_local)
                        green_pixels = cv2.countNonZero(roi_green_local)

                        # Check if both occupy a significant portion of this blod
                        has_red = red_pixels > (area * 0.2)
                        has_green = green_pixels > (area * 0.2)

                        if has_red and has_green:
                            self.get_logger().info("Roundabout confirmed: Red and Green halves detected!")
                            return True
        return False

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h, w, _ = cv_image.shape
            #hsv_full = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            # ROI : On prend une zone un peu plus haute pour voir venir la ligne
            roi = cv_image[int(h * 0.4):h, :]
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

            # --- MASQUES HSV ---
            mask_red1 = cv2.inRange(hsv, (0, 100, 50), (10, 255, 255))
            mask_red2 = cv2.inRange(hsv, (160, 100, 50), (180, 255, 255))

            mask_red_full = cv2.bitwise_or(mask_red1, mask_red2)
            mask_green_full = cv2.inRange(hsv, (35, 50, 50), (90, 255, 255))
            
            # part added
            
            if self.in_roundabout:
                # Create a blank mask the same size as your HSV image
                h, w = hsv.shape[:2]
                mask_focus = np.zeros((h, w), dtype=np.uint8)
                
                # If turning left, only look at the RIGHT 50% of the screen
                direction_choisie = self.get_parameter('direction').value
                if direction_choisie == "left":
                    mask_focus[:, int(2*w/3):w] = 255
                else:
                    mask_focus[:, 0:int(2*w/3)] = 255
                    
                # Apply this focus to your color masks
                mask_red_full = cv2.bitwise_and(mask_red_full, mask_red_full, mask=mask_focus)
                mask_green_full = cv2.bitwise_and(mask_green_full, mask_green_full, mask=mask_focus)
            

            # --- NETTOYAGE ---
            kernel = np.ones((5, 5), np.uint8)
            mask_red_full = cv2.morphologyEx(mask_red_full, cv2.MORPH_OPEN, kernel)
            mask_green_full = cv2.morphologyEx(mask_green_full, cv2.MORPH_OPEN, kernel)
            
            # --- ROND POINT ---
            mask_combined = cv2.bitwise_or(mask_red_full, mask_green_full)
            mask_combined = cv2.dilate(mask_combined, kernel, iterations=2)
            detected = self.check_roundabout_entry(mask_combined, mask_red_full, mask_green_full, h, w)
            if detected:
                self.in_roundabout = True

            # --- REDIMENSIONNEMENT ---
            mask_red = mask_red_full[int(h * 0.2):h, :]
            mask_green = mask_green_full[int(h * 0.2):h, :]

            # Moments
            M_red = cv2.moments(mask_red)
            M_green = cv2.moments(mask_green)

            twist = Twist()
            direction_choisie = self.get_parameter('direction').value

            # --- LOGIQUE DE MOUVEMENT MODIFIÉE ---
            if self.in_roundabout:
                self.get_logger().info("rond point detecter !!!")
                twist.linear.x = 0.00 # 0.05
                twist.angular.z = -1.2 if self.chosen_side == "right" else 1.2
                self.timer += 1
                self.get_logger().info(f"timer : {self.timer}")

                # Only exit if the red/green line is on the right half of the screen
                cx_red = int(M_red['m10'] / M_red['m00']) if M_red['m00'] > 0 else 0
                if cx_red > (w / 2):
                    if self.timer > 15 : # or (M_red['m00'] > 2000 and M_green['m00'] > 2000)
                        self.in_roundabout = False
                        self.timer = 0
                        self.get_logger().info("Exit line correctly positioned. Switching to Line Follower.")

            # CAS 1 : On voit les deux lignes
            elif M_red['m00'] > 2000 and M_green['m00'] > 2000:
                cx_red = int(M_red['m10'] / M_red['m00'])
                cx_green = int(M_green['m10'] / M_green['m00'])
                cible = (cx_red + cx_green) / 2
                error = cible - (w / 2)
                self.last_state = "RV"
                
                twist.linear.x = 0.1
                twist.angular.z = -float(error) / 100.0 # 100.0

            # CAS 2 : On voit seulement la ligne rouge (Le vert a disparu)
            elif M_red['m00'] > 2000:
                cy_red = int(M_red['m01'] / M_red['m00']) # Hauteur de la ligne dans le ROI
                self.last_state = "R"
                
                if direction_choisie == 'left': # left
                    # On ne tourne que si la ligne rouge est assez BASSE (proche du robot)
                    if cy_red > 38: # 45
                        twist.linear.x = 0.08
                        twist.angular.z = 0.7  # (0.8) Virage gauche
                    else:
                        twist.linear.x = 0.1
                        twist.angular.z = 0.0  # On attend d'être plus près
                
                else:
                    # On longe la ligne rouge pour rester sur la piste
                    """
                    cx_red = int(M_red['m10'] / M_red['m00'])
                    error = cx_red - 80
                    wist.linear.x = 0.08
                    twist.angular.z = -float(error) / 50.0  
                    """
                    cx_red = int(M_red['m10'] / M_red['m00'])
                    error = cx_red - 80
                    # P Gain
                    kp = 0.02

                    if abs(error) > 25:
                        kp = 0.05
                        twist.linear.x = 0.04
                    else:
                        twist.linear.x = 0.08

                    twist.angular.z = -float(error)*kp 
                    

            # CAS 3 : On voit seulement la ligne verte (Le rouge a disparu)
            elif M_green['m00'] > 2000:
                cy_green = int(M_green['m01'] / M_green['m00'])
                self.last_state = "V"
                
                if direction_choisie == 'right': # right
                    if cy_green > 40: # 45
                        twist.linear.x = 0.08
                        twist.angular.z = -0.6 # (0.8) Virage droite
                    else:
                        twist.linear.x = 0.1
                        twist.angular.z = 0.0
                else:
                    # On longe la ligne verte
                    """
                    cx_green = int(M_green['m10'] / M_green['m00'])
                    error = cx_green - 80
                    wist.linear.x = 0.08
                    twist.angular.z = -float(error) / 50.0  
                    """
                    cx_green = int(M_green['m10'] / M_green['m00'])
                    error = cx_green - 80
                    
                    # P Gain
                    kp = 0.02

                    if abs(error) > 25:
                        kp = 0.05
                        twist.linear.x = 0.04
                    else:
                        twist.linear.x = 0.08
                        
                    twist.angular.z = -float(error)*kp

            # CAS 4 : Rien du tout
            else:
                if self.last_state == "R":
                    twist.linear.x = 0.05   # 0.15
                    twist.angular.z = -0.3
                elif self.last_state == "V":
                    twist.linear.x = 0.05   # 0.15
                    twist.angular.z = 0.3
                else:                       #self.last_state == "RV"
                    twist.linear.x = 0.08
                    twist.angular.z = 0.0

            self.cmd_vel_pub.publish(twist)

            # Affichage Debug
            cv2.imshow("Masques", cv2.bitwise_or(mask_red, mask_green))
            cv2.imshow("Vision", cv_image) # l'afficher
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Erreur : {e}")

def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
