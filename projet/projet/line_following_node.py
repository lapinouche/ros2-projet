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
        self.chosen_side = "right"
        
        self.get_logger().info("Nœud de suivi de ligne opérationnel avec contrôle CY !")

        # dimensionnement de la fenetre de la camera
        cv2.namedWindow("Vision", cv2.WINDOW_NORMAL) # nomee la fenetre
        cv2.resizeWindow("Vision", 300, 300) # la redimensionner

    def check_roundabout_entry(self, mask_combined, mask_red_full, mask_green_full, h, w):
        # Slice the combined mask to look at the "horizon"
        top_zone = mask_combined[int(h*0.3):int(h*0.6), :]
        
        contours, _ = cv2.findContours(top_zone, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for cnt in contours:
            if cv2.contourArea(cnt) > 400:
                if len(cnt) >= 5:
                    # 1. Shape Detection (Ellipse)
                    ellipse = cv2.fitEllipse(cnt)
                    (cx, cy), (w_e, h_e), angle = ellipse
                    
                    # 2. Centering
                    is_centered = abs(cx - (w / 2)) < (w * 0.2)
                    
                    # 3. Color Verification
                    # We check the original red/green masks in the same area
                    # to ensure BOTH colors are present in this blob
                    roi_red = mask_red_full[int(h*0.3):int(h*0.6), :]
                    roi_green = mask_green_full[int(h*0.3):h, :] # Using h for green just in case
                    
                    has_red = cv2.countNonZero(roi_red) > 200
                    has_green = cv2.countNonZero(roi_green) > 200

                    if is_centered and has_red and has_green:
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
            
            # CAS 1 : On voit les deux lignes
            if M_red['m00'] > 2000 and M_green['m00'] > 2000:
                cx_red = int(M_red['m10'] / M_red['m00'])
                cx_green = int(M_green['m10'] / M_green['m00'])
                cible = (cx_red + cx_green) / 2
                error = cible - (w / 2)
                self.last_state = "RV"
                
                twist.linear.x = 0.1
                twist.angular.z = -float(error) / 100.0

            # CAS 2 : On voit seulement la ligne rouge (Le vert a disparu)
            elif M_red['m00'] > 2000:
                cy_red = int(M_red['m01'] / M_red['m00']) # Hauteur de la ligne dans le ROI
                self.last_state = "R"
                
                if direction_choisie == 'left':
                    # On ne tourne que si la ligne rouge est assez BASSE (proche du robot)
                    if cy_red > 45: 
                        twist.linear.x = 0.08
                        twist.angular.z = 0.8  # Virage gauche
                    else:
                        twist.linear.x = 0.1
                        twist.angular.z = 0.0  # On attend d'être plus près
                else:
                    # On longe la ligne rouge pour rester sur la piste
                    cx_red = int(M_red['m10'] / M_red['m00'])
                    error = cx_red - 80
                    twist.linear.x = 0.08
                    twist.angular.z = -float(error) / 50.0 

            # CAS 3 : On voit seulement la ligne verte (Le rouge a disparu)
            elif M_green['m00'] > 2000:
                cy_green = int(M_green['m01'] / M_green['m00'])
                self.last_state = "V"
                
                if direction_choisie == 'right':
                    if cy_green > 45:
                        twist.linear.x = 0.08
                        twist.angular.z = -0.8 # Virage droite
                    else:
                        twist.linear.x = 0.1
                        twist.angular.z = 0.0
                else:
                    # On longe la ligne verte
                    cx_green = int(M_green['m10'] / M_green['m00'])
                    error = cx_green - (w - 80)
                    twist.linear.x = 0.08
                    twist.angular.z = float(error) / 50.0 

            # CAS 4 : Rien du tout
            else:
                if self.in_roundabout:
                    self.get_logger().info("rond point detecter !!!")
                    twist.linear.x = 0.05
                    twist.angular.z = -1.2 if self.chosen_side == "right" else 1.2

                    if M_red['m00'] > 5000 or M_green['m00'] > 5000:
                        self.in_roundabout = False

                elif self.last_state == "R":
                    twist.linear.x = 0.05 # 0.15
                    twist.angular.z = -0.3
                elif self.last_state == "V":
                    twist.linear.x = 0.05 # 0.15
                    twist.angular.z = 0.3
                elif self.last_state == "RV":
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
