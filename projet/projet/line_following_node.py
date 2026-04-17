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
        
        self.get_logger().info("Nœud de suivi de ligne opérationnel avec contrôle CY !")

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h, w, _ = cv_image.shape
            
            # ROI : On prend une zone un peu plus haute pour voir venir la ligne
            roi = cv_image[int(h * 0.6):h, :]
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
            # --- MASQUES HSV ---
            mask_red1 = cv2.inRange(hsv, (0, 100, 50), (10, 255, 255))
            mask_red2 = cv2.inRange(hsv, (160, 100, 50), (180, 255, 255))
            mask_red = cv2.bitwise_or(mask_red1, mask_red2)
            mask_green = cv2.inRange(hsv, (35, 50, 50), (90, 255, 255))

            # --- NETTOYAGE ---
            kernel = np.ones((5, 5), np.uint8)
            mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
            mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)

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
                
                twist.linear.x = 0.1
                twist.angular.z = -float(error) / 100.0

            # CAS 2 : On voit seulement la ligne rouge (Le vert a disparu)
            elif M_red['m00'] > 2000:
                cy_red = int(M_red['m01'] / M_red['m00']) # Hauteur de la ligne dans le ROI
                
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
                twist.linear.x = 0.0
                twist.angular.z = -0.2

            self.cmd_vel_pub.publish(twist)

            # Affichage Debug
            cv2.imshow("Masques", cv2.bitwise_or(mask_red, mask_green))
            cv2.imshow("Vision", cv_image)
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