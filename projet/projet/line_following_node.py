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
        
        # Subscriber pour l'image du projet 2025
        self.subscription = self.create_subscription(
            Image, 
            '/image_raw', 
            self.listener_callback, 
            10
        )

        # PARAMÈTRE : Pour choisir la direction au rond-point
        # Valeurs possibles : 'both', 'left', 'right'
        self.declare_parameter('direction', 'both')
        
        # Publisher pour le mouvement
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info("Nœud de suivi de ligne opérationnel !")

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h, w, _ = cv_image.shape
            
            # ROI fine (moitié basse de l'image)
            roi = cv_image[int(h/3):h, :]
            
            # Passage en HSV (obligatoire pour la robustesse)
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
            # --- LES MASQUES HSV ---
            # Rouge (Double plage car le rouge est aux deux extrémités du spectre)
            mask_red1 = cv2.inRange(hsv, (0, 100, 50), (10, 255, 255))
            mask_red2 = cv2.inRange(hsv, (160, 100, 50), (180, 255, 255))
            mask_red = cv2.bitwise_or(mask_red1, mask_red2)
            
            # Vert (Tolérant sur l'ombre)
            mask_green = cv2.inRange(hsv, (35, 50, 50), (90, 255, 255))

            # --- NETTOYAGE MORPHOLOGIQUE ---
            kernel = np.ones((5, 5), np.uint8)
            mask_green = cv2.dilate(mask_green, kernel, iterations=1) 
            mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
            mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)
            mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)
            mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, kernel)

            # Fusion pour l'affichage uniquement
            mask_combined = cv2.bitwise_or(mask_red, mask_green)

            # 1. Calcul des moments séparés
            M_red = cv2.moments(mask_red)
            M_green = cv2.moments(mask_green)
            
            twist = Twist()

            # CAS 1 : On voit les deux lignes (Le couloir parfait)
            if M_red['m00'] > 0 and M_green['m00'] > 0:
                cx_red = int(M_red['m10'] / M_red['m00'])
                cx_green = int(M_green['m10'] / M_green['m00'])
                
                # La cible est la moyenne des deux centres
                cible = (cx_red + cx_green) / 2
                error = cible - (w / 2)
                
                twist.linear.x = 0.1
                twist.angular.z = -float(error) / 100.0
                # self.get_logger().info("Mode : Entre les deux lignes")
            
            # CAS 2 : On voit seulement la ligne rouge (On est à gauche, il faut aller à droite)
            elif M_red['m00'] > 0:
                cx_red = int(M_red['m10'] / M_red['m00'])
                error = cx_red - (w / 2)
                
                twist.linear.x = 0.1
                twist.angular.z = -float(error) / 100.0
                # self.get_logger().info("Mode : Seule la ligne rouge détectée")

            # CAS 3 : On voit seulement la ligne verte (On est à droite, il faut aller à gauche)
            elif M_green['m00'] > 0:
                cx_green = int(M_green['m10'] / M_green['m00'])
                error = cx_green - (w / 2)
                
                twist.linear.x = 0.1
                twist.angular.z = -float(error) / 100.0
                # self.get_logger().info("Mode : Seule la ligne verte détectée")

            # CAS 4 : On ne voit rien
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.2 # On tourne doucement pour chercher la piste
                # self.get_logger().warn("Piste perdue !")

            self.cmd_vel_pub.publish(twist)

            self.cmd_vel_pub.publish(twist)

            # --- AFFICHAGE ---
            cv2.imshow("Vision Originale", cv_image)
            cv2.imshow("Masques Fusionnes", mask_combined)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Erreur dans le callback : {e}")

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