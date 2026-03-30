import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
import numpy as np
import cv2

class LineFollowerNode(Node):
    def __init__(self):
        super().__init__('line_follower_node')
        
        # Subscriber pour l'image du projet 2025
        self.subscription = self.create_subscription(
            CompressedImage, 
            '/image_raw/compressed', 
            self.listener_callback, 
            10)
            
        # Publisher pour le mouvement
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info("Nœud de suivi de ligne opérationnel !")

    def get_centroid(self, mask):
        """Calcule le centre X d'un masque à l'aide des moments d'image """
        M = cv2.moments(mask)
        if M['m00'] > 0:
            return int(M['m10'] / M['m00'])
        return None

    def listener_callback(self, msg):
        # 1. Décodage de l'image
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if image is None: return

        height, width, _ = image.shape
        # On ne regarde que le bas de l'image (la route devant le robot)
        roi = image[int(height*0.6):height, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # 2. Masques de couleur (Ligne Verte et Ligne Rouge) 
        # Ajustez ces seuils si les couleurs sur le terrain réel diffèrent
        mask_green = cv2.inRange(hsv, np.array([40, 100, 50]), np.array([80, 255, 255]))
        
        lower_red = cv2.inRange(hsv, np.array([0, 100, 50]), np.array([10, 255, 255]))
        upper_red = cv2.inRange(hsv, np.array([160, 100, 50]), np.array([180, 255, 255]))
        mask_red = cv2.bitwise_or(lower_red, upper_red)

        # 3. Calcul des positions avec les Moments
        gx = self.get_centroid(mask_green)
        rx = self.get_centroid(mask_red)

        twist = Twist()
        twist.linear.x = 0.08  # Vitesse lente et prudente pour la démo 

        # 4. Logique de suivi (Rester entre les deux lignes) 
        if gx is not None and rx is not None:
            # On veut que le milieu entre les deux lignes soit au centre de l'image
            middle_line = (gx + rx) / 2
            error = middle_line - (width / 2)
            twist.angular.z = -float(error) * 0.005 # Correction proportionnelle
            self.get_logger().info(f"Suivi des deux lignes - Erreur: {error:.2f}")
            
        elif gx is not None: # Seulement ligne verte visible (virage ou bord)
            error = gx - (width * 0.2) # On essaie de garder la verte à gauche
            twist.angular.z = -float(error) * 0.005
            
        elif rx is not None: # Seulement ligne rouge visible
            error = rx - (width * 0.8) # On essaie de garder la rouge à droite
            twist.angular.z = -float(error) * 0.005
            
        else:
            twist.linear.x = 0.0 # On ne voit rien : STOP de sécurité 
            self.get_logger().warn("Lignes perdues !")

        # 5. Publication et Affichage (indispensable pour débugger !)
        self.cmd_vel_pub.publish(twist)
        
        # Visualisation pour votre binôme et l'évaluateur [cite: 76]
        cv2.imshow("Masque Vert", mask_green)
        cv2.imshow("Masque Rouge", mask_red)
        cv2.imshow("Vue Robot", image)
        cv2.waitKey(1)

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

if __name__ == '__main__':
    main()