import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
from cv_bridge import CvBridge
import cv2

class LineFollowerNode(Node):
    def __init__(self):
        super().__init__('line_follower_node')
        self.bridge = CvBridge()
        self.sub_img = self.create_subscription(Image, '/image_raw', self.listener_callback, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.obstacle_detecte = False
        self.declare_parameter('direction', 'left')
        self.get_logger().info("Nœud de suivi de ligne démarré.")
    
    def listener_callback(self, msg):
        try:
            if self.obstacle_detecte:
                self.get_logger().info("Obstacle détecté, arrêt du robot.")
                return # On ne fait rien tant qu'on a pas traité l'obstacle
            
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h, w, _ = cv_image.shape

            self.get_logger().info(f"Dimensions : {h} x {w}")
            
            # Au lieu de h/2 (qui voit trop loin), utilise h*0.7 ou h*0.75
            roi = cv_image[int(h * 0.6):h, 0:w]
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
            # --- MASQUES HSV ---
            mask_red1 = cv2.inRange(hsv, (0, 50, 30), (15, 255, 255))
            mask_red2 = cv2.inRange(hsv, (160, 50, 30), (180, 255, 255))
            mask_red = cv2.bitwise_or(mask_red1, mask_red2)
            mask_green = cv2.inRange(hsv, (35, 50, 50), (90, 255, 255))

            # 1. On crée un masque global (tout ce qui est coloré)
            mask_total = cv2.bitwise_or(mask_red, mask_green)
            M_total = cv2.moments(mask_total)

            # Moments
            M_red = cv2.moments(mask_red)
            M_green = cv2.moments(mask_green)

            twist = Twist()
            direction_choisie = self.get_parameter('direction').value

            # --- LOGIQUE DE MOUVEMENT MODIFIÉE ---
            
            # CAS 1 : On voit les deux lignes
            if M_red['m00'] > 0 and M_green['m00'] > 0:

                cx_red = int(M_red['m10'] / M_red['m00'])
                cx_green = int(M_green['m10'] / M_green['m00'])
                cible = (cx_red + cx_green) / 2
                error = cible - (w / 2)
                
                twist.linear.x = 0.1
                twist.angular.z = -float(error) / 100.0

            # CAS 2 : On voit seulement la ligne rouge (Le vert a disparu)
            elif M_red['m00'] > 0:

                cx_red = int(M_red['m10'] / M_red['m00'])
                cy_red = int(M_red['m01'] / M_red['m00'])
                error = cx_red - (w / 4)
                v_auto = abs(float(error)) / 200.0
                self.err = v_auto

                if cy_red > h / 4 : 
                    twist.linear.x = v_auto * 0.01
                    twist.angular.z = v_auto # Virage gauche

                else:
                    twist.linear.x = 0.1
                    twist.angular.z = 0.0

            # CAS 3 : On voit seulement la ligne verte (Le rouge a disparu)
            elif M_green['m00'] > 0:

                cx_green = int(M_green['m10'] / M_green['m00'])
                cy_green = int(M_green['m01'] / M_green['m00'])
                error = cx_green - (w * 3 / 4)
                v_auto = abs(float(error)) / 200.0
                self.err = v_auto
                
                if cy_green > h / 4:
                    twist.linear.x = v_auto * 0.01
                    twist.angular.z = - v_auto
                else:
                    twist.linear.x = 0.1
                    twist.angular.z = 0.0

            # CAS 4 : Rien du tout
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0

            self.pub.publish(twist)

            # Affichage Debug
            cv2.imshow("Masques", cv2.bitwise_or(mask_red, mask_green))
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Erreur : {e}")
    
    def scan_callback(self, msg):
        # On regarde les distances devant le robot
        front_ranges = msg.ranges[0 : 20] + msg.ranges[340 : 360]  # 20 degrés à gauche et à droite
        ok_distance = [dist for dist in front_ranges if dist < msg.range_max and dist > msg.range_min]

        # 4. On prend la décision.
        if ok_distance and min(ok_distance) < 0.3: # Si un objet est à moins de 30cm
                self.obstacle_detecte = True
                stop_twist = Twist()
                self.pub.publish(stop_twist)
                return # On sort de la fonction
            
        # Si on arrive ici, c'est que c'est libre
        self.obstacle_detecte = False



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