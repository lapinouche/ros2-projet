import rclpy
from rclpy.node import Node
#from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import numpy as np
#from cv_bridge import CvBridge
import cv2

class LineFollowerNode(Node):
    def __init__(self):
        super().__init__('line_follower_node')
        #self.bridge = CvBridge()
        #self.sub_img = self.create_subscription(Image, '/camera/image_raw', self.listener_callback, 10)
        self.sub_img = self.create_subscription(CompressedImage, '/camera/image_raw/compressed', self.listener_callback, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.obstacle_detecte = False
        self.in_roundabout = False
        self.turning = False
        self.last_line = "RV"
        self.roundabout_cooldown = 0  # To prevent flickering detection
        self.declare_parameter('direction', 'right')
        self.get_logger().info("Nœud de suivi de ligne démarré.")

        # Make node check that it's her time to publish
        self.current_state = 0
        self.state_robot = self.create_subscription(Int32, '/robot_state', self.state_callback, 10)

    def state_callback(self, msg):
        self.current_state = msg.data
    
    def listener_callback(self, msg):
        self.get_logger().info("node running")
        if self.current_state not in [0, 3]:
            self.get_logger().info("current_state incorrect")
            return
        try:
            if self.obstacle_detecte:
                self.get_logger().info("Obstacle détecté, arrêt du robot.")
                return # On ne fait rien tant qu'on a pas traité l'obstacle
            
            #cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            #cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h, w, _ = cv_image.shape

            #self.get_logger().info(f"Dimensions : {h} x {w}")
            
            # Au lieu de h/2 (qui voit trop loin), utilise h*0.7 ou h*0.75
            roi = cv_image[int(h * 0.6):h, 0:w]
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
            # --- MASQUES HSV ---
            #mask_red1 = cv2.inRange(hsv, (0, 60, 20), (0, 255, 255))
            #mask_red2 = cv2.inRange(hsv, (170, 40, 20), (170, 255, 255))

            mask_red1 = cv2.inRange(hsv, (0, 50, 30), (15, 255, 255))
            mask_red2 = cv2.inRange(hsv, (160, 50, 30), (180, 255, 255))
            mask_red = cv2.bitwise_or(mask_red1, mask_red2)
            mask_green = cv2.inRange(hsv, (25, 60, 60), (80, 255, 255))

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
                self.last_line = "RV"
                cx_red = int(M_red['m10'] / M_red['m00'])
                cx_green = int(M_green['m10'] / M_green['m00'])

                self.get_logger().info(f"cx_red = {cx_red}")
                self.get_logger().info(f"cx_green = {cx_green}")

                dist_between_lines = abs(cx_red - cx_green)
                if dist_between_lines < 25:
                    if not self.turning: # Only trigger once
                        self.get_logger().info("!!! ENTRANCE/EXIT DETECTED !!!")
                        self.turning = True
                        self.in_roundabout = not self.in_roundabout # Toggle state
                        self.turn_start_time = self.get_clock().now()

                # If we are in 'turning' mode, force the turn for at least 1.5 seconds
                if self.turning:
                    elapsed = (self.get_clock().now() - self.turn_start_time).nanoseconds / 1e9
                    if elapsed < 1.5: # Turn duration
                        twist.linear.x = 0.05
                        twist.angular.z = 0.6 if self.get_parameter('direction').value == 'left' else -0.6
                        self.pub.publish(twist)
                        return # Skip the rest of the logic until turn is done
                    else:
                        self.turning = False # Turn finished, go back to line following

                cible = (cx_red + cx_green) / 2
                error = cible - (w / 2)
                
                twist.linear.x = 0.08
                twist.angular.z = -float(error) / 100.0

            # CAS 2 : On voit seulement la ligne rouge (Le vert a disparu)
            elif M_red['m00'] > 0:
                self.last_line = "R"
                cx_red = int(M_red['m10'] / M_red['m00'])
                cy_red = int(M_red['m01'] / M_red['m00'])
                error = cx_red - (w / 4)
                v_auto = abs(float(error)) / 200.0

                if cy_red > h / 4 : # h/8 
                    twist.linear.x = v_auto * 0.01
                    twist.angular.z = v_auto # Virage gauche

                else:
                    twist.linear.x = 0.08
                    twist.angular.z = 0.0

            # CAS 3 : On voit seulement la ligne verte (Le rouge a disparu)
            elif M_green['m00'] > 0:
                self.last_line = "V"
                cx_green = int(M_green['m10'] / M_green['m00'])
                cy_green = int(M_green['m01'] / M_green['m00'])
                error = cx_green - (w * 3 / 4)
                v_auto = abs(float(error)) / 200.0
                
                if cy_green > h / 4: # h/8
                    twist.linear.x = v_auto * 0.01
                    twist.angular.z = - v_auto
                else:
                    twist.linear.x = 0.1
                    twist.angular.z = 0.0

            # CAS 4 : Rien du tout
            else:
                if self.last_line == "RV" :
                    twist.linear.x = 0.05
                    twist.angular.z = 0.0 # continuer tout droit
                elif self.last_line == "R":
                    twist.linear.x = 0.05
                    twist.angular.z = -0.4 # 0.7  Tourne à gauche sur place
                elif self.last_line == "V":
                    twist.linear.x = 0.05
                    twist.angular.z = 0.4 # Tourne à droite sur place

            self.pub.publish(twist)

            # Affichage Debug
            cv2.imshow("Camera lfn", cv_image)
            cv2.imshow("Masques", cv2.bitwise_or(mask_red, mask_green))
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Erreur : {e}")
    
    def scan_callback(self, msg):
        if self.current_state not in [0, 3]:
            return
            
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
