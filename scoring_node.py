
import rclpy
from rclpy.node import Node
# from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
# from cv_bridge import CvBridge
import cv2
import numpy as np

class ScoringNode(Node):
    def __init__(self):
        super().__init__('scoring_node')
        # self.bridge = CvBridge()
        # Souscription à l'image (Gazebo ou réel)
        # self.sub_img = self.create_subscription(Image, '/image_raw', self.listener_callback, 10)
        self.sub_img = self.create_subscription(CompressedImage, '/camera/image_raw/compressed', self.listener_callback, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def listener_callback(self, msg):
        # cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        cv_image = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        h, w, _ = cv_image.shape
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Masques
        # mask_ball = cv2.inRange(hsv, (25, 100, 100), (35, 255, 255)) # Jaune-vert
        mask_ball = cv2.inRange(hsv, (25, 10, 100), (90, 255, 255))
        mask_red1 = cv2.inRange(hsv, (0, 100, 50), (10, 255, 255))
        mask_red2 = cv2.inRange(hsv, (160, 100, 50), (180, 255, 255))
        mask_goal = cv2.bitwise_or(mask_red1, mask_red2)
        
        M_b = cv2.moments(mask_ball)
        M_g = cv2.moments(mask_goal)

        twist = Twist()

        if M_b['m00'] > 0: # On voit la balle
            cx_b = int(M_b['m10'] / M_b['m00'])

            if M_g['m00'] > 0: # On voit les poteaux (le rouge)
                # Ce point représente le milieu du but
                cx_g = int(M_g['m10'] / M_g['m00'])
                
                # Le robot calcule l'erreur : 
                # "Où est la balle par rapport au milieu de mes poteaux ?"
                error1 = cx_g - cx_b
                error2 = (cx_g + w/2) - cx_b
                error = (error1 + error2) / 2.0
                
                # Si l'erreur est petite, on est bien en face du but !
                if abs(error) < 20:
                    twist.linear.x = 0.8  # Tir puissant !
                    twist.angular.z = 0.0
                else:
                    # On s'aligne en tournant vers la balle
                    twist.linear.x = 0.2
                    twist.angular.z = float(error) / 100.0
            else:
                # On voit la balle mais pas encore les deux poteaux
                # On reste sur la balle
                twist.linear.x = 0.2
                twist.angular.z = float(w/2 - cx_b) / 100.0

        self.pub.publish(twist)
        cv2.imshow("Masques", cv2.bitwise_or(mask_ball, mask_goal))
        cv2.imshow("Image originale", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ScoringNode()
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
