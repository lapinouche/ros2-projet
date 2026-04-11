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

        self.prev_error = 0.0
        self.accumulated_error = 0.0

        self.state = "on center"
        # Publisher pour le mouvement
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info("Nœud de suivi de ligne opérationnel !")

    def get_centroid(self, mask):
        """Calcule le centre X d'un masque à l'aide des moments d'image """
        M = cv2.moments(mask)
        if M['m00'] > 0:
            return int(M['m10'] / M['m00'])
        return None
    
    def get_hough_x(self, mask, roi_height, side):
        """
        Calculates the X coordinate of the line at the bottom of the ROI 
        using Canny edges and Hough Transform.
        """
        # 1. Clean up the mask (Dilation helps Hough find lines)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=1)
        edges = cv2.Canny(mask, 50, 150)

        # 2. Detect mathematical lines (adjust thresholds as needed for your Gazebo world)
        # threshold: minimum number of intersections to "count" as a line
        # minLineLength: minimum length of a line segment in pixels
        # maxLineGap: maximum gap between pixels to still connect them into a single line
        lines = cv2.HoughLinesP(edges, rho=1, theta=np.pi/180, threshold=40, minLineLength=30, maxLineGap=10)

        if lines is None:
            return None

        valid_x = []
        for line in lines:
            x1, y1, x2, y2 = line[0]

            # Filter: Ignore near-horizontal lines (noise/horizon)
            if abs(y2 - y1) < 20:
                continue
                
            # Calculate the slope of the line (m = dy / dx)
            slope = (y2 - y1) / (x2 - x1) if (x2 - x1) != 0 else 999
            bottom_x = int((roi_height - y1) / slope + x1)
            
            valid_x.append(bottom_x)
            
        # Create a debug image
        """
        line_img = np.zeros_like(mask)
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(line_img, (x1, y1), (x2, y2), (255), 2)
        cv2.imshow("Detected Hough Lines", line_img)
        """
        
        return int(np.mean(valid_x)) if valid_x else None

    def listener_callback(self, msg):
        # 1. Décodage de l'image
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if image is None: return

        height, width, _ = image.shape
        # On ne regarde que le bas de l'image (la route devant le robot)
        roi = image[int(height*0.6):height, :]
        roi_height = roi.shape[0] # Get the height of the cropped image
        #roi = image[int(height*0.4):int(height*0.8), :] # to get a better view
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # 2. Masques de couleur (Ligne Verte et Ligne Rouge) 
        # Ajustez ces seuils si les couleurs sur le terrain réel diffèrent
        lower_green = np.array([35, 40, 40]) 
        upper_green = np.array([85, 255, 255])
        mask_green = cv2.inRange(hsv, lower_green, upper_green)
        # filtrage de la couleur vert
        kernel = np.ones((5, 5), np.uint8)
        mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, kernel) # This fills small holes inside the object
        mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel) # Optional: This removes tiny speckles of noise
        mask_green = cv2.dilate(mask_green, kernel, iterations=1) # This makes the green line appear thicker to the algorithm
        
        lower_red = cv2.inRange(hsv, np.array([0, 100, 50]), np.array([10, 255, 255]))
        upper_red = cv2.inRange(hsv, np.array([160, 100, 50]), np.array([180, 255, 255]))
        mask_red = cv2.bitwise_or(lower_red, upper_red)
        # filtrage de la couleur rouge
        kernel = np.ones((5, 5), np.uint8)
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel) # This fills small holes inside the object
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel) # Optional: This removes tiny speckles of noise

        # 3. Calcul des positions avec Hough
        #gx = self.get_hough_x(mask_green, roi_height, side="left")
        #rx = self.get_hough_x(mask_red, roi_height, side="right")

        gx = self.get_centroid(mask_green)
        rx = self.get_centroid(mask_red)

        twist = Twist()
        twist.linear.x = 0.08  #  Vitesse lente et prudente pour la démo 

        # to try for fun : D_GAIN = 0.01, P_GAIN = 0.08, I = 0
        # total instability started at D = 0, P = 0.05 => P = 0.025
        P_GAIN = 0.03 # 0.012 give a bit more snap that 0.008 | P-Gain (how violently the robot turns to correct an error)
        D_GAIN = 0.003 # 0.01 
        I_GAIN = 0.008
        Left_Target = 0.08 # 0.35 is too close to the middles (0.5), 0.2 keeps the green line on the far edge of the robot's vision | left and right target represent where we want the line to be in the camera's field view when only one line is visible !
        Right_Target = 1 - Left_Target
        #Right_Target = 0.9

        # 4. Logique de suivi (Rester entre les deux lignes) 
        if gx is not None and rx is not None:
            # On veut que le milieu entre les deux lignes soit au centre de l'image
            middle_line = (gx + rx) / 2
            error = middle_line - (width / 2)
            derivative = error - self.prev_error
            self.accumulated_error += error
            self.accumulated_error = max(min(self.accumulated_error, 10.0), -10.0)
            self.prev_error = error
            
            if self.state != "on center":
                twist.linear.x = 0.08
                self.state = "on center"
            
            # PD Formula
            twist.angular.z = -(float(error) * P_GAIN + float(derivative)*D_GAIN + self.accumulated_error*I_GAIN) # Correction proportionnelle
            self.get_logger().info(f"Suivi des deux lignes - Erreur: {error:.2f}")
            
        elif gx is not None: # Seulement ligne verte visible (virage ou bord)
            error = gx - (width * Left_Target) # On essaie de garder la vert à gauche
            
            if self.state == "on center": # Just lost the red line!
                self.prev_error = error # Reset D to avoid a jerk
                self.state = "out of center"
            
            derivative = error - self.prev_error
            self.prev_error = error
            self.accumulated_error += error
            self.accumulated_error = max(min(self.accumulated_error, 10.0), -10.0)
            
            twist.linear.x = 0.04
            RECOVERY_P = P_GAIN * 1.5

            twist.angular.z = 0.05 -(float(error) * P_GAIN + float(derivative)*(D_GAIN/2) + self.accumulated_error*I_GAIN)
            
            self.get_logger().info(f"Récupération Ligne Rouge - Erreur: {error:.2f}")
            
        elif rx is not None: # Seulement ligne rouge visible
            error = rx - (width * Right_Target) # On essaie de garder la rouge à droite
            
            if self.state == "on center": # Just lost the green line!
                self.prev_error = error # Reset D to avoid a jerk
                self.state = "out of center"
            
            derivative = error - self.prev_error
            self.prev_error = error
            self.accumulated_error += error
            self.accumulated_error = max(min(self.accumulated_error, 10.0), -10.0)
            
            twist.linear.x = 0.04
            RECOVERY_P = P_GAIN * 1.5

            twist.angular.z = 0.05 -(float(error) * P_GAIN + float(derivative)*(D_GAIN/2) + self.accumulated_error*I_GAIN)
            
            self.get_logger().info(f"Récupération Ligne Vert - Erreur: {error:.2f}")

        else:
            twist.linear.x = 0.3 # On ne voit rien : STOP de sécurité (0.0, 0.3 pour le test)
            self.get_logger().warn("Lignes perdues !")
        

        # 5. Publication et Affichage (indispensable pour débugger !)
        self.cmd_vel_pub.publish(twist)
        
        # Visualisation pour votre binôme et l'évaluateur [cite: 76]
        #cv2.imshow("Masque Vert", mask_green)
        #cv2.imshow("Masque Rouge", mask_red)
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
