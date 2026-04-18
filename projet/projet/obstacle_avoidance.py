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
        self.subscription_img = self.create_subscription(
            CompressedImage, 
            '/image_raw/compressed', # Make sure the topic name is correct!
            self.camera_callback, 
            10)
        
        # Create the subscriber for LIDAR data
        self.subscription_lidar = self.create_subscription(
            LaserScan,
            '/scan',           # The default topic for TurtleBot LiDAR
            self.scan_callback,
            10                 # QoS history depth
        )

        # Publisher to cmd_vel_obstacle
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def camera_callback(self, msg):
        # 1. Avoid cylinder
        max_range = 0.8 # Meters
        min_range = 0.3 # was 0.1

        # look in front of the robot to avoid obstacle
        front_left_side = msg.ranges[0:60] 
        valid_front_left = [l for l in front_left_side if min_range < l < max_range]

        front_right_side = msg.ranges[300:360]
        valid_front_right = [r for r in front_right_side if min_range < r < max_range]

        # look at the direct left and right to the robot (if object, don't turn yet)
        left_side = msg.ranges[80:100] 
        valid_left = [l for l in left_side if min_range < l < max_range]

        right_side = msg.ranges[260:280]
        valid_right = [r for r in right_side if min_range < r < max_range]

        is_obstacle_detected = False
        twist = Twist()

        if valid_front_left:
            if len(valid_left) != 0:
                dist_left = sum(valid_front_left) / len(valid_front_left)
                self.get_logger().info(f"dist_front_left : {dist_left}")
                is_obstacle_detected = True
            else:
                pass

            twist.angular.z = -1 # (0.7) Turn Right
            twist.linear.x = 0.03  # Slow down while avoiding

            self.cmd_vel_pub.publish(twist)
            break
        
        elif valid_left:
            if len(valid_left) != 0:
                dist_left = sum(valid_left) / len(valid_left)
                self.get_logger().info(f"dist_left : {dist_left}")
                is_obstacle_detected = True
            else:
                pass

            if dist_left > 3:
                is_obstacle_detected = False # Safe distance, try to keep between the line
                twist.linear.x = 0.07  # Keep going
            else:
                twist.angular.z = -0.3 # turn a little more
                twist.linear.x = 0.04  # Slow down !

            self.cmd_vel_pub.publish(twist)
            break

            
        elif valid_front_right:
            if len(valid_right) != 0:
                dist_right = sum(valid_front_right) / len(valid_front_right)
                self.get_logger().info(f"dist_front_right : {dist_right}")
                is_obstacle_detected = True
            else:
                pass

            twist.angular.z = 1 # (0.7) Turn Left
            twist.linear.x = 0.03  # Slow down while avoiding

            self.cmd_vel_pub.publish(twist)
            break

        elif valid_right
            if len(valid_right) != 0:
                dist_right = sum(valid_right) / len(valid_right)
                self.get_logger().info(f"dist_left : {dist_right}")
                is_obstacle_detected = True
            else:
                pass

            if dist_right > 3:
                is_obstacle_detected = False # Safe distance, try to keep between the line
                twist.linear.x = 0.07  # Keep going
            else:
                twist.angular.z = -0.3 # turn a little more
                twist.linear.x = 0.04  # Slow down !

            self.cmd_vel_pub.publish(twist)
            break
                
        # 3. If no standing cylinders were found, keep following the line
        if not is_obstacle_detected:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            # 1. Décodage de l'image
            
            # 2. Masques de couleur (line_following)
            # Ajustez ces seuils si les couleurs sur le terrain réel diffèrent

            #####
            ## TO BE DONE (or to be delet if using linear.x = 0 and angular.z = 0)
            #####

            #twist.linear.x = 0.0
            #twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)

        #cv2.imshow("Masque Bleu", mask_blue)
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
