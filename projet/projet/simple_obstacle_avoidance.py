import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
import numpy as np
import cv2

# Use "ros2 launch projet2025 projet.launch.py x_pose:=-0.8 y_pose:=0.9 yaw_angle:=-1.57"

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        # Subscriptions
        self.sub_lidar = self.create_subscription(LaserScan, '/scan', self.lidar_cb, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Make node check that it's her time to publish
        self.current_state = -1
        self.state_robot = self.create_subscription(Int32, '/robot_state', self.state_callback, 10)
        
        self.last_dist_front = 1.0
        self.last_dist_right = 1.0
        self.last_dist_left = 1.0

        self.passing_obs_1 = True
        self.passing_obs_2 = False

        self.message_1 = False
        self.message_2 = False
        self.cpt = 0

        # to come back after passing an obstacle
        self.cpt_get_on_center = 0
        self.stop_loop = 30
        self.close_dist_front = False
        self.emergency_avoidance = False

        self.back_up = False

    def state_callback(self, msg):
        self.current_state = msg.data

    def lidar_cb(self, msg):
        if self.current_state not in [1]:
            #self.get_logger().info("exit current state not valid")
            return
        
        # Filter helper: returns the shortest valid distance in a zone
        def get_min_dist(zone):
            valid = [d for d in zone if 0.1 < d < 0.8] # 0.8m threshold
            #return min(valid) if valid else float('inf')
            return min(valid) if valid else 1.0

        # Define strict detection zones (indices for TurtleBot3)
        front_ranges = msg.ranges[0:20] + msg.ranges[340:360] # Front: -20 to +20 degrees
        left_ranges = msg.ranges[20:70] # Left: 20 to 70 degrees
        right_ranges = msg.ranges[290:340] # Right: 290 to 340 degrees

        dist_front = get_min_dist(front_ranges)
        dist_left = get_min_dist(left_ranges)
        dist_right = get_min_dist(right_ranges)

        #self.get_logger().info("-------------------------")
        #self.get_logger().info(f"dist_front = {dist_front}")
        #self.get_logger().info(f"dist_left = {dist_left}")
        #self.get_logger().info(f"dist_right = {dist_right}")

        # 3. Logic Priority (Emergency Stop -> Turn -> Cruise)
        twist = Twist()

        if dist_front < 0.2 or self.emergency_avoidance :
            self.get_logger().info("condition 1 - dist_front < 0.2")
            twist.linear.x = -0.025 # - 0.02
            self.back_up = True
            self.cpt_get_on_center = 0
            if not self.emergency_avoidance :
                self.emergency_avoidance = True
            if dist_front > 0.3 and (dist_left == 1.0 or dist_right == 1.0):
                self.emergency_avoidance = False
            # Decide to steer away
            #twist.angular.z = 0.8 if dist_right > dist_left else -0.8 # Steer slightly left
            if self.passing_obs_1:
                twist.angular.z = -0.8 # 0.8
            elif self.passing_obs_2:
                twist.angular.z = 0.8 # 0.8
            else:
                twist.angular.z = 0.0
                self.get_logger().info("obstacle shouldn't be there")

        elif (self.close_dist_front == True and dist_front == 1.0 and (dist_right == 1.0 or dist_left == 1.0)) or self.cpt_get_on_center != 0:
            self.get_logger().info("condition 2 - transition")
            self.cpt_get_on_center += 1

            if self.passing_obs_1:
                self.stop_loop = 25 # 33
                if self.cpt_get_on_center < 8: # 8
                    twist.linear.x = 1.0
                    twist.angular.z = 0.0
                if self.cpt_get_on_center < 12: # 15
                    self.get_logger().info(f"turning back 1 - cpt : {self.cpt_get_on_center}")
                    twist.linear.x = 0.02
                    twist.angular.z = 0.7
                elif self.cpt_get_on_center < 20: # 28
                    twist.linear.x = 0.06
                    twist.angular.z = 0.0
                else:
                    twist.linear.x = 0.06
                    twist.angular.z = -0.7
            
            if self.passing_obs_2:
                self.stop_loop = 15
                self.get_logger().info(f"turning back 2 - cpt : {self.cpt_get_on_center}")
                if self.cpt_get_on_center < 6:
                    twist.linear.x = 0.02
                    twist.angular.z = -0.6
                else:
                    twist.linear.x = 0.06
                    twist.angular.z = 0.0

            if self.cpt_get_on_center == self.stop_loop:
                self.cpt_get_on_center = 0
                self.close_dist_front = False

                if self.passing_obs_1:
                    self.passing_obs_1 = False
                    self.passing_obs_2 = True
                    if not self.message_1:
                        self.get_logger().info("PASSING TO NEXT OBSTACLE")
                        self.cpt+=1
                        if self.cpt == 3:
                            self.message_1 = True
                            self.cpt = 0
                elif self.passing_obs_2:
                    self.passing_obs_2 = False
                    if not self.message_2:
                        self.get_logger().info("FINISHING")
                        self.cpt+=1
                        if self.cpt == 3:
                            self.message_2 = True
                            self.cpt = 0
                else:
                    self.get_logger().info("end of node !?")
                    return


        elif dist_left < 0.5 and dist_front > 0.4: # 0.6
            self.get_logger().info("condition 4 - dist_left < 0.5")
            if self.back_up:
                self.back_up = False
            twist.linear.x = 0.04
            if self.last_dist_front < 0.2:
                twist.angular.z = 0.0
            else: 
                twist.angular.z = 0.3
            
        elif dist_right < 0.5 and dist_front > 0.4: # 0.6
            self.get_logger().info("condition 5 - dist_right < 0.5")
            if self.back_up:
                self.back_up = False
            twist.linear.x = 0.04
            twist.angular.z = 0.0
            if self.last_dist_front < 0.2:
                twist.angular.z = 0.0
            else: 
                twist.angular.z = -0.3

        elif dist_front <= 0.6 : # 0.8 and dist_left==1.0 and dist_right==1.0
            self.get_logger().info("last condition - dist_front < 0.6")
            if self.back_up:
                    self.back_up = False
            twist.linear.x = 0.04

            if self.passing_obs_1: # dist_right < dist_left
                twist.angular.z = -0.4 # 0.7
            elif self.passing_obs_2:
                twist.angular.z = 0.4
            else :
                twist.angular.z = 0.0

        else:
            twist.linear.x = 0.04
            twist.angular.z = 0.0

        self.last_dist_front = dist_front
        self.last_dist_right = dist_right
        self.last_dist_front = dist_front

        if dist_front < 0.3:
            self.close_dist_front = True

        self.cmd_vel_pub.publish(twist)

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
