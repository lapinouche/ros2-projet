# to lunch only this node : ros2 launch projet2025 projet.launch.py x_pose:=-0.23 y_pose:=-0.23 yaw_angle:=-0.25
# This node is not working yet !

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import cv2
import mediapipe as mp

class HumanMotionControl(Node):
    def __init__(self):
        super().__init__('human_motion_control')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Initialize MediaPipe Hand Tracking
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.7
        )
        self.mp_draw = mp.solutions.drawing_utils
        
        # OpenCV Camera Setup
        self.cap = cv2.VideoCapture(0)
        
        # Timer to process frames at 20Hz
        self.timer = self.create_timer(0.05, self.process_frame)
        self.get_logger().info("Vision Teleop Node Started. Show your hand to the camera!")

    def process_frame(self):
        success, frame = self.cap.read()
        if not success:
            return

        # Flip frame horizontally for natural 'mirror' feel
        frame = cv2.flip(frame, 1)
        h, w, c = frame.shape
        left_line = w // 3
        right_line = 2*w // 3

        # Convert BGR to RGB for MediaPipe
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb_frame)

        msg = Twist()

        if results.multi_hand_landmarks:
            for hand_lms in results.multi_hand_landmarks:
                # Get the coordinates of the Index Finger Tip (Landmark 8)
                index_tip = hand_lms.landmark[8]
                cx = int(index_tip.x * w)

                # Control Logic
                if cx > right_line:
                    msg.linear.x = 0.3
                    msg.angular.z = -0.5  # Turn Right
                    label = "Turning Right"
                elif cx < left_line:
                    msg.linear.x = 0.3
                    msg.angular.z = 0.5   # Turn Left
                    label = "Turning Left"
                else:
                    msg.linear.x = 0.5
                    msg.angular.z = 0.0
                    label = "Go further"
                
                # Visual feedback on your screen
                self.mp_draw.draw_landmarks(frame, hand_lms, self.mp_hands.HAND_CONNECTIONS)
                cv2.putText(frame, label, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        else:
            # No hand detected = Stop
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            cv2.putText(frame, "No Hand - Stopped", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        # Draw the divider line
        cv2.line(frame, (left_line, 0), (left_line, h), (255, 255, 255), 2)
        cv2.line(frame, (right_line, 0), (right_line, h), (255, 255, 255), 2)
        
        # Publish the command to the robot
        self.publisher_.publish(msg)

        # Show the camera feed (useful for debugging in your Docker VNC)
        cv2.imshow("Teleop Monitor", frame)
        cv2.waitKey(1)

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VisionTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
