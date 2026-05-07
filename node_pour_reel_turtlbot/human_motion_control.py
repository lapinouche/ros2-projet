# to lunch only this node : ros2 launch projet2025 projet.launch.py x_pose:=-0.23 y_pose:=-0.23 yaw_angle:=-0.25

import rclpy
from rclpy.node import Node
#from sensor_msgs.msg import Image
from sensor_msgs.msg  import CompressedImage
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
#from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import numpy as np
import threading
from flask import Flask, Response

output_frame = None
lock = threading.Lock()
app = Flask(__name__)

@app.route('/video')
def video():
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

def gen():
    global output_frame, lock
    while True:
        whith lock:
            if output_frame is None:
                continue
            _, jpeg = cv2.imencode('.jpg', frame)
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')

class HumanMotionControl(Node):
    def __init__(self):
        super().__init__('human_motion_control')
        #self.bridge = CvBridge()

        # OpenCV Camera Setup
        self.cap = cv2.VideoCapture(1)

        #self.subscription = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        #self.subscription = self.create_subscription(CompressedImage, '/camera/image_raw/compressed', self.image_callback, 10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_robot = self.create_subscription(Int32, '/robot_state', self.state_callback, 10)
        
        # Initialize MediaPipe Hand Tracking
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.7
        )
        self.mp_draw = mp.solutions.drawing_utils

        # or self.cap = cv2.VideoCapture('test_hand.mp4') # to use video to try the code on simulation (see picture of gemini text)
        # Timer to process frames at ~30 FPS
        self.timer = self.create_timer(0.033, self.image_callback)

        # Make node check that it's her time to publish
        self.current_state = -1
        self.get_logger().info("Vision Teleop Node Started. Show your hand to the camera!") 
        
    def state_callback(self, msg):
        self.current_state = msg.data

    def image_callback(self, msg):
        if self.current_state not in [4]:
            return

        global output_frame, lock
        ret, frame = self.cap.read()
        if not ret:
            return
        
        #frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        h, w, _ = frame.shape

        # Process Hand Landmarks (Convert BGR to RGB for MediaPipe)
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb_frame)

        msg = Twist()
        if results.multi_hand_landmarks:
            for hand_lms in results.multi_hand_landmarks:
                # Get the coordinates of the Index Finger Tip (Landmark 8)
                index_tip = hand_lms.landmark[8]
                cx = int(index_tip.x * w)

                # Control Logic
                if cx < w / 3:
                    msg.linear.x = 0.3
                    msg.angular.z = 0.5  # Turn Left
                    label = "Turning Left"
                elif w / 3 <= cx <= 2*w / 3:
                    msg.linear.x = 0.5
                    msg.angular.z = 0.0  
                    label = "Forward"
                else:
                    msg.linear.x = 0.3
                    msg.angular.z = - 0.5 # Turn Right
                    label = "Turning Right"
                
                self.mp_draw.draw_landmarks(frame, hand_lms, self.mp_hands.HAND_CONNECTIONS)
        
        else:
            # No hand detected = Stop
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        # Publish the command to the robot
        self.publisher_.publish(msg)

        # Draw the divider line
        cv2.line(frame, (int(w/3), 0), (int(w/3), h), (255, 255, 255), 1)
        cv2.line(frame, (int(2*w/3), 0), (int(2*w/3), h), (255, 255, 255), 1)

        # Mise à jour de la frame pour Flask
        with lock:
            output_frame = frame.copy()
            
def main(args=None):
    rclpy.init(args=args)
    node = HumanMotionControl()

    # Lancer Flask dans un thread séparé pour ne pas bloquer ROS
    threading.Thread(target=lambda: app.run(host='0.0.0.0', port=8080, debug=False, use_reloader=False), daemon=True).start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
