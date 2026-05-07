ros2 launch projet2025 projet.launch.py x_pose:=-0.23 y_pose:=-0.23 yaw_angle:=-0.25

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
