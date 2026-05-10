import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

class ColorTunerNode(Node):
    def __init__(self):
        super().__init__('color_tuner_node')
        
        # Change this topic to match your TurtleBot's camera (usually 'image_raw')
        #self.subscription = self.create_subscription(CompressedImage, 'camera/image_raw/compressed', self.listener_callback, 10)
        self.subscription = self.create_subscription(CompressedImage, '/image_raw/compressed', self.listener_callback, 10)
        
        # Initialize Trackbars
        cv2.namedWindow("Trackbars", cv2.WINDOW_NORMAL)
        cv2.createTrackbar("L-H", "Trackbars", 0, 179, self.nothing)
        cv2.createTrackbar("L-S", "Trackbars", 0, 255, self.nothing)
        cv2.createTrackbar("L-V", "Trackbars", 0, 255, self.nothing)
        cv2.createTrackbar("U-H", "Trackbars", 179, 179, self.nothing)
        cv2.createTrackbar("U-S", "Trackbars", 255, 255, self.nothing)
        cv2.createTrackbar("U-V", "Trackbars", 255, 255, self.nothing)

    def nothing(self, x):
        pass

    def listener_callback(self, msg):
        # Convert ROS Image to OpenCV format
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if frame is None:
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Get trackbar positions
        l_h = cv2.getTrackbarPos("L-H", "Trackbars")
        l_s = cv2.getTrackbarPos("L-S", "Trackbars")
        l_v = cv2.getTrackbarPos("L-V", "Trackbars")
        u_h = cv2.getTrackbarPos("U-H", "Trackbars")
        u_s = cv2.getTrackbarPos("U-S", "Trackbars")
        u_v = cv2.getTrackbarPos("U-V", "Trackbars")

        lower_range = np.array([l_h, l_s, l_v])
        upper_range = np.array([u_h, u_s, u_v])

        # Create Mask and Result
        mask = cv2.inRange(hsv, lower_range, upper_range)

        # Calculate Area
        pixel_count = cv2.countNonZero(mask)

        # Create Result Image
        res = cv2.bitwise_and(frame, frame, mask=mask)

        # Add Text Overlay to the result
        cv2.putText(res, f"Area: {pixel_count}", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Display Combined Window
        combined = np.hstack([frame, res])
        cv2.imshow("Real-time Tuning (Original | Filtered)", combined)
        
        # 7. Single waitKey for both GUI refresh and ESC key detection
        key = cv2.waitKey(1)
        if key == 27:  # ESC key
            self.get_logger().info("Closing node...")
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ColorTunerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
