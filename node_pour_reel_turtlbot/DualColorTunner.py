import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

class ColorTunerNode(Node):
    def __init__(self):
        super().__init__('color_tuner_node')
        
        # Subscription to your camera topic
        self.subscription = self.create_subscription(
            CompressedImage, 
            'camera/image_raw/compressed', 
            self.listener_callback, 
            10)
        
        # Initialize Trackbars Window
        cv2.namedWindow("Trackbars", cv2.WINDOW_NORMAL)
        
        # --- Range 1 Trackbars (Lower Red) ---
        cv2.createTrackbar("L-H1", "Trackbars", 0, 179, self.nothing)
        cv2.createTrackbar("L-S1", "Trackbars", 100, 255, self.nothing)
        cv2.createTrackbar("L-V1", "Trackbars", 70, 255, self.nothing)
        cv2.createTrackbar("U-H1", "Trackbars", 10, 179, self.nothing)
        cv2.createTrackbar("U-S1", "Trackbars", 255, 255, self.nothing)
        cv2.createTrackbar("U-V1", "Trackbars", 255, 255, self.nothing)

        # --- Range 2 Trackbars (Upper Red) ---
        cv2.createTrackbar("L-H2", "Trackbars", 160, 179, self.nothing)
        cv2.createTrackbar("L-S2", "Trackbars", 100, 255, self.nothing)
        cv2.createTrackbar("L-V2", "Trackbars", 70, 255, self.nothing)
        cv2.createTrackbar("U-H2", "Trackbars", 179, 179, self.nothing)
        cv2.createTrackbar("U-S2", "Trackbars", 255, 255, self.nothing)
        cv2.createTrackbar("U-V2", "Trackbars", 255, 255, self.nothing)

    def nothing(self, x):
        pass

    def get_trackbar_values(self):
        # Read all trackbar positions
        lh1 = cv2.getTrackbarPos("L-H1", "Trackbars")
        ls1 = cv2.getTrackbarPos("L-S1", "Trackbars")
        lv1 = cv2.getTrackbarPos("L-V1", "Trackbars")
        uh1 = cv2.getTrackbarPos("U-H1", "Trackbars")
        us1 = cv2.getTrackbarPos("U-S1", "Trackbars")
        uv1 = cv2.getTrackbarPos("U-V1", "Trackbars")

        lh2 = cv2.getTrackbarPos("L-H2", "Trackbars")
        ls2 = cv2.getTrackbarPos("L-S2", "Trackbars")
        lv2 = cv2.getTrackbarPos("L-V2", "Trackbars")
        uh2 = cv2.getTrackbarPos("U-H2", "Trackbars")
        us2 = cv2.getTrackbarPos("U-S2", "Trackbars")
        uv2 = cv2.getTrackbarPos("U-V2", "Trackbars")

        lower1 = np.array([lh1, ls1, lv1])
        upper1 = np.array([uh1, us1, uv1])
        lower2 = np.array([lh2, ls2, lv2])
        upper2 = np.array([uh2, us2, uv2])
        
        return lower1, upper1, lower2, upper2

    def listener_callback(self, msg):
        # Convert ROS Image to OpenCV format
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if frame is None:
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Get ranges from trackbars
        l1, u1, l2, u2 = self.get_trackbar_values()

        # Create two masks
        mask1 = cv2.inRange(hsv, l1, u1)
        mask2 = cv2.inRange(hsv, l2, u2)

        # Combine masks (Logical OR)
        combined_mask = cv2.bitwise_or(mask1, mask2)

        # Calculate Area of the combined mask
        pixel_count = cv2.countNonZero(combined_mask)

        # Apply mask to original frame
        res = cv2.bitwise_and(frame, frame, mask=combined_mask)

        # Add Overlay info
        cv2.putText(res, f"Area: {pixel_count}", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        # Display side-by-side
        combined_view = np.hstack([frame, res])
        cv2.imshow("Dual Mask Tuning (Original | Combined Result)", combined_view)
        
        key = cv2.waitKey(1)
        if key == 27:  # ESC to exit
            self.get_logger().info("Shutting down...")
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
