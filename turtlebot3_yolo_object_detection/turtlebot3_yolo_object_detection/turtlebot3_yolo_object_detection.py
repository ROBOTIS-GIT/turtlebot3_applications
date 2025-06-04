import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__("turtlebot3_object_detection_node")

        self.model = YOLO("/home/ubuntu/best.pt")  # Update with your actual model path
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image, "/camera/image_raw", self.image_callback, 10)
        self.image_pub = self.create_publisher(
            CompressedImage, "/camera/detections/compressed", 10)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        results = self.model(frame)
        annotated_frame = np.array(results[0].plot())
        annotated_frame = cv2.cvtColor(annotated_frame, cv2.COLOR_RGB2BGR)
        success, encoded_image = cv2.imencode('.jpg', annotated_frame)
        if success:
            compressed_image = CompressedImage()
            compressed_image.header = msg.header
            compressed_image.format = "jpeg"
            compressed_image.data = encoded_image.tobytes()
            self.image_pub.publish(compressed_image)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
