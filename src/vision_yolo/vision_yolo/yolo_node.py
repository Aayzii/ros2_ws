import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

import cv2
from ultralytics import YOLO


class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')

        self.bridge = CvBridge()

        self.model = YOLO('yolov8n.pt')
        self.get_logger().info("YOLO model loaded")

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.sub = self.create_subscription(
            Image,
            '/camera/image',   # ✅ correct topic
            self.image_callback,
            qos
        )

        self.pub = self.create_publisher(String, '/detections', 10)

    def image_callback(self, msg):
        self.get_logger().info("Image received")

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        cv2.imshow("camera", frame)
        cv2.waitKey(1)

        results = self.model(frame, verbose=False)

        detected = []
        for r in results:
            if r.boxes is None:
                continue
            for c in r.boxes.cls:
                detected.append(self.model.names[int(c)])

        if detected:
            msg_out = String()
            msg_out.data = ', '.join(detected)
            self.pub.publish(msg_out)
            self.get_logger().info(f"Detected: {msg_out.data}")


def main():
    rclpy.init()
    node = YoloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
