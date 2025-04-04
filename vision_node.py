# color_box_nav/vision_node.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import torch
import torchvision.transforms as transforms

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.subscription = self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()

        self.model = torch.load('/absolute/path/to/AOPT4.pt', map_location='cpu')
        self.model.eval()
        self.transform = transforms.Compose([transforms.ToTensor()])

        self.target_colors = {'red': 0, 'green': 1, 'blue': 2}  # Example class indices

    def image_callback(self, msg: Image):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        input_tensor = self.transform(cv_image).unsqueeze(0)  # Add batch dimension

        with torch.no_grad():
            predictions = self.model(input_tensor)[0]  # adjust depending on model output

        target = self.select_target(predictions)

        if target:
            twist = self.compute_twist(cv_image.shape[1], target)
            self.publisher.publish(twist)
        else:
            self.publisher.publish(Twist())  # stop if nothing seen

    def select_target(self, preds):
        # Expects format: boxes, labels, scores
        boxes = preds.get('boxes', [])
        labels = preds.get('labels', [])
        scores = preds.get('scores', [])

        best = None
        best_score = 0.5  # confidence threshold

        for box, label, score in zip(boxes, labels, scores):
            if score >= best_score:
                if label in self.target_colors.values():
                    best = box
                    best_score = score

        return best

    def compute_twist(self, image_width, box):
        x1, y1, x2, y2 = box
        cx = (x1 + x2) / 2

        offset = (cx - image_width / 2) / (image_width / 2)  # -1.0 to 1.0

        twist = Twist()
        twist.linear.x = 0.2
        twist.angular.z = -offset  # turn toward center

        return twist

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
