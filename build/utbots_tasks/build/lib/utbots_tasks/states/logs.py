import cv2
import os
from yasmin import State, Blackboard
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_msgs.msg import Header
from vision_msgs.msg import BoundingBox2D  # substitua com o tipo correto se necessário
from cv_bridge import CvBridge
from reportlab.lib.pagesizes import A4
from reportlab.pdfgen import canvas
from reportlab.lib.units import inch
from rclpy.node import Node
import rclpy
import time
from datetime import datetime

class DetectionLogState(State):
    def __init__(self, node: Node):
        super().__init__(outcomes=['log_saved', 'aborted'])
        self.node = node
        self.bridge = CvBridge()

    def execute(self, blackboard: Blackboard) -> str:
        self.node.get_logger().info("Executing DetectionLogState...")

        try:
            labeled_img: Image = blackboard.get("labeled_img")
            bboxes = blackboard.get("bboxes").bounding_boxes  # adjust type if needed

            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(labeled_img, desired_encoding='bgr8')

            # Save image temporarily
            image_filename = "/tmp/log_img.jpg"
            cv2.imwrite(image_filename, cv_image)

            # Create timestamp string
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            pdf_path = f"/tmp/object_recognition_log_{timestamp}.pdf"

            # Create PDF
            c = canvas.Canvas(pdf_path, pagesize=A4)
            c.drawImage(image_filename, x=85, y=440, width=6*inch, height=4.5*inch)

            # Add label text
            c.setFont("Helvetica", 12)
            text_height = 420
            c.drawString(100, text_height, "Objects recognized:")
            text_height -= 15

            for bbox in bboxes:
                # If bbox.Class doesn't exist, replace with bbox.class_id or appropriate field
                detected_class = getattr(bbox, "Class", getattr(bbox, "class_id", "Unknown"))
                c.drawString(100, text_height, f"- {detected_class}")
                text_height -= 12

            c.showPage()
            c.save()

            self.node.get_logger().info(f"PDF saved to {pdf_path}")
            return 'log_saved'

        except Exception as e:
            self.node.get_logger().error(f"Error while saving detection log: {e}")
            return 'aborted'

class AnswersLogState(State):
    def __init__(self, node: Node):
        super().__init__(outcomes=['log_saved', 'aborted'], input_keys=['nlu_input', 'nlu_output'])
        self.node = node
        self.question_number = 1
        self.log = ""

    def execute(self, blackboard: Blackboard) -> str:
        self.node.get_logger().info("Executing AnswersLogState")

        try:
            nlu_input: String = blackboard.get('nlu_input')
            nlu_output: String = blackboard.get('nlu_output')

            question = nlu_input.data.replace("data: ", "")
            answer = nlu_output.data.replace("data: ", "")

            self.node.get_logger().info(f"Question: {question}")
            self.node.get_logger().info(f"Answer: {answer}")

            question_log = f"{self.question_number}- {question}\n{answer}\n"
            self.node.get_logger().info(f"Question log: {question_log}")

            self.question_number += 1
            self.log += question_log

            return 'log_saved'

        except Exception as e:
            self.node.get_logger().error(f"Exception in AnswersLogState: {e}")
            return 'aborted'
