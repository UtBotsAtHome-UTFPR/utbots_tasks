import cv2
import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from reportlab.lib.pagesizes import A4
from reportlab.pdfgen import canvas
from reportlab.lib.units import inch
from rclpy.node import Node
from datetime import datetime

class DetectionLogState(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.bridge = CvBridge()

    def execute(self, blackboard: Blackboard) -> str:
        yasmin.YASMIN_LOG_INFO("person_recognition_sm started")

        try:
            labeled_img: Image = blackboard["annotated_img"]
            bboxes = blackboard["detections"] # adjust type if needed

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
                detected_class = getattr(bbox, "Class", getattr(bbox, "id", "Unknown"))
                c.drawString(100, text_height, f"- {detected_class}")
                text_height -= 12

            c.showPage()
            c.save()

            yasmin.YASMIN_LOG_INFO(f"PDF saved to {pdf_path}")
            return SUCCEED

        except Exception as e:
            yasmin.YASMIN_LOG_INFO(f"Error while saving detection log: {e}")
            return ABORT
        
class CrowdLogState(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.bridge = CvBridge()
        self.pdf_path = "/tmp/personal_recognition.pdf"

    def execute(self, blackboard: Blackboard) -> str:
        yasmin.YASMIN_LOG_INFO("Saving crowd log")

        try:
            # Create PDF
            c = canvas.Canvas(self.pdf_path, pagesize=A4)
            c.setPageSize(A4)
            c.setFont("Helvetica", 12)

            # People count
            detections = blackboard["detections"]
            people_count = len(detections)
            c.drawString(100, 400, f"People Count: {people_count}")
            blackboard["people_count"] = people_count

            # Annotated person image
            c.drawString(100, 420, "Annotated Crowd Image:")
            annotated_img = blackboard["annotated_img"]
            cv_image = self.bridge.imgmsg_to_cv2(annotated_img, desired_encoding='bgr8')

            # Save image temporarily
            image_filename = "/tmp/crowd_log_img.jpg"
            cv2.imwrite(image_filename, cv_image)
            c.drawImage(image_filename, x=85, y=440, width=6*inch, height=4.5*inch)

            face_detection = blackboard['recognized_img']
            # Save recognized face image temporarily
            face_image = self.bridge.imgmsg_to_cv2(face_detection, desired_encoding='bgr8')
            face_image_filename = "/tmp/crowd_face_log_img.jpg"
            cv2.imwrite(face_image_filename, face_image)
            c.drawString(100, 420 - 15, "Recognized Faces Image:")
            c.drawImage(face_image_filename, x=85, y=200, width=6*inch, height=2.5*inch)
            
            c.showPage()
            return SUCCEED

        except Exception as e:
            yasmin.YASMIN_LOG_INFO(f"Error while processing annotated image: {e}")
            return ABORT
            
        ### Adicionar detecção de rosto

            
class AnswersLogState(State):
    def __init__(self, node: Node):
        super().__init__(outcomes=['log_saved', 'aborted'], input_keys=['nlu_input', 'nlu_output'])
        self.node = node
        self.question_number = 1
        self.log = ""

    def execute(self, blackboard: Blackboard) -> str:
        self.node.get_logger().info("Executing AnswersLogState")

        try:
            nlu_input: String = blackboard['nlu_input']
            nlu_output: String = blackboard['nlu_output']

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