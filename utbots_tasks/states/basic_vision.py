import rclpy
import yasmin
from yasmin import Blackboard, StateMachine, State
from yasmin_ros import ActionState, MonitorState, set_ros_loggers
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from utbots_actions.action import YOLOBatchDetection
from std_msgs.msg import String, Int32, Float32
from yasmin_viewer import YasminViewerPub
from utbots_tasks.states.basic_face import USBCamOff, USBCamOn, RecognitionState
from utbots_tasks.states.basic_mediapipe import GetPersonPointState
from cv_bridge import CvBridge
import cv2
import time
import numpy as np
from utbots_actions.action import MPPose
from sensor_msgs.msg import Image, PointCloud2

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

custom_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    depth=10
)

class FindObjectState(ActionState):
    """
    Class for searching and retrieving detections for one or more objects with several YOLO detections

    Attributes:
        action_server (str : "YOLO_batch_detection"): address of the YOLO action server
        verbose (bool : False): show debug information
    """
    def __init__(self, action_server="YOLO_batch_detection", verbose=False,) ->None:
        super().__init__(
            YOLOBatchDetection,
            action_server,
            self.create_goal_handler,
            [SUCCEED, 'not_detected', CANCEL, ABORT],
            self.response_handler,
            None,
        )
        self.verbose = verbose

    def create_goal_handler(self, blackboard: Blackboard) -> YOLOBatchDetection.Goal:
        goal = YOLOBatchDetection.Goal()
        objects = blackboard["objects"]

        if self.verbose:
            yasmin.YASMIN_LOG_INFO(f"[DEBUG] Target objects: {objects}")
        
        if isinstance(objects, str):
            goal.target_categories = [String(data=objects)]
        elif isinstance(objects, list):
            goal.target_categories = [String(data=obj) for obj in objects]
        else:
            raise ValueError("Expected 'objects' to be a string or a list.")
        
        goal.batch_size = Int32()
        goal.batch_size.data = blackboard["batch_size"]
        goal.iou_threshold = Float32()
        goal.iou_threshold.data = blackboard["iou_threshold"]
        goal.support_threshold = Float32()
        goal.support_threshold.data = blackboard["support_threshold"]
        if self.verbose:
            yasmin.YASMIN_LOG_INFO(f"[DEBUG] Goal message: {goal}")

        return goal

    def response_handler(self, blackboard: Blackboard, response: YOLOBatchDetection.Result) -> str:
        detections = response.detected_objs.bounding_boxes
        blackboard["annotated_img"] = response.annotated_image
        blackboard["detections"] = detections if detections else []
        
        if self.verbose:
            yasmin.YASMIN_LOG_INFO(f"[DEBUG] Detections: ")
            # for detection in detections:
        
        yasmin.YASMIN_LOG_INFO(f"[DEBUG] {detections}")
        
        return SUCCEED if detections else "not_detected"

class FramePerson(State):
    def __init__(self) -> None:
        super().__init__([SUCCEED, ABORT])
        self.bridge = CvBridge()

    def execute(self, blackboard: Blackboard) -> str:
        # yasmin.YASMIN_LOG_INFO("Executing state FOO")
        people = blackboard["people"] # Face recognition output
        detections = blackboard["detections"] # Yolo output

        img = blackboard["annotated_img"]
        cv_image = self.bridge.imgmsg_to_cv2(img, desired_encoding="bgr8")

        face_bbox = None
        for person in people:
            if person.id != "Unknown":
                face_bbox = {}
                face_bbox["xmin"] = person.xmin
                face_bbox["xmax"] = person.xmax
                face_bbox["ymin"] = person.ymin
                face_bbox["ymax"] = person.ymax
                face_bbox["name"] = person.id
        
        if not face_bbox:
            return ABORT

        people_bbox = []
        for detection in detections:
            people_bbox.append({})

            people_bbox[-1]["xmin"] = detection.xmin
            people_bbox[-1]["xmax"] = detection.xmax
            people_bbox[-1]["ymin"] = detection.ymin
            people_bbox[-1]["ymax"] = detection.ymax

        face_height = face_bbox["ymax"] - face_bbox["ymin"]

        extracted_bbox = None
        max_intersect = 0
        for bbox in people_bbox:

            inter_x_min = max(face_bbox["xmin"], bbox["xmin"])
            inter_y_min = max(face_bbox["ymin"], bbox["ymin"])
            inter_x_max = min(face_bbox["xmax"], bbox["xmax"])
            inter_y_max = min(face_bbox["ymax"], bbox["ymin"] + face_height) # In case someone is behind a seated known person

            inter_width = max(0, inter_x_max - inter_x_min)
            inter_height = max(0, inter_y_max - inter_y_min)

            inter_area = inter_width * inter_height

            area1 = (face_bbox["xmax"] - face_bbox["xmin"]) * (face_bbox["ymax"] - face_bbox["ymin"])
            area2 = (bbox["xmax"] - bbox["xmax"]) * (bbox["ymax"] - bbox["ymin"])

            union_area = area1 + area2 - inter_area

            # Compute IoU
            if union_area == 0:
                return 0.0
            if (inter_area / union_area) > max_intersect:
                max_intersect = inter_area / union_area
                extracted_bbox = bbox

        h, w, a = cv_image.shape

        mask = np.zeros((h, w), dtype=np.uint8)

        mask[extracted_bbox["ymin"]:extracted_bbox["ymax"], extracted_bbox["xmin"]:extracted_bbox["xmax"]] = 255

        mask_img = cv2.bitwise_and(cv_image, cv_image, mask=mask)
        
        identified_image = self.bridge.cv2_to_imgmsg(mask_img, encoding="bgr8")

        blackboard["cropped_person"] = identified_image
        # TODO: passar essa imagem pro mediapipe estimar a pose da pessoa
        # cv2.imshow("img", mask_img)
        # cv2.waitKey(0)

        return SUCCEED

class GetPersonPositiontate(MonitorState):
    def __init__(self) -> None:
        super().__init__(Image, 
                         "/kinect2/sd/image_depth_rect", 
                         [SUCCEED, ABORT], 
                         self.monitor_handler,
                         qos=custom_qos,
                         msg_queue=10, 
                         timeout=30)
        self.cvBridge = CvBridge()
        
    def monitor_handler(self, blackboard: Blackboard, msg: Image) -> str:

        cv_image = self.cvBridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        # Determine the pixels for the skeleton positions 
        person_points = blackboard["mediapipe_points_normalized"]

        valid_positions = []
        for point in person_points.points:
            # If point is at the image (mediapipe creates estimated points outside)
            if point.x > 0 and point.x < 1 and point.y > 0 and point.y < 1:
                point.x = point.x * msg.width
                point.y = point.y * msg.height
                
                # Kinect depth sensor has less width than rgb and it's filled with 0
                if cv_image[int(point.x)][int(point.y)] > 0:
                    point.z = cv_image[int(point.x)][int(point.y)] / 1000 # millimiter to meter conversion
                    valid_positions.append(point)
        
        if not valid_positions:
            return ABORT
        
        distance = 0
        for point in valid_positions:
            distance += point.z
        distance /= len(valid_positions)

        # Convert distance to x/y/z coordinates (y doesn't matter but is needed for estimation)

        return SUCCEED

def locate_person_from_face():
    yasmin.YASMIN_LOG_INFO("locate_person_from_face_demo")
    rclpy.init()
    
    node = rclpy.create_node("locate_person_from_face_sm")

     # Set up ROS 2 logs
    set_ros_loggers()

    sm = StateMachine(outcomes=[SUCCEED, CANCEL, ABORT])

    sm.add_state(
        "USBCAM_ON_STATE",
        USBCamOn(),
        transitions={
            SUCCEED: "RECOGNIZE",
            ABORT: ABORT,
        },
    )

    sm.add_state(
        "RECOGNIZE",
        RecognitionState(),
        transitions={
            SUCCEED: "FIND_PEOPLE",
            ABORT: ABORT,
        }
    )

    sm.add_state(
        "FIND_PEOPLE",
        FindObjectState(action_server="/yolo_node_coco/YOLO_batch_detection", verbose=True),
        transitions={
            SUCCEED: "FRAME_PERSON",
            'not_detected': ABORT,
            CANCEL: CANCEL,
            ABORT: ABORT
        }
    )

    sm.add_state(
        "USBCAM_OFF_STATE",
        USBCamOff(),
        transitions={
            SUCCEED: "FRAME_PERSON",
            ABORT: ABORT,
        },
    )

    sm.add_state(
        "FRAME_PERSON",
        FramePerson(),
        transitions={
            SUCCEED:"TRACK_PERSON_FROM_CROPPED",
            ABORT:ABORT
        }
    )

    sm.add_state(
        "TRACK_PERSON_FROM_CROPPED",
        GetPersonPointState(),
        transitions={
            SUCCEED:"TRACK_PERSON",
            ABORT:ABORT
        },
        remappings = {"mediapipe_img" : "cropped_person"}
    )

    # Estado track person com a imagem não cropada pras partes que não terão bounding box
    sm.add_state(
        "TRACK_PERSON",
        GetPersonPointState(),
        transitions={
            SUCCEED:"GET_PERSON_POSE_FROM_TORSO",
            ABORT:ABORT
        },
    )

    # Estado monitor de estimar a posição da pessoa a partir da posição da posição do torso dela
    sm.add_state(
        "GET_PERSON_POSE_FROM_TORSO",
        GetCurrentPoseState(),
        transitions={
            SUCCEED:"TRACK_PERSON",
            ABORT:ABORT
        },
    )

    sm.add_state(
        "USBCAM_OFF_STATE2",
        USBCamOff(),
        transitions={
            SUCCEED: SUCCEED,
            ABORT: ABORT,
        },
    )

    blackboard = Blackboard()

    # Face recognition variables
    blackboard["person_name"] = "Teste"
    blackboard['objects'] = ['person']

    # Yolo variables
    blackboard["batch_size"] = 10
    blackboard["iou_threshold"] = 0.5
    blackboard["support_threshold"] = 0.6

    # Publish FSM information
    YasminViewerPub("YASMIN_ACTION_CLIENT_DEMO", sm)

    try:
        outcome = sm(blackboard)
        yasmin.YASMIN_LOG_INFO(outcome)
    except KeyboardInterrupt:
        if sm.is_running():
            sm.cancel_state()  # Cancel the state if interrupted

    # Shutdown ROS
    if rclpy.ok():
        rclpy.shutdown()
    
def find_seat_sm():
    yasmin.YASMIN_LOG_INFO("yasmin_action_client_demo")
    rclpy.init()
    node = rclpy.create_node("receptionist_sm")

     # Set up ROS 2 logs
    set_ros_loggers()

    sm = StateMachine(outcomes=[SUCCEED, CANCEL, ABORT])
    sm.add_state(
        "FIND_PEOPLE",
        FindObjectState(action_server="/yolo_node_coco/YOLO_batch_detection", verbose=True),
        transitions={
            SUCCEED: "FIND_SEAT",
            'not_detected': "FIND_SEAT",
            CANCEL: CANCEL,
            ABORT: ABORT
        },
        remappings={"objects": "person", "detections": "bboxes2"},
    )
    sm.add_state(
        "FIND_SEAT",
        FindObjectState(action_server="/yolo_node_coco/YOLO_batch_detection", verbose=True),
        transitions={
            SUCCEED: "FIND_DRINKS",
            'not_detected': "FIND_DRINKS",
            CANCEL: CANCEL,
            ABORT: ABORT
        },
        remappings={"objects": "seat", "detections": "bboxes1"},
    )
    sm.add_state(
        "FIND_DRINKS",
        FindObjectState(action_server="/yolo_node_home/YOLO_batch_detection", verbose=True),
        transitions={
            SUCCEED: SUCCEED,
            'not_detected': SUCCEED,
            CANCEL: CANCEL,
            ABORT: ABORT
        },
        remappings={"objects": "drinks", "detections": "bboxes1"}, 
    )

    blackboard = Blackboard()
    blackboard["iou_threshold"] = 0.5
    blackboard["support_threshold"] = 0.4
    blackboard["batch_size"] = 50
    blackboard['person'] = ['person']
    blackboard['seat'] = ['chair', 'couch']
    blackboard['drinks'] = ['drinks-drinks-cofee', 'drinks-coke', 'drinks-fanta', 'drinks-kuat', 'drinks-milk', 'drinks-orange_juice']

    # Publish FSM information
    YasminViewerPub("YASMIN_ACTION_CLIENT_DEMO", sm)

    try:
        outcome = sm(blackboard)
        yasmin.YASMIN_LOG_INFO(outcome)
    except KeyboardInterrupt:
        if sm.is_running():
            sm.cancel_state()  # Cancel the state if interrupted

    # Shutdown ROS
    if rclpy.ok():
        rclpy.shutdown()

def main():
    #find_seat_sm()
    locate_person_from_face()

if __name__ == "__main__":
    main()

