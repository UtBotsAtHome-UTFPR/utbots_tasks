import rclpy
import yasmin
from yasmin import Blackboard, StateMachine, State
from yasmin_ros import ActionState, set_ros_loggers
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

        # TODO: Pegando a primeira pq o yolo batch detection está quebrado, fazer iou da face e da bounding box yolo em x do topo em y até +- 2x altura da face
        extracted_bbox = people_bbox[0]

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
            SUCCEED:"TRACK_PERSON",
            ABORT:ABORT
        }
    )

    sm.add_state(
        "TRACK_PERSON",
        GetPersonPointState(),
        transitions={
            SUCCEED:"USBCAM_OFF_STATE2",
            ABORT:ABORT
        },
        remappings = {"mediapipe_img" : "cropped_person"}
    )

    # Estado de identificar dentro de qual bbox está a face

    # Estado para croppar a imagem de acordo

    # Chamar mediapipe com a imagem cortada para pegar o ponto

    # Talvez tenha que fazer coisas pra colocar a posição em relação ao robô

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

