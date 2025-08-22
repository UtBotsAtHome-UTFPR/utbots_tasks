import rclpy
import yasmin
from yasmin import Blackboard, StateMachine, State
from yasmin_ros import ActionState, set_ros_loggers
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from utbots_actions.action import MPPose
from yasmin_viewer import YasminViewerPub
from cv_bridge import CvBridge

class FindObjectState(ActionState):
    """
    Class for searching and retrieving detections for one or more objects with several YOLO detections

    Attributes:
        action_server (str : "YOLO_batch_detection"): address of the YOLO action server
        verbose (bool : False): show debug information
    """
    def __init__(self, action_server="mediapipe_pose", verbose=False,) ->None:
        super().__init__(
            MPPose,
            action_server,
            self.create_goal_handler,
            self.response_handler,
            None,
        )
        self.verbose = verbose

    def create_goal_handler(self, blackboard: Blackboard) -> MPPose.Goal:
        goal = MPPose.Goal()
        
        if "mediapipe_img" in blackboard:
            img = blackboard["mediapipe_image"]

        goal.

        cv_image = self.bridge.imgmsg_to_cv2(img, desired_encoding="bgr8")
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