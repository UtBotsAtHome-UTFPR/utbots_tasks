import rclpy
import yasmin
from yasmin import Blackboard, StateMachine, State
from yasmin_ros import ActionState, set_ros_loggers
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from utbots_actions.action import MPPose
from yasmin_viewer import YasminViewerPub
from cv_bridge import CvBridge

class GetPersonPointState(ActionState):
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
            [SUCCEED, CANCEL, ABORT],
            self.response_handler,
            None,
        )
        self.verbose = verbose

    def create_goal_handler(self, blackboard: Blackboard) -> MPPose.Goal:
        
        goal = MPPose.Goal()

        goal.get_torso_point.data = True
        goal.get_drawn.data = True

        if "mediapipe_img" in blackboard:
            goal.image = blackboard["mediapipe_img"]

        return goal

    def response_handler(self, blackboard: Blackboard, response: MPPose.Result) -> str:

        blackboard["mediapipe_skeleton_img"] = response.skeleton_img
        blackboard["mediapipe_skeleton_point"] = response.point
        blackboard["mediapipe_points_normalized"] = response.skeleton_points_normalized

        return SUCCEED