import rclpy
from utbots_actions.action import YOLODetection
from utbots_msgs import BoundingBoxes

import yasmin
from yasmin import CbState, Blackboard, StateMachine
from yasmin_ros import ActionState
from yasmin_ros import set_ros_loggers
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from yasmin_viewer import YasminViewerPub


def initializeDetections(blackboard: Blackboard) -> str:
    blackboard["n_detections"] = 0
    return SUCCEED

def checkDetections(blackboard: Blackboard) -> str:
    if blackboard["n_detections"] < blackboard["threshold"]:
        return "redetect"
    else:
        return "found_candidate"

class VoteDetectionsState(ActionState):
    def __init__(self) -> None:
        super().__init__(
            YOLODetection,  # action type
            "/yolo_detection",  # action name
            self.create_goal_handler,  # callback to create the goal
            None,  # outcomes. Includes (SUCCEED, ABORT, CANCEL)
            self.response_handler,  # callback to process the response
            None,  # callback to process the feedback
        )

    def create_goal_handler(self, blackboard: Blackboard) -> YOLODetection.Goal:
        goal = YOLODetection.Goal()
        goal.target_category = blackboard["beverage"]
        return goal

    def response_handler(self, blackboard: Blackboard, response: YOLODetection.Result) -> str:
        result = YOLODetection.Result
        if len(result.detected_objs.bounding_boxes) > 0:
            blackboard["n_detections"] += 1
        return SUCCEED

def main():
    yasmin.YASMIN_LOG_INFO("yasmin_action_client_demo")

    # Initialize ROS 2
    rclpy.init()

    # Set up ROS 2 logs
    set_ros_loggers()

    # Create a finite state machine (FSM)
    sm = StateMachine(outcomes=["outcome4", "outcome3"])

    # Add states to the FSM
    sm.add_state(
        "INITIALIZE_DETECTIONS",
        CbState([SUCCEED], initializeDetections),
        transitions={
            SUCCEED: "outcome4",
        },
    )

    sm.add_state(
        "VOTE_DETECTIONS",
        VoteDetectionsState(),
        transitions={
            SUCCEED: "CHECK_DETECTIONS",
            CANCEL: "outcome4",
            ABORT: "outcome4",
        },
    )
    sm.add_state(
        "CHECK_DETECTIONS",
        CbState(["redetect", "found_candidate"], checkDetections),
        transitions={
            "redetect": "VOTE_DETECTIONS",
            "found_candidate": "outcome3"
        },
    )

    # Publish FSM information
    YasminViewerPub("YASMIN_ACTION_CLIENT_DEMO", sm)

    # Create an initial blackboard with the input value
    blackboard = Blackboard()
    blackboard["threshold"] = 5
    blackboard["beverage"] = "Person"

    # Execute the FSM
    try:
        outcome = sm(blackboard)
        yasmin.YASMIN_LOG_INFO(outcome)
    except KeyboardInterrupt:
        if sm.is_running():
            sm.cancel_state()  # Cancel the state if interrupted

    # Shutdown ROS 2
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == "__main__":
    main()