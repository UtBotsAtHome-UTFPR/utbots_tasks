import rclpy
#Monkey path (TODO:change)
import numpy as np
if not hasattr(np, 'float'):
    np.float = float

import yasmin
from yasmin import CbState, Blackboard, StateMachine
from yasmin_ros import ActionState
from yasmin_ros import set_ros_loggers
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from yasmin_viewer import YasminViewerPub

from utbots_tasks.states.basic_face import RecognitionState, NewFaceState
from utbots_tasks.states.basic_nav import GetCurrentPoseState, RotateInPlaceState, GoToWaypointState, WaitDoorOpenState
from utbots_tasks.states.basic_vision import FindObjectState

def main():
    yasmin.YASMIN_LOG_INFO("yasmin_action_client_demo")
    rclpy.init()
    node = rclpy.create_node("receptionist_sm")

     # Set up ROS 2 logs
    set_ros_loggers()

    # Create a finite state machine (FSM)
    sm = StateMachine(outcomes=["success", "failed"])

    sm.add_state(
        "WAIT_DOOR",
        WaitDoorOpenState(),
        transitions={
            SUCCEED: "GO_TO_KITCHEN",
            "cancel": "WAIT_DOOR",
            ABORT: "failed"
        }
    )

    sm.add_state(
        "NEW_FACE",
        NewFaceState(),
        transitions={
            SUCCEED: "GO_TO_KITCHEN", # All mapping to SUCCEED for now
            CANCEL: "failed",
            ABORT: "failed",
        },
    )

    sm.add_state(
        "GO_TO_KITCHEN",
        GoToWaypointState(),
        transitions={
            SUCCEED: "GET_CURRENT_POSE",
            ABORT: "failed"
        },
    )

    sm.add_state(
        "GET_CURRENT_POSE",
        GetCurrentPoseState(),
        transitions={
            SUCCEED: "ROTATE",
            ABORT: "failed"
        },
    )
    sm.add_state(
        "ROTATE",
        RotateInPlaceState(node),
        transitions={
            SUCCEED: "FIND_PERSON",
            CANCEL: "failed",
            ABORT: "failed",
        },
    )
    sm.add_state(
        "FIND_PERSON",
        FindObjectState(),
        transitions={
            SUCCEED: "RECOGNITION",
            CANCEL: "GET_CURRENT_POSE",
            ABORT: "failed",
        },
    )

    sm.add_state(
        "RECOGNITION",
        RecognitionState(),
        transitions={
            SUCCEED: "success", # All mapping to SUCCEED for now
            CANCEL: "failed",
            ABORT: "failed",
        },
    )

    # Publish FSM information
    YasminViewerPub("YASMIN_ACTION_CLIENT_DEMO", sm)

    # Create an initial blackboard with the input value
    blackboard = Blackboard()
    blackboard["iou_threshold"] = 0.5
    blackboard["support_threshold"] = 0.4
    blackboard["batch_size"] = 50
    blackboard["beverage"] = "person"
    blackboard["rotate"] = 90
    blackboard['yaml_path'] = '/home/laser/ros2_ws/src/utbots_navigation/utbots_nav/map/pitaco_waypoints.yaml'
    blackboard['waypoint_nametag'] = 'living_room'

    try:
        outcome = sm(blackboard)
        yasmin.YASMIN_LOG_INFO(outcome)
    except KeyboardInterrupt:
        if sm.is_running():
            sm.cancel_state()  # Cancel the state if interrupted

    # Shutdown ROS
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == "__main__":
    main()