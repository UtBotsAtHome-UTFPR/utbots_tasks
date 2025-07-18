import rclpy
import yasmin
from yasmin import Blackboard, StateMachine, State
from yasmin_ros import set_ros_loggers
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from yasmin_viewer import YasminViewerPub

from utbots_tasks.states.utils import CheckIterations
from utbots_tasks.states.basic_nav import GoToWaypointState
from utbots_tasks.states.basic_voice import CoquiTTSState
from utbots_tasks.states.basic_vision import FindObjectState
from utbots_tasks.states.logs import DetectionLogState

def main():
    yasmin.YASMIN_LOG_INFO("manipulation_and_object_detection_sm started")
    rclpy.init()

    # Set up ROS 2 logs
    set_ros_loggers()

    # Create a finite state machine (FSM)
    sm = StateMachine(outcomes=[SUCCEED, ABORT, CANCEL])

    sm.add_state(
        "TTS_INITIATING_TASK",
        CoquiTTSState(),
        transitions={
            SUCCEED: "FIND_OBJECTS",
            CANCEL: CANCEL,
        },
        remappings={"tts_text": "tts-initiating_task"}
    )

    sm.add_state(
        "GO_TO_COLLECTION_LOCATION",
        GoToWaypointState(),
        transitions={
            SUCCEED: "TTS_INITIATING_TASK",
            ABORT: ABORT
        },
        remappings={"waypoint_nametag": "wp-object_collection", "yaml_path": "yaml_path"}
    )

    sm.add_state(
        "FIND_OBJECTS",
        FindObjectState(action_server="/YOLO_batch_detection"),
        transitions={
            SUCCEED: "RETRY_OR_NOT",
            'not_detected': "FIND_OBJECTS",
            CANCEL: CANCEL,
            ABORT: ABORT
        },
    )

    sm.add_state(
        "RETRY_OR_NOT",
        CheckIterations(),
        transitions={
            "continue": "DETECTION_LOG",
            "repeat": "FIND_OBJECTS"
        },
    )

    sm.add_state(
        "DETECTION_LOG",
        DetectionLogState(),
        transitions={
            SUCCEED: SUCCEED,
            ABORT: ABORT
        },
    )

    # IF NOT_DETECTED, REPEAT FIND OBJECTS FOR A NUMBER OF TIMES

    ### MANIPULATE STATES

    # sm.add_state(
    #     "GO_TO_DELIVERY_LOCATION",
    #     GoToWaypointState(),
    #     transitions={
    #         SUCCEED: "TTS_INITIATING_TASK",
    #         ABORT: "failed"
    #     },
    #     remappings={"waypoint_nametag": "waypoint_room", "yaml_path": "yaml_path"}
    # )

    ### REPEAT FOR NUMBER OF OBJECTS

    # Publish FSM information
    YasminViewerPub("MANIPULATION_AND_OBJECT_RECOGNITION_SM", sm)

    # Create an initial blackboard with the input value
    blackboard = Blackboard()
    blackboard["iou_threshold"] = 0.5
    blackboard["support_threshold"] = 0.4
    blackboard["batch_size"] = 50
    blackboard["objects"] = []
    blackboard['yaml_path'] = '/home/ehg2004/utbots_ws/src/utbots_navigation/utbots_nav/map/arena_filled_waypoints.yaml'
    blackboard['wp-object_collection'] = 'receptionist_bar'
    blackboard['iterations'] = 3  # Number of iterations for retrying object detection
    # blackboard['wp-object_collection'] = 'object_collection'

    # TTS blackboard variables for this task
    blackboard["tts-initiating_task"] = "Initiating manipulation and object recognition task."

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