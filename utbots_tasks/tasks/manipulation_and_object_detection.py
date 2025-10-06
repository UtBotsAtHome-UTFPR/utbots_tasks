import rclpy
from rclpy.node import Node
import yasmin
from yasmin import Blackboard, StateMachine, State
from yasmin_ros import set_ros_loggers
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from yasmin_viewer import YasminViewerPub

from utbots_tasks.states.utils import CheckIterations
from utbots_tasks.states.basic_nav import GoToWaypointState
from utbots_tasks.states.basic_voice import CoquiTTSState
from utbots_tasks.states.basic_vision import FindObjectState, EstimateGraspPoint, SendGraspPointToPlanner
from utbots_tasks.states.logs import DetectionLogState

import os
home_directory = os.environ['HOME']

def main():
    yasmin.YASMIN_LOG_INFO("manipulation_and_object_detection_sm started")
    rclpy.init()

    # Create a temporary ROS 2 node just to handle parameters
    node = Node("manipulation_and_object_detection_node")

    # Declare a string parameter with a default value
    node.declare_parameter("map_name", "map")

    # Read the string parameter
    map_name = node.get_parameter("map_name").get_parameter_value().string_value

    # Set up ROS 2 logs
    set_ros_loggers()

    # Create a finite state machine (FSM)
    sm = StateMachine(outcomes=[SUCCEED, ABORT, CANCEL])

    # sm.add_state(
    #     "TTS_INITIATING_TASK",
    #     CoquiTTSState(),
    #     transitions={
    #         SUCCEED: "GO_TO_COLLECTION_LOCATION",
    #         CANCEL: CANCEL,
    #     },
    #     remappings={"tts_text": "tts-initiating_task"}
    # )

    # sm.add_state(
    #     "GO_TO_COLLECTION_LOCATION",
    #     GoToWaypointState(),
    #     transitions={
    #         SUCCEED: "FIND_OBJECTS",
    #         ABORT: ABORT
    #     },
    #     remappings={"waypoint_nametag": "wp-object_collection", "yaml_path": "yaml_path"}
    # )

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
            SUCCEED: "ESTIMATE_POINT",
            ABORT: ABORT
        },
    )
    
    # sm.add_state(
    #     "TTS_SAVING_LOG",
    #     CoquiTTSState(),
    #     transitions={
    #         SUCCEED: SUCCEED,
    #         CANCEL: CANCEL,
    #     },
    #     remappings={"tts_text": "tts-saving_log"}
    # )

    # IF NOT_DETECTED, REPEAT FIND OBJECTS FOR A NUMBER OF TIMES

    ### MANIPULATE STATES

    sm.add_state(
        "ESTIMATE_POINT",
        EstimateGraspPoint(),
        transitions={
            SUCCEED: "PICKUP_OBJECT",
            CANCEL: "ESTIMATE_POINT"
        }
    )

    sm.add_state(
        "PICKUP_OBJECT", 
        SendGraspPointToPlanner(node),
        transitions={
            SUCCEED:SUCCEED,
            CANCEL:CANCEL
        }
    )

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
    blackboard['yaml_path'] = f'{home_directory}/ros2_ws/src/utbots_navigation/utbots_nav/map/{map_name}_waypoints.yaml'
    blackboard['wp-object_collection'] = 'room'
    blackboard['iterations'] = 3  # Number of iterations for retrying object detection
    blackboard["fov_hor"] = 69.4
    blackboard["fov_ver"] = 42.5
    # blackboard['wp-object_collection'] = 'object_collection'

    # TTS blackboard variables for this task
    blackboard["tts-initiating_task"] = "Initiating manipulation and object recognition task."
    blackboard["tts-saving_log"] = "Objects detected. saving log."

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