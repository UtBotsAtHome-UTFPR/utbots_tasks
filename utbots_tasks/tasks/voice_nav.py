import rclpy
from utbots_actions.action import TextToSpeech, Transcription

import yasmin
from yasmin import CbState, Blackboard, StateMachine
from yasmin_ros import ActionState
from yasmin_ros import  ServiceState, ActionState
from yasmin_ros import set_ros_loggers
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from yasmin_viewer import YasminViewerPub

from utbots_actions.action import Transcription,TextToSpeech,InterpretNLU
from utbots_msgs.msg import BoundingBoxes
from std_msgs.msg import String

from utbots_tasks.states.basic_voice import WhisperSTTState,whisper_process_cb

from utbots_tasks.states.basic_voice import NLUInference,get_process_nlu,NLUProcess

from utbots_tasks.states.basic_voice import CoquiTTSState

from utbots_tasks.states.basic_voice import wait_cb

# from utbots_tasks.states.basic_voice import print_result as whisper_print_result

from utbots_tasks.tasks.beverage_search import GoToWaypointState

from rcl_interfaces.msg import ParameterDescriptor

from rclpy.node import Node

PROCESS_NLU=get_process_nlu()

def main():
    """
    Main function to execute the ROS 2 action client demo.

    This function initializes the ROS 2 client, sets up the finite state
    machine, adds the states, and starts the action processing.

    Parameters:
        None

    Returns:
        None

    Raises:
        KeyboardInterrupt: If the user interrupts the execution.
    """
    yasmin.YASMIN_LOG_INFO("yasmin_action_client_demo")

    # Initialize ROS 2
    rclpy.init()

    # Set up ROS 2 logs
    set_ros_loggers()


    # # Create a temporary node just for declaring/reading parameters
    # node = Node("yasmin_main_node")

    # # Declare parameter
    # node.declare_parameter(
    #     'verbose',
    #     False,
    #     ParameterDescriptor(
    #         description='Enable verbose logging for the RASA NLU interpreter. Default is False.') 
    # )

    # # Read the value
    # verbose = node.get_parameter('verbose').get_parameter_value().bool_value
    verbose = True  # Set verbose to False for less logging output
    # Create a finite state machine (FSM)
    sm = StateMachine(outcomes=["exit"])

    # Add states to the FSM

    sm.add_state(
        "WAIT_INITIALIZATION",
        CbState(["waited"],wait_cb),
        transitions={
            "waited": "WHISPER_PROCESS",
        },
    )

    sm.add_state(
        "CALLING_WHISPER",
        WhisperSTTState(),
        transitions={
            SUCCEED: "WHISPER_PROCESS",
            CANCEL: "exit",
            ABORT: "exit",
        },
    )
    sm.add_state(
        "WHISPER_PROCESS",
        CbState(["process_whisper1","process_whisper2","process_whisper3"],whisper_process_cb),
        transitions={
            "process_whisper1": "CALLING_WHISPER",
            "process_whisper2": "NLU_INFERENCE",
            # "process_whisper3": "outcome4",

        },
    )
    sm.add_state(
        "NLU_INFERENCE",
        NLUInference(),
        transitions={
            SUCCEED: "NLU_PROCESS",
            CANCEL: "exit",
            ABORT: "exit",
        },
        remappings={"nlu_input_text": "whispered"},
    )
# PROCESS_NLU=[ "greet",
#     "introduce_robot",
#     "affirm",
#     "deny",
#     "mood_great",
#     "mood_unhappy",
#     "follow",
#     "stop",
#     "go_to",
#     "say_operator_name",
#     "identify_operator",
#     "describe_ambient",
#     "default",]

    sm.add_state(
        "NLU_PROCESS",
        NLUProcess(verbose),  # Set verbose to True for detailed logging
        remappings={
            "nlu_input_text": "whispered",  # Input from the Whisper STT state
            # "nlu_input_text": "whispered",  # Input from the Whisper STT state
            },              
        transitions={
            PROCESS_NLU[0]: "TALK",
            PROCESS_NLU[1]: "TALK",
            PROCESS_NLU[2]: "TALK",
            PROCESS_NLU[3]: "TALK",
            PROCESS_NLU[4]: "TALK",
            PROCESS_NLU[5]: "TALK",
            PROCESS_NLU[6]: "TALK",
            PROCESS_NLU[7]: "TALK",
            PROCESS_NLU[8]: "NAV_TALK",
            PROCESS_NLU[9]: "TALK",
            PROCESS_NLU[10]: "TALK",
            PROCESS_NLU[11]: "TALK",
            PROCESS_NLU[12]: "TALK",
        },
    )


    sm.add_state(
        "NAV_TALK",
        CoquiTTSState(),
        transitions={
            SUCCEED: "NAV",
            CANCEL: "exit",
            ABORT: "exit",
        },
    )

    sm.add_state(
        "TALK",
        CoquiTTSState(),
        transitions={
            SUCCEED: "CALLING_WHISPER",
            CANCEL: "exit",
            ABORT: "exit",
        },
    )

    sm.add_state(
        "NAV",
        GoToWaypointState(),
        transitions={
            SUCCEED: "CALLING_WHISPER",
            CANCEL: "exit",
            ABORT: "exit",
        },
        remappings={
            "waypoint_nametag": "nlu_data",  # Input from the Whisper STT state
            # "nlu_input_text": "whispered",  # Input from the Whisper STT state
            },              
    )

    # sm.add_state(
    #     "GO_TO_KITCHEN",
    #     GoToWaypointState(),
    #     transitions={
    #         SUCCEED: "GET_CURRENT_POSE",
    #         ABORT: "outcome3"
    #     },
    # )



    # Publish FSM information
    YasminViewerPub("YASMIN_ACTION_CLIENT_DEMO", sm)
    # Create an initial blackboard with the input value

    # BLACKBOARD:
    blackboard = Blackboard()
    blackboard["tts_text"] = None
    blackboard["text"] = None
    blackboard["whispered"] = None
    blackboard["nlu_input_text"] = None
    blackboard["nlu_output"] = None # Store the result sequence in the blackboard

    blackboard["nlu_intent"] = None  # Store the result sequence in the blackboard

    blackboard["nlu_data"] = None # Store the result sequence in the blackboard

    blackboard["waypoint_nametag"] = None

    # blackboard["iou_threshold"] = 0.5
    # blackboard["support_threshold"] = 0.4
    # blackboard["batch_size"] = 50
    # blackboard["beverage"] = "person"
    # blackboard["rotate"] = 90
    # blackboard['yaml_path'] = '/home/laser/ros2_ws/src/utbots_navigation/utbots_nav/map/pitaco_waypoints.yaml'
    # blackboard['waypoint_nametag'] = 'living_room'


    import subprocess
    try:
        map_file = subprocess.check_output(
        ["ros2", "param", "get", "/map_server", "yaml_filename"],
        universal_newlines=True
        ).rsplit("String value is: ")[1]
        map_file = map_file.strip()  # Remove any leading/trailing whitespace
        print(f"Map file found: {map_file}")
        # Set the yaml_path in the blackboard
        blackboard['yaml_path'] = map_file.rsplit(".yaml")[0]+"_waypoints.yaml" #'/home/robo/david_ws/src/utbots_navigation/utbots_nav/map/pitaco_waypoints.yaml'
        print(f"Map file found: {blackboard['yaml_path']}")
    except:
        blackboard['yaml_path'] ='/home/ehg2004/utbots_ws/src/utbots_navigation/utbots_nav/map/pitaco_waypoints.yaml'
        print(f"Map file found: {blackboard['yaml_path']}")
    # blackboard['waypoint_nametag'] = 'kitchen'



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