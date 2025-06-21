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

from utbots_tasks.states.basic_voice import WhisperTTSState

from utbots_tasks.states.basic_voice import print_result as whisper_print_result

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

    # Create a finite state machine (FSM)
    sm = StateMachine(outcomes=["outcome4"])

    # Add states to the FSM
    sm.add_state(
        "CALLING_WHISPER",
        WhisperTTSState(),
        transitions={
            SUCCEED: "PRINTING_RESULT",
            CANCEL: "outcome4",
            ABORT: "outcome4",
        },
    )
    sm.add_state(
        "PRINTING_RESULT",
        CbState([SUCCEED], whisper_print_result),
        transitions={
            SUCCEED: "CALLING_WHISPER",
            # SUCCEED: "outcome4",
        },
    )

    # Publish FSM information
    YasminViewerPub("YASMIN_ACTION_CLIENT_DEMO", sm)

    # Create an initial blackboard with the input value
    blackboard = Blackboard()
    blackboard["n"] = 10  # Set the Whisper order to 10

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