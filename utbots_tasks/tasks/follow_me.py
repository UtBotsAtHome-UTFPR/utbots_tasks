# Robô inicia fora da arena, espera a porta abrir

# Vai para o primeiro waypoint

# Vai para waypoint 2

# Esperar por operador

# Aprender o operador

# Robot signals it is ready to start

# Seguir o operador até o operador dizer para parar

# Go back to waypoint 2

import rclpy
#Monkey path (TODO:change)
import numpy as np
if not hasattr(np, 'float'):
    np.float = float
import yasmin
from yasmin import CbState, Blackboard, StateMachine, State
from yasmin_ros import set_ros_loggers
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from yasmin_viewer import YasminViewerPub
import os
home_dir = os.path.expanduser("~")

from utbots_tasks.states.basic_face import RecognitionState, NewFaceState, generate_new_face_sm, generate_recognition_sm, IdentifyYAW, SavePosition, CheckContinuation, get_people_position_sm, USBCamOn

from utbots_tasks.states.basic_nav import GetCurrentPoseState, generate_rotate_in_place, GoToWaypointState, GoToState, WaitDoorOpenState, SetInitialPose

from utbots_tasks.states.basic_vision import FindObjectState

from utbots_tasks.states.basic_voice import CoquiTTSState, get_process_nlu, generate_ask_name_sm, generate_ask_drink_sm
from utbots_tasks.states.basic_voice import Register,Person,ask_interested_in_sm #,greet_and_name_cb, new_face_error_cb

def main():
    yasmin.YASMIN_LOG_INFO("yasmin_action_client_demo")
    rclpy.init()
    node = rclpy.create_node("receptionist_sm")

     # Set up ROS 2 logs
    set_ros_loggers()

    blackboard = Blackboard()

    blackboard["person"] = "person"
    blackboard["rotation_count"] = 0
    blackboard["tts-initiate_task"] = "I am ready to begin the task, please open the door."
    blackboard["check_operator_ready"] = "Are you ready? Can I take pictures of you?"
    blackboard["tell_operator_to_move"] = "I am ready, you can now go."

    # Create a finite state machine (FSM)
    #single_guest_routine_sm = StateMachine(outcomes=[SUCCEED, "success", "failed", CANCEL, ABORT])
    sm = StateMachine(outcomes=[SUCCEED, "success", "failed", CANCEL, ABORT])

    sm.add_state(
        "TTS_INITIATING",
        CoquiTTSState(),
        transitions={
            SUCCEED: "COME_IN",
            CANCEL: ABORT,
        },
        remappings = {"tts_text" : "tts-initiate_task"}   
    )

    sm.add_state(
        "WAIT_DOOR",
        WaitDoorOpenState(),
        transitions={
            SUCCEED: "GOTO_WAYPOINT1",
            CANCEL: "WAIT_DOOR",
            ABORT: "failed"
        }
    )

    sm.add_state(
        "GOTO_WAYPOINT1",
        GoToWaypointState(),
        transitions={
            SUCCEED: "",
            CANCEL: ABORT,
            ABORT: ABORT
        },
        remappings={
            "waypoint_nametag" : "placeholder1"
        }
    )

    sm.add_state(
        "GOTO_WAYPOINT2",
        GoToWaypointState(),
        transitions={
            SUCCEED: "",
            CANCEL: ABORT,
            ABORT: ABORT
        },
        remappings={
            "waypoint_nametag" : "placeholder2"
        }
    )


    sm.add_state(
        "WAIT_FOR_OPERATOR",
        FindObjectState(action_server="/yolo_node1/YOLO_batch_detection"),
        transitions={
            SUCCEED: "",
            CANCEL: "WAIT_FOR_OPERATOR",
            "not_detected" : "WAIT_FOR_OPERATOR",
            ABORT: "failed",
        },
        remappings={"objects": "person", "detections": "bboxes2"}
    )

    # Trocar isso por uma sub-máquina is operator ready com stt e nlu
    # Perguntar para o operador se ele está pronto para começar (pra garantir que a pessoa identificada foi o operador, se não responder ou der ruim volta pro find_operator)
    sm.add_state(
        "TTS_CHECK_OPERATOR_READY",
        CoquiTTSState(),
        transitions={
            SUCCEED: "",
            CANCEL: ABORT,
        },
        remappings = {"tts_text" : "check_operator_ready"}
    )

    sm.add_state(
        "NEW_FACE",
        NewFaceState(),
        transitions={
            SUCCEED : "",
            CANCEL : "",
            ABORT : ""
        }
    )

    sm.add_state(
        "TTS_READY TO FOLLOW",
        CoquiTTSState(),
        transitions={
            SUCCEED : "",
            ABORT : ""
        },
        remappings = {"tts_text" : "tell_operator_to_move"}
    )

    # Seguir o operador até o operador dizer para parar

    # Go back to waypoint 2

    # Publish FSM information
    YasminViewerPub("YASMIN_ACTION_CLIENT_DEMO", sm)

    # Create an initial blackboard with the input value
    

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