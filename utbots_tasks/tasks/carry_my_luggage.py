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



from utbots_tasks.states.basic_nav import GetCurrentPoseState, generate_rotate_in_place, GoToWaypointState, GoToState, WaitDoorOpenState, SetInitialPose

from utbots_tasks.states.basic_vision import FindObjectState

from utbots_tasks.states.basic_voice import CoquiTTSState, get_process_nlu, generate_ask_name_sm, generate_ask_drink_sm
from utbots_tasks.states.basic_voice import Register,Person,ask_interested_in_sm, NLUInference, NLUProcess, WhisperSTTState, whisper_process_cb, PROCESS_NLU #,greet_and_name_cb, new_face_error_cb
from utbots_tasks.states.basic_vision import GetPersonPositionState, FramePersonState
from utbots_tasks.states.basic_mediapipe import GetPersonPointState
from utbots_tasks.states.basic_nav import FollowPersonState

from utbots_tasks.states.basic_face import RecognitionState, NewFaceState, generate_new_face_sm, generate_recognition_sm, IdentifyYAW, SavePosition, CheckContinuation, get_people_position_sm

def generate_follow_person_sm():

    follow_person_state = StateMachine(outcomes=[SUCCEED, CANCEL, ABORT])
    
    follow_person_state.add_state(
        "TRACK_PERSON",
        GetPersonPointState(),
        transitions={
            SUCCEED:"GET_PERSON_POSE_FROM_TORSO",
            ABORT:ABORT
        },
    )

    # Estado track person com a imagem não cropada pras partes que não terão bounding box
    follow_person_state.add_state(
        "GET_PERSON_POSE_FROM_TORSO",
        GetPersonPositionState(),
        transitions={
            SUCCEED:"NAV_FOLLOW",
            ABORT:ABORT
        },
    )

    follow_person_state.add_state(
        "NAV_FOLLOW",
        FollowPersonState(),
        transitions={
            SUCCEED:"TRACK_PERSON",
            ABORT:ABORT,
            CANCEL:ABORT
        }
    )

    return follow_person_state

def generate_check_stop_sm():

    check_stop_sm = StateMachine(outcomes=[SUCCEED, CANCEL, ABORT])

    check_stop_sm.add_state(
        "CALLING_WHISPER2",
        WhisperSTTState(),
        transitions={
            SUCCEED: "WHISPER_PROCESS2",
            CANCEL: ABORT,
            ABORT: ABORT,
        },
    )

    check_stop_sm.add_state(
        "WHISPER_PROCESS2",
        CbState(["process_whisper1","process_whisper2","process_whisper3"],whisper_process_cb),
        transitions={
            "process_whisper1": "CALLING_WHISPER2",
            "process_whisper2": "NLU_INFERENCE2",
            # "process_whisper3": "outcome4",

        },
    )

    check_stop_sm.add_state(
        "NLU_INFERENCE2",
        NLUInference(),
        transitions={
            SUCCEED: "NLU_PROCESS2",
            CANCEL: ABORT,
            ABORT: ABORT,
        },
        remappings={"nlu_input_text": "whispered"},
    )

    check_stop_sm.add_state(
        "NLU_PROCESS2",
        NLUProcess(True),  # Set verbose to True for detailed logging     
        transitions={
            PROCESS_NLU[0]: "CALLING_WHISPER2",
            PROCESS_NLU[1]: "CALLING_WHISPER2",
            PROCESS_NLU[2]: "CALLING_WHISPER2",
            PROCESS_NLU[3]: SUCCEED,
            PROCESS_NLU[4]: "CALLING_WHISPER2",
            PROCESS_NLU[5]: "CALLING_WHISPER2",
            PROCESS_NLU[6]: "CALLING_WHISPER2",
            PROCESS_NLU[7]: "CALLING_WHISPER2",
            PROCESS_NLU[8]: "CALLING_WHISPER2",
            PROCESS_NLU[9]: "CALLING_WHISPER2",
            PROCESS_NLU[10]: "CALLING_WHISPER2",
            PROCESS_NLU[11]: "CALLING_WHISPER2",
            PROCESS_NLU[12]: "CALLING_WHISPER2",
            PROCESS_NLU[13]: "CALLING_WHISPER2",
            PROCESS_NLU[14]: "CALLING_WHISPER2",
            PROCESS_NLU[15]: "CALLING_WHISPER2",
            PROCESS_NLU[16]: "CALLING_WHISPER2",
            PROCESS_NLU[17]: "CALLING_WHISPER2",
            PROCESS_NLU[18]: "CALLING_WHISPER2",
            PROCESS_NLU[19]: "CALLING_WHISPER2",
        },
    )
    # Add state to check if person said stop
    # "greet",                # 0
    # "introduce_robot",      # 1
    # "affirm",               # 2
    # "deny",                 # 3
    # "mood_great",           # 4
    # "mood_unhappy",         # 5
    # "follow",               # 6
    # "stop",                 # 7
    # "go_to",                # 8
    # "say_operator_name",    # 9
    # "identify_operator",    # 10
    # "describe_ambient",     # 11
    # "like_drink",           # 12
    # "pick_object",          # 13
    # "ask_interested_in",    # 14
    # "default",              # 15
    # "ask_about_bahia",      # 16
    # "bahia_facts",          # 17  
    # "bahia_geography",      # 18
    # "bahia_climate",        # 19
    # "bahia_economy",        # 20

def cb_wait(blackboard: Blackboard):
    import time
    time.sleep(5)
    return SUCCEED

def main():
    yasmin.YASMIN_LOG_INFO("yasmin_action_client_demo")
    rclpy.init()
    node = rclpy.create_node("receptionist_sm")

     # Set up ROS 2 logs
    set_ros_loggers()

    blackboard = Blackboard()

    blackboard["person"] = "person"
    blackboard["check_operator_ready"] = "Are you ready? Can I take pictures of you.Please say yes."
    blackboard["tell_operator_to_move"] = "Please stand in front of me and look at the camera."

    blackboard["person_name"] = "Teste"
    blackboard['objects'] = ['person']

    # Yolo variables
    blackboard["batch_size"] = 10
    blackboard["iou_threshold"] = 0.5
    blackboard["support_threshold"] = 0.6

    blackboard["camera_width"] = 0
    blackboard["camera_height"] = 0

    blackboard["camera_horizontal_fov"] = 69.4
    blackboard["camera_vertical_fov"] = 42.5

    blackboard["follow_me_wp1"] = "follow_me_wp1"
    blackboard["follow_me_wp2"] = "follow_me_wp2"

    blackboard['yaml_path'] =f'{home_dir}/ros2_ws/src/utbots_navigation/utbots_nav/map/cbr2025v2_waypoints.yaml'

    # Create a finite state machine (FSM)
    #single_guest_routine_sm = StateMachine(outcomes=[SUCCEED, "success", "failed", CANCEL, ABORT])
    sm = StateMachine(outcomes=[SUCCEED, "success", "failed", CANCEL, ABORT])

    sm.add_state(
        "TTS_WILL_FOLLOW",
        CoquiTTSState(),
        transitions={
            SUCCEED : "5S_WAIT",
            ABORT : ABORT
        },
        remappings = {"tts_text" : "tell_operator_to_move"}
    )

    sm.add_state(
        "5S_WAIT",
        yasmin.CbState([SUCCEED],cb_wait),
        transitions={
            SUCCEED: "NEW_FACE",
        }
    )


    sm.add_state(
        "NEW_FACE",
        NewFaceState(),
        transitions={
            SUCCEED : "RECOGNIZE",
            CANCEL : ABORT,
            ABORT : ABORT
        }
    )

    sm.add_state(
        "RECOGNIZE",
        RecognitionState(),
        transitions={
            SUCCEED: "FIND_PEOPLE",
            CANCEL: ABORT,
        }
    )

    sm.add_state(
        "FIND_PEOPLE",
        FindObjectState(action_server="/YOLO_batch_detection", verbose=True),
        transitions={
            SUCCEED: "FRAME_PERSON",
            'not_detected': ABORT,
            CANCEL: CANCEL,
            ABORT: ABORT
        }
    )

    sm.add_state(
        "FRAME_PERSON",
        FramePersonState(),
        transitions={
            SUCCEED:"FOLLOW_PERSON",
            ABORT:ABORT
        }
    )

    sm.add_state(
        "FOLLOW_PERSON",
        generate_follow_person_sm(),
        transitions={
            SUCCEED:SUCCEED,# Ir para waypoint 2
            ABORT:ABORT
        },
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
