import rclpy
import yasmin
from yasmin import Blackboard, StateMachine
from yasmin_ros import set_ros_loggers
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from yasmin_viewer import YasminViewerPub

from utbots_tasks.states.basic_face import generate_new_face_sm, generate_recognition_sm
from utbots_tasks.states.basic_nav import generate_rotate_in_place, SetInitialPose
from utbots_tasks.states.basic_voice import CoquiTTSState, get_process_nlu, generate_ask_name_sm
from utbots_tasks.states.basic_vision import FindObjectState

# PROCESS_NLU=get_process_nlu()

def main():
    yasmin.YASMIN_LOG_INFO("person_recognition_sm started")
    rclpy.init()
    node = rclpy.create_node("person_recognition_sm")

     # Set up ROS 2 logs
    set_ros_loggers()

    # Create a finite state machine (FSM)
    sm = StateMachine(outcomes=[SUCCEED, "success", "failed", CANCEL])

    sm.add_state(
        "SET_INIT_POSE",
        SetInitialPose(node, 0.0, 0.0, 0.0),
        transitions={
            SUCCEED: "TTS_INITIATING",
            ABORT: "failed"
        }
    )

    sm.add_state(
        "TTS_INITIATING",
        CoquiTTSState(),
        transitions={
            SUCCEED: "TTS_COME_IN",
            CANCEL: "failed",
        },
        remappings = {"tts_text" : "tts-initiate_task"}   
    )

    sm.add_state(
        "TTS_COME_IN",
        CoquiTTSState(),
        transitions={
            SUCCEED: "FIND_OPERATOR_ALONE",
            CANCEL: "failed",
        },
        remappings = {"tts_text" : "tts-come_in"}   
    )

    sm.add_state(
        "FIND_OPERATOR_ALONE",
        FindObjectState(remappings={"objects": "person"}, action_server="/YOLO_batch_detection"),
        transitions={
            SUCCEED: "TTS_GREET",
            CANCEL: "FIND_OPERATOR_ALONE",
            ABORT: "failed",
        }
    )

    sm.add_state(
        "TTS_GREET",
        CoquiTTSState(),
        transitions={
            SUCCEED: "ASK_NAME",
            CANCEL: "failed",
        },
        remappings = {"tts_text" : "tts-greet"}   
    )

    sm.add_state(
        "ASK_NAME",
        generate_ask_name_sm(),
        transitions={
            SUCCEED: "CONFIRM_NAME",
            CANCEL: "failed",
        },
        remappings = {"tts_text" : "tts-ask_name"}
    )

    sm.add_state(
        "TTS_CONFIRM_NAME",
        CoquiTTSState(),
        transitions={
            SUCCEED: "TTS_INSTRUCT_REGISTER_FACE",
            CANCEL: "failed",
        },
        # tts_text is set by last state
    )

    sm.add_state(
        "TTS_INSTRUCT_REGISTER_FACE",
        CoquiTTSState(),
        transitions={
            SUCCEED: "NEW_FACE_SM",
            CANCEL: "failed",
        },
        remappings = {"tts_text" : "tts-instruct_register_face"}
    )

    sm.add_state(
        "NEW_FACE_SM",
        generate_new_face_sm(),
        transitions={
            SUCCEED: "TTS_PERSON_REGISTERED",
            CANCEL: "failed",
        },           
    )

    sm.add_state(
        "TTS_PERSON_REGISTERED",
        CoquiTTSState(),
        transitions={
            SUCCEED: "ROTATE_180_DEGREES",
            CANCEL: "failed",
        },
        remappings = {"tts_text" : "person_registered"}
    )

    sm.add_state(
        "ROTATE_180_DEGREES",
        generate_rotate_in_place(node),
        transitions={
            SUCCEED: "FIND_PEOPLE",
            ABORT: "failed"
        },
    )

    sm.add_state(
        "FIND_PEOPLE",
        FindObjectState(remappings={"objects": "person"}, action_server="/YOLO_batch_detection"),
        transitions={
            SUCCEED: "RECOGNITION_SM",
            CANCEL: "RECOGNITION_SM",
            ABORT: "failed",
        }
    )

    sm.add_state(
        "RECOGNITION_SM",
        generate_recognition_sm(),
        transitions={
            SUCCEED: SUCCEED,
            CANCEL: CANCEL,
        },
    )

    # Publish FSM information
    YasminViewerPub("PERSONAL_RECOGNITION_SM", sm)

    # Create an initial blackboard with the input value
    blackboard = Blackboard()
    blackboard["iou_threshold"] = 0.5
    blackboard["support_threshold"] = 0.4
    blackboard["batch_size"] = 50
    blackboard["person"] = "person"
    blackboard["rotate"] = 180
    blackboard["people_count"] = 0

    # TTS blackboard variables for this task
    blackboard["tts-initiating_task"] = "Initiating person recognition task."
    blackboard["tts-come_in"] = "Hello,please come in and stand in front of me."
    blackboard["tts-greet"] = "Hello, I am Hestia."
    blackboard["tts-ask_name"] = "What is your name?"
    blackboard["tts-instruct_register_face"] = "Please stand still and face me while I register your face."
    blackboard["tts-person_registered"] = "Your face has been registered successfully. Please go to the crowd. I will turn in 30 seconds."
    
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