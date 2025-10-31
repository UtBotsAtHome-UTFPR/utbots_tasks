import rclpy
import yasmin
from yasmin import Blackboard, StateMachine
from yasmin_ros import set_ros_loggers
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from yasmin_viewer import YasminViewerPub

from utbots_tasks.states.basic_face import generate_new_face_sm, generate_recognition_sm, NewFaceState, RecognitionState, USBCamOff, USBCamOn
from utbots_tasks.states.basic_nav import generate_rotate_in_place, SetInitialPose
from utbots_tasks.states.basic_voice import CoquiTTSState, get_process_nlu, generate_ask_name_sm, whisper_process_cb
from utbots_tasks.states.basic_vision import FindObjectState
from utbots_tasks.states.logs import CrowdLogState

## UPGRADES TODO:
# - Show person face image feedback in the screen
# - Generate person point and navigate to the front of the operator

def cb_tts_log(blackboard: Blackboard): 
    people_count = blackboard["people_count"]
    blackboard["tts_text"] = f"The log has been saved successfully. Found {people_count} people in the crowd."
    return SUCCEED

def cb_save_operator_name(blackboard: Blackboard):
    blackboard['operator'] = blackboard['name']
    return SUCCEED

def cb_wait(blackboard: Blackboard):
    import time
    time.sleep(60)
    return SUCCEED

def main():
    yasmin.YASMIN_LOG_INFO("person_recognition_sm started")
    rclpy.init()
    node = rclpy.create_node("person_recognition_sm")

     # Set up ROS 2 logs
    set_ros_loggers()

    # Create a finite state machine (FSM)
    sm = StateMachine(outcomes=[SUCCEED, CANCEL, ABORT])
    yasmin.YASMIN_LOG_INFO("person_recognition_sm started")



    # sm.add_state(
    #     "SET_INIT_POSE",
    #     SetInitialPose(node, 0.0, 0.0, 0.0),
    #     transitions={
    #         SUCCEED: "TTS_INITIATING",
    #         ABORT: ABORT
    #     }
    # )

    sm.add_state(
        "TTS_INITIATING",
        CoquiTTSState(),
        transitions={
            SUCCEED: "TTS_COME_IN",
            CANCEL: ABORT,
        },
        remappings = {"tts_text" : "tts-initiate_task"}   
    )


    sm.add_state(
        "TTS_COME_IN",
        CoquiTTSState(),
        transitions={
            SUCCEED: "FIND_OPERATOR_ALONE",
            CANCEL: ABORT,
        },
        remappings = {"tts_text" : "tts-come_in"}   
    )

    sm.add_state(
        "FIND_OPERATOR_ALONE",
        FindObjectState(action_server="/YOLO_batch_detection"),
        transitions={
            SUCCEED: "TTS_GREET",
            CANCEL: "FIND_OPERATOR_ALONE",
            'not_detected': "FIND_OPERATOR_ALONE",
            ABORT: ABORT,
        },
        remappings={"objects": "person"}
    )

    sm.add_state(
        "TTS_GREET",
        CoquiTTSState(),
        transitions={
            SUCCEED: "ASK_NAME",
            CANCEL: ABORT,
        },
        remappings = {"tts_text" : "tts-greet"}   
    )

    sm.add_state(
        "ASK_NAME",
        generate_ask_name_sm(),
        transitions={
            SUCCEED: "SAVE_OPERATOR_NAME",
            CANCEL: ABORT,
        },
        remappings = {"tts_text" : "ask_name"}
    )

    sm.add_state(
        "SAVE_OPERATOR_NAME",
        yasmin.CbState([SUCCEED, CANCEL], cb_save_operator_name),
        transitions={
            SUCCEED: "TTS_CONFIRM_NAME",
            CANCEL: ABORT,
        },
    )

    sm.add_state(
        "TTS_CONFIRM_NAME",
        CoquiTTSState(),
        transitions={
            SUCCEED: "TTS_INSTRUCT_REGISTER_FACE",
            CANCEL: ABORT,
        },
        # tts_text is set by last state
    )

    sm.add_state(
        "TTS_INSTRUCT_REGISTER_FACE",
        CoquiTTSState(),
        transitions={
            SUCCEED: "NEW_FACE_SM",
            CANCEL: ABORT,
        },
        remappings = {"tts_text" : "tts-instruct_register_face"}
    )

    sm.add_state(
        "NEW_FACE_SM",
        NewFaceState(),
        transitions={
            SUCCEED: "TTS_PERSON_REGISTERED",
            CANCEL: ABORT,
        },
        remappings={"operator" : "name"}
    )

    sm.add_state(
        "TTS_PERSON_REGISTERED",
        CoquiTTSState(),
        transitions={
            SUCCEED: "30S_WAIT",
            CANCEL: ABORT,
        },
        remappings = {"tts_text" : "tts-person_registered"}
    )

    
    sm.add_state(
        "30S_WAIT",
        yasmin.CbState([SUCCEED],cb_wait),
        transitions={
            SUCCEED: "ROTATE_180_DEGREES",
        }
    )

    sm.add_state(
        "ROTATE_180_DEGREES",
        generate_rotate_in_place(node),
        transitions={
            SUCCEED: "FIND_PEOPLE",
            ABORT: ABORT
        },
    )

    sm.add_state(
        "FIND_PEOPLE",
        FindObjectState(action_server="/YOLO_batch_detection"),
        transitions={
            SUCCEED: "RECOGNITION_SM",
            CANCEL: CANCEL,
            'not_detected': "RECOGNITION_SM",
            ABORT: ABORT,
        },
        remappings={"objects": "person"}
    )

    sm.add_state(
        "RECOGNITION_SM",
        RecognitionState(),
        transitions={
            SUCCEED: "GENERATE_LOG",
            CANCEL: CANCEL,
        },
    )
    
    sm.add_state(
        "GENERATE_LOG",
        CrowdLogState(),
        transitions={
            SUCCEED: "WRITE_TTS_LOG",
            ABORT: ABORT,
        },
    )

    sm.add_state(
        "WRITE_TTS_LOG",
        yasmin.CbState([SUCCEED], cb_tts_log),
        transitions={
            SUCCEED: "TTS_LOG_SAVED",
        },
    )

    sm.add_state(
        "TTS_LOG_SAVED",
        CoquiTTSState(),
        transitions={
            SUCCEED: SUCCEED,
            CANCEL: ABORT,
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
    blackboard['n_pics'] = 3
    blackboard["people_count"] = 0

    # TTS blackboard variables for this task
    blackboard["tts-initiate_task"] = "Initiating person recognition task."
    # blackboard["tts_text"] = "Initiating person recognition task."
    blackboard["tts-come_in"] = "Hello,please come in and stand in front of me."
    blackboard["tts-greet"] = "Hello, I am Hestia."
    blackboard["ask_name"] = "What is your name.Please say my name is."
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