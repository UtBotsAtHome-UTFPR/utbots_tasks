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

from utbots_tasks.states.basic_face import RecognitionState, NewFaceState, generate_new_face_sm, generate_recognition_sm, IdentifyYAW, SavePosition, CheckContinuation, get_people_position_sm

from utbots_tasks.states.basic_nav import GetCurrentPoseState, generate_rotate_in_place, GoToWaypointState, GoToState, WaitDoorOpenState, SetInitialPose

from utbots_tasks.states.basic_vision import FindObjectState

from utbots_tasks.states.basic_voice import CoquiTTSState, get_process_nlu, generate_ask_name_sm, generate_ask_drink_sm
from utbots_tasks.states.basic_voice import Register,Person,ask_interested_in_sm #,greet_and_name_cb, new_face_error_cb

PROCESS_NLU=get_process_nlu()

def main():
    yasmin.YASMIN_LOG_INFO("yasmin_action_client_demo")
    rclpy.init()
    node = rclpy.create_node("receptionist_sm")

     # Set up ROS 2 logs
    set_ros_loggers()

    blackboard = Blackboard()
    blackboard["iou_threshold"] = 0.5
    blackboard["support_threshold"] = 0.4
    blackboard["batch_size"] = 50
    blackboard["beverage"] = "bottle"
    blackboard["seat"] = ["chair","sofa","couch"]
    blackboard["person"] = "person"
    blackboard["rotate"] = 45
    blackboard['yaml_path'] = '/home/laser/ros2_ws/src/utbots_navigation/utbots_nav/map/arena_filled_waypoints.yaml'
    blackboard['waypoint_room'] = 'room_bar_kitchen'
    blackboard["drink"] = 'drinks-milk'
    blackboard["detections"] = None

    # blackboard["bedroom"] = "bedroom"
    # blackboard["kitchen"] = "kitchen"
    blackboard["living_room"] = "receptionist_greet"
    blackboard["room"] = "receptionist_bar" #bedroom_to_table
    # blackboard["room"] = "room_bar_kitchen"
    # TTS blackboard variables for this task
    blackboard["come_in"] = "Hello,please come in."
    blackboard["greet"] = "I am Hestia." 
    blackboard["ask_name"] = "What is your name?"
    blackboard["ask_drink"] = "What drink would you like."
    blackboard["ask_follow"] = "Please follow me."
    blackboard["tts_text"] = "come_in."
    blackboard["tts-initiate_task"] = "Initiating person recognition task."
    blackboard["tts-instruct_register_face"] = "Please stand still and face me while I register your face."
    blackboard["name"]= None
    # blackboard["drink"]=None
    blackboard["all_objects"]=['drinks-coffee', 'drinks-coke', 'drinks-fanta', 'drinks-kuat', 'drinks-milk', 'drinks-orange_juice']

    blackboard["person_list"]=[]
    blackboard["interested_in"]="robotics"
    blackboard["ask_interested_in"]="What are your interests?"

    blackboard["people_count"] = 0
    blackboard["rotation_count"] = 0

    # Create a finite state machine (FSM)
    #single_guest_routine_sm = StateMachine(outcomes=[SUCCEED, "success", "failed", CANCEL, ABORT])
    sm = StateMachine(outcomes=[SUCCEED, "success", "failed", CANCEL, ABORT])

    # single_guest_routine_sm.add_state(
    #     "WAIT_DOOR",
    #     WaitDoorOpenState(),
    #     transitions={
    #         SUCCEED: "COME_IN",
    #         CANCEL: "WAIT_DOOR",
    #         ABORT: "failed"
    #     }
    # )
    
    sm.add_state(
        "GREET",
        CoquiTTSState(),
        transitions={
            SUCCEED: "ASK_NAME",
            CANCEL: "failed",
        },
        remappings = {"tts_text" : "greet"}   
    )

    sm.add_state(
        "ASK_NAME",
        generate_ask_name_sm(),
        transitions={
            SUCCEED: "CONFIRM_NAME",
            CANCEL: "failed",
        },
        remappings = {"tts_text" : "ask_name"}
    )

    sm.add_state(
        "CONFIRM_NAME",
        CoquiTTSState(),
        transitions={
            SUCCEED: "ASK_INTERESTED_IN",
            CANCEL: "failed",
        },
    )

    sm.add_state(
        "ASK_INTERESTED_IN",
        ask_interested_in_sm(),
        transitions={
            SUCCEED: SUCCEED,#"TTS_INSTRUCT_REGISTER_FACE",
            CANCEL: "failed",
        }
    )

    # sm.add_state(
    #     "TTS_INSTRUCT_REGISTER_FACE",
    #     CoquiTTSState(),
    #     transitions={
    #         SUCCEED: "NEW_FACE_SM",
    #         CANCEL: ABORT,
    #     },
    #     remappings = {"tts_text" : "tts-instruct_register_face"}
    # )


    # sm.add_state(
    #     "NEW_FACE_SM",
    #     generate_new_face_sm(),
    #     transitions={
    #         SUCCEED: "ASK_FOLLOW",
    #         CANCEL: "failed",
    #     },           
    # )

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