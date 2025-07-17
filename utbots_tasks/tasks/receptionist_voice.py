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

from utbots_tasks.states.basic_face import RecognitionState, NewFaceState, generate_new_face_sm, generate_recognition_sm

from utbots_tasks.states.basic_nav import GetCurrentPoseState, generate_rotate_in_place, GoToWaypointState, WaitDoorOpenState, SetInitialPose

from utbots_tasks.states.basic_vision import FindObjectState

from utbots_tasks.states.basic_voice import CoquiTTSState, get_process_nlu, generate_ask_name_sm, generate_ask_drink_sm
from utbots_tasks.states.basic_voice import Register,Person,ask_interested_in_sm #,greet_and_name_cb, new_face_error_cb



PROCESS_NLU=get_process_nlu()

personList= []

class PointToObjectState(State):
    def __init__(self) -> None:
        super().__init__([SUCCEED, CANCEL])

    def execute(self, blackboard: Blackboard) -> str:
        # yasmin.YASMIN_LOG_INFO("Executing state FOO")
        detections = blackboard["detections"]
        objects = blackboard["objects"]
        if isinstance(objects, str):
            object = objects
        elif isinstance(objects, list):
            object = objects[0]
        if len(detections) > 0:
            obj_bb = detections[0]
            x_cent = (obj_bb.xmaxn - obj_bb.xminn)/2 + obj_bb.xminn
            print(x_cent)
            left_divider = 1/3
            right_divider = 2/3
            if x_cent > left_divider:
                if x_cent > right_divider:
                    response = f"Your {object} is to my right."
                else:
                    response = f"Your {object} is right in front of me."
            else:
                response = f"Your {object} is to my left."
            yasmin.YASMIN_LOG_INFO(response)
            blackboard["tts_text"] = response
            return SUCCEED
        else:
            response = f"Your {object} is not here."
            yasmin.YASMIN_LOG_INFO(response)
            blackboard["tts_text"] = response
            return CANCEL

class CalculateIOUsState(State):
    def __init__(self) -> None:
        super().__init__([SUCCEED, CANCEL])

    def execute(self, blackboard: Blackboard) -> str:
        # yasmin.YASMIN_LOG_INFO("Executing state FOO")
        try:
            bboxes1 = blackboard["bboxes1"]
        except:
            bboxes1 = []
        try:
            bboxes2 = blackboard["bboxes2"]
        except:
            bboxes2 = []
        if len(bboxes1) > 0:
            if len(bboxes2) == 0:
                blackboard["object_bbox"] = [bboxes1[0]]
            else:
                min_iou = float('inf')
                best_pair = (None, None)

                for box1 in bboxes1:
                    for box2 in bboxes2:
                        iou = self.compute_iou(box1, box2)
                        print(iou)
                        if iou < min_iou:
                            min_iou = iou
                            best_pair = (box1, box2)
                if min_iou > blackboard["iou_threshold"]:
                    blackboard["tts_text"] = "Sorry, there is no seat available for you"
                    return CANCEL
                blackboard["object_bbox"] = [best_pair[0]]
            return SUCCEED
        else:
            blackboard["tts_text"] = "Sorry, there is no seat available for you"
            return CANCEL

    def compute_iou(self, box1, box2):
        # Unpack coordinates
        x1_min, y1_min, x1_max, y1_max = box1.xmin, box1.ymin, box1.xmax, box1.ymax
        x2_min, y2_min, x2_max, y2_max = box2.xmin, box2.ymin, box2.xmax, box2.ymax

        # Intersection rectangle
        inter_x_min = max(x1_min, x2_min)
        inter_y_min = max(y1_min, y2_min)
        inter_x_max = min(x1_max, x2_max)
        inter_y_max = min(y1_max, y2_max)

        print(inter_x_min, inter_x_max, inter_y_min, inter_y_max)

        # Compute intersection area
        inter_width = max(0, inter_x_max - inter_x_min)
        inter_height = max(0, inter_y_max - inter_y_min)
        inter_area = inter_width * inter_height

        # Areas of each box
        area1 = (x1_max - x1_min) * (y1_max - y1_min)
        area2 = (x2_max - x2_min) * (y2_max - y2_min)

        # Compute union area
        union_area = area1 + area2 - inter_area

        # Compute IoU
        if union_area == 0:
            return 0.0
        return inter_area / union_area

def main():
    yasmin.YASMIN_LOG_INFO("yasmin_action_client_demo")
    rclpy.init()
    node = rclpy.create_node("receptionist_sm")

     # Set up ROS 2 logs
    set_ros_loggers()

    # Create a finite state machine (FSM)
    sm = StateMachine(outcomes=["success", "failed"])

    single_guest_routine_sm = StateMachine(outcomes=["success", "failed"])
    # single_guest_routine_sm.add_state(
    #     "SET_INIT_POSE",
    #     SetInitialPose(node, 0.0, 0.0, 0.0),
    #     transitions={
    #         SUCCEED: "WAIT_DOOR",
    #         ABORT: "failed"
    #     }
    # )

    # single_guest_routine_sm.add_state(
    #     "WAIT_DOOR",
    #     WaitDoorOpenState(),
    #     transitions={
    #         SUCCEED: "COME_IN",
    #         CANCEL: "WAIT_DOOR",
    #         ABORT: "failed"
    #     }
    # )

    single_guest_routine_sm.add_state(
        "COME_IN",
        CoquiTTSState(),
        transitions={
            # SUCCEED: "FIND_OPERATOR_AT_DOOR",
            SUCCEED: "GREET",
            CANCEL: "failed",
        },
        remappings = {"tts_text" : "come_in"}   
    )

    # single_guest_routine_sm.add_state(
    #     "FIND_OPERATOR_AT_DOOR",
    #     FindObjectState(remappings={"objects": "person", "detections": "bboxes2"}),
    #     transitions={
    #         SUCCEED: "GREET",
    #         CANCEL: "FIND_OPERATOR_AT_DOOR",
    #         ABORT: "failed",
    #     }
    # )

    single_guest_routine_sm.add_state(
        "GREET",
        CoquiTTSState(),
        transitions={
            SUCCEED: "ASK_NAME",
            CANCEL: "failed",
        },
        remappings = {"tts_text" : "greet"}   
    )

    single_guest_routine_sm.add_state(
        "ASK_NAME",
        generate_ask_name_sm(),
        transitions={
            SUCCEED: "CONFIRM_NAME",
            CANCEL: "failed",
        },
        remappings = {"tts_text" : "ask_name"}
    )

    single_guest_routine_sm.add_state(
        "CONFIRM_NAME",
        CoquiTTSState(),
        transitions={
            # SUCCEED: "NEW_FACE_SM",
            # SUCCEED: "ASK_INTERESTED_IN",
            # SUCCEED: "REGISTER_PERSON",
            SUCCEED: "ASK_FOLLOW",
            CANCEL: "failed",
        },
    )

    # single_guest_routine_sm.add_state(
    #     "NEW_FACE_SM",
    #     generate_new_face_sm(),
    #     transitions={
    #         SUCCEED: "ASK_DRINK",
    #         CANCEL: "failed",
    #     },           
    # )

    # single_guest_routine_sm.add_state(
    #     "ASK_INTERESTED_IN",
    #     ask_interested_in_sm(),
    #     transitions={
    #         SUCCEED: "REGISTER_PERSON",
    #         CANCEL: "failed",
    #     },
    #     remappings = {"tts_text" : "ask_drink"}
    # )

    # single_guest_routine_sm.add_state(
    #     "REGISTER_PERSON",
    #     Register(),  # Set verbose to True for detailed logging
    #     transitions={
    #         SUCCEED: "ASK_FOLLOW",
    #         CANCEL: "failed",
    #     },
    # )

    single_guest_routine_sm.add_state(
        "ASK_FOLLOW",
        CoquiTTSState(),
        transitions={
            # SUCCEED: "GO_TO_BAR",
            SUCCEED: "ASK_DRINK",
            CANCEL: "failed",
        },
        remappings = {"tts_text" : "ask_follow"}
    )

# Find beverage in the beverage area

    # single_guest_routine_sm.add_state(
    #     "GO_TO_BAR",
    #     GoToWaypointState(),
    #     transitions={
    #         SUCCEED: "ASK_DRINK",
    #         ABORT: "failed"
    #     },
    #     remappings={"waypoint_nametag" : "room"}
    # )

    single_guest_routine_sm.add_state(
        "ASK_DRINK",
        generate_ask_drink_sm(),
        transitions={
            SUCCEED: "CONFIRM_DRINK",
            CANCEL: "failed",
        },
        remappings = {"tts_text" : "ask_drink"}
    )

    single_guest_routine_sm.add_state(
        "CONFIRM_DRINK",
        CoquiTTSState(),
        transitions={
            # SUCCEED: "FIND_BEVERAGE",
            SUCCEED: "DRINK_POSITION_TTS",
            CANCEL: "failed",
        },
    )

    # single_guest_routine_sm.add_state(
    #     "ROTATE_IN_BEVERAGE",
    #     generate_rotate_in_place(node),
    #     transitions={
    #         SUCCEED: "FIND_BEVERAGE",
    #         ABORT: "failed"
    #     },
    # )

    # single_guest_routine_sm.add_state(
    #     "FIND_BEVERAGE",
    #     FindObjectState(remappings={"objects": "drink"}),
    #     transitions={
    #         SUCCEED: "POINT_TO_BEVERAGE",
    #         CANCEL: "POINT_TO_BEVERAGE",
    #         ABORT: "failed"
    #     }
    # )

    # single_guest_routine_sm.add_state(
    #     "POINT_TO_BEVERAGE",
    #     PointToObjectState(),
    #     transitions={
    #         SUCCEED: "DRINK_POSITION_TTS",
    #         CANCEL: "DRINK_POSITION_TTS"
    #     },
    #     remappings = {"objects" : "drink"}
    # )

    single_guest_routine_sm.add_state(
        "DRINK_POSITION_TTS",
        CoquiTTSState(),
        transitions={
            SUCCEED: "ASK_INTERESTED_IN",
            CANCEL: "failed",
        }
    )

    single_guest_routine_sm.add_state(
        "ASK_INTERESTED_IN",
        ask_interested_in_sm(),
        transitions={
            SUCCEED: "REGISTER_PERSON",
            CANCEL: "failed",
        }
    )

    single_guest_routine_sm.add_state(
        "REGISTER_PERSON",
        Register(True),
        transitions={
            SUCCEED: "ASK_FOLLOW_LIVING_ROOM",
            CANCEL: "failed",
        }
    )



    single_guest_routine_sm.add_state(
        "ASK_FOLLOW_LIVING_ROOM",
        CoquiTTSState(),
        transitions={
            # SUCCEED: "GO_TO_LIVING_ROOM",
            SUCCEED: "GO_TO_LIVING_ROOM",
            CANCEL: "failed",
        },
        remappings = {"tts_text" : "ask_follow"}
    )

# Find seat in the living room

    single_guest_routine_sm.add_state(
        "GO_TO_LIVING_ROOM",
        GoToWaypointState(),
        transitions={
            SUCCEED: "ROTATE_IN_SEATING",
            ABORT: "failed"
        },
        remappings={"waypoint_nametag" : "living_room"}
    )

    single_guest_routine_sm.add_state(
        "ROTATE_IN_SEATING",
        generate_rotate_in_place(node),
        transitions={
            SUCCEED: "FIND_PEOPLE",
            ABORT: "failed"
        },
    )

    single_guest_routine_sm.add_state(
        "FIND_PEOPLE",
        FindObjectState(remappings={"objects": "person", "detections": "bboxes2"}),
        transitions={
            SUCCEED: "FIND_SEAT",
            CANCEL: "FIND_SEAT",
            ABORT: "failed",
        }
    )

    single_guest_routine_sm.add_state(
        "FIND_SEAT",
        FindObjectState(remappings={"objects": "seat", "detections": "bboxes1"}),
        transitions={
            SUCCEED: "FIND_BEST_SEAT",
            CANCEL: "ROTATE_IN_SEATING",
            ABORT: "failed"
        }
    )

    single_guest_routine_sm.add_state(
        "FIND_BEST_SEAT",
        CalculateIOUsState(),
        transitions={
            SUCCEED: "POINT_TO_SEAT",
            CANCEL: "TTS_STATE",
        },
        remappings = {"object_bbox" : "object_bbox"},
    )

    single_guest_routine_sm.add_state(
        "POINT_TO_SEAT",
        PointToObjectState(),
        transitions={
            SUCCEED: "TTS_STATE",
            CANCEL: "failed",
        },
        remappings = {"objects" : "seat", "detections" : "object_bbox"}
    )

    single_guest_routine_sm.add_state(
        "TTS_STATE",
        CoquiTTSState(),
        transitions={
            SUCCEED: "RECOGNITION_SM",
            CANCEL: "failed",
        },
    )

    single_guest_routine_sm.add_state(
        "RECOGNITION_SM",
        generate_recognition_sm(),
        transitions={
            SUCCEED: "success",
            CANCEL: "failed",
        },           
    )

    sm = StateMachine(outcomes=["success", "failed"])

    sm.add_state(
        "GUEST_ROUTINE",
        single_guest_routine_sm,
        transitions={
            "success":"GUEST_ROUTINE",
            "failed":"failed"
        }            
    )

    # Publish FSM information
    YasminViewerPub("YASMIN_ACTION_CLIENT_DEMO", sm)

    # Create an initial blackboard with the input value
    blackboard = Blackboard()
    blackboard["iou_threshold"] = 0.5
    blackboard["support_threshold"] = 0.4
    blackboard["batch_size"] = 50
    blackboard["beverage"] = "bottle"
    blackboard["seat"] = "chair"
    blackboard["person"] = "person"
    blackboard["rotate"] = 45
    blackboard['yaml_path'] = '/home/ehg2004/utbots_ws/src/utbots_navigation/utbots_nav/map/arena_filled.yaml'
    blackboard['waypoint_room'] = 'room'

    blackboard["bedroom"] = "bedroom"
    blackboard["kitchen"] = "kitchen"
    blackboard["living_room"] = "living_room"
    blackboard["room"] = "room"

    # TTS blackboard variables for this task
    blackboard["come_in"] = "Hello,please come in."
    blackboard["greet"] = "I am Hestia." 
    blackboard["ask_name"] = "What is your name?"
    blackboard["ask_drink"] = "What drink would you like."
    blackboard["ask_follow"] = "Please follow me."
    blackboard["tts_text"] = "come_in."
    blackboard["name"]= None
    blackboard["drink"]=None

    blackboard["person_list"]=[]
    blackboard["interested_in"]="robotics"

    blackboard["ask_interested_in"]="What are your interests?"

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