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

from utbots_tasks.states.basic_face import RecognitionState, NewFaceState

from utbots_tasks.states.basic_nav import GetCurrentPoseState, RotateInPlaceState, GoToWaypointState, WaitDoorOpenState, SetInitialPose

from utbots_tasks.states.basic_vision import FindObjectState

from utbots_tasks.states.basic_voice import CoquiTTSState, WhisperSTTState, whisper_process_cb, NLUInference, get_process_nlu, NLUProcess, generate_ask_name_sm, generate_ask_drink_sm, wait_cb#,greet_and_name_cb, new_face_error_cb

PROCESS_NLU=get_process_nlu()

class PointToObjectState(State):
    def __init__(self) -> None:
        super().__init__([SUCCEED, CANCEL])

    def execute(self, blackboard: Blackboard) -> str:
        # yasmin.YASMIN_LOG_INFO("Executing state FOO")
        detections = blackboard["detections"]
        object = blackboard["object"]
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
        bboxes1 = blackboard["bboxes1"]
        bboxes2 = blackboard["bboxes2"]
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

    # Create a finite state machine (FSM)zzz
    sm = StateMachine(outcomes=["success", "failed"])

    sm.add_state(
        "SET_INIT_POSE",
        SetInitialPose(node, 0.0, 0.0, 0.0),
        transitions={
            SUCCEED: "COME_IN",
            ABORT: "failed"
        }
    )

    # sm.add_state(
    #     "WAIT_DOOR",
    #     WaitDoorOpenState(),
    #     transitions={
    #         SUCCEED: "COME_IN",
    #         CANCEL: "WAIT_DOOR",
    #         ABORT: "failed"
    #     }
    # )

    sm.add_state(
        "COME_IN",
        CoquiTTSState(),
        transitions={
            SUCCEED: "GREET",
            CANCEL: "failed",
        },
        remappings = {"tts_text" : "come_in"}   
    )

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
        generate_ask_name_sm("ask_name"),
        transitions={
            SUCCEED: "ASK_DRINK",
            CANCEL: "failed",
        },
        remappings = {"tts_text" : "ask_name"}
    )


    sm.add_state(
        "NEW_FACE",
        NewFaceState(),
        transitions={
            SUCCEED: "ASK_DRINK", # All mapping to SUCCEED for now
            CANCEL: "failed",
            ABORT: "failed",
        },
        remappings={"operator" : "nlu_data"}
    )

    sm.add_state(
        "ASK_DRINK",
        generate_ask_drink_sm("ask_drink"),
        transitions={
            SUCCEED: "ASK_FOLLOW",
            CANCEL: "failed",
        },
        remappings = {"tts_text" : "ask_drink"}
    )

    sm.add_state(
        "ASK_FOLLOW",
        CoquiTTSState(),
        transitions={
            SUCCEED: "GO_TO_ROOM",
            CANCEL: "failed",
        },
        remappings = {"tts_text" : "ask_follow"}
    )

# Find beverage in the beverage area

    sm.add_state(
        "GO_TO_ROOM",
        GoToWaypointState(),
        transitions={
            SUCCEED: "GET_CURRENT_POSE2",
            ABORT: "failed"
        },
        remappings={"waypoint_nametag" : "room"}
    )

    sm.add_state(
        "GET_CURRENT_POSE2",
        GetCurrentPoseState(),
        transitions={
            SUCCEED: "ROTATE2",
            ABORT: "failed"
        },
    )
    sm.add_state(
        "ROTATE2",
        RotateInPlaceState(node),
        transitions={
            SUCCEED: "FIND_BEVERAGE",
            CANCEL: "failed",
            ABORT: "failed",
        },
    )

    sm.add_state(
        "FIND_BEVERAGE",
        FindObjectState(remappings={"object": "beverage"}),
        transitions={
            SUCCEED: "POINT_TO_BEVERAGE",
            CANCEL: "POINT_TO_BEVERAGE",
            ABORT: "failed"
        }
    )

    sm.add_state(
        "POINT_TO_BEVERAGE",
        PointToObjectState(),
        transitions={
            SUCCEED: "DRINK_POSITION_TTS",
            CANCEL: "DRINK_POSITION_TTS"
        },
        remappings = {"object" : "beverage"}
    )

    sm.add_state(
        "DRINK_POSITION_TTS",
        CoquiTTSState(),
        transitions={
            SUCCEED: "ASK_FOLLOW_LIVING_ROOM",
            CANCEL: "failed",
        }
    )

    sm.add_state(
        "ASK_FOLLOW_LIVING_ROOM",
        CoquiTTSState(),
        transitions={
            SUCCEED: "GO_TO_LIVING_ROOM",
            CANCEL: "failed",
        },
        remappings = {"tts_text" : "ask_follow"}
    )

# Find seat in the living room

    sm.add_state(
        "GO_TO_LIVING_ROOM",
        GoToWaypointState(),
        transitions={
            SUCCEED: "GET_CURRENT_POSE",
            ABORT: "failed"
        },
        remappings={"waypoint_nametag" : "living_room"}
    )

    sm.add_state(
        "GET_CURRENT_POSE",
        GetCurrentPoseState(),
        transitions={
            SUCCEED: "ROTATE",
            ABORT: "failed"
        },
    )
    sm.add_state(
        "ROTATE",
        RotateInPlaceState(node),
        transitions={
            SUCCEED: "FIND_SEAT",
            CANCEL: "failed",
            ABORT: "failed",
        },
    )

    sm.add_state(
        "FIND_PEOPLE",
        FindObjectState(remappings={"object": "person", "detections": "bboxes2"}),
        transitions={
            SUCCEED: "FIND_SEAT",
            CANCEL: "FIND_SEAT",
            ABORT: "failed",
        }
    )

    sm.add_state(
        "FIND_SEAT",
        FindObjectState(remappings={"object": "seat", "detections": "bboxes1"}),
        transitions={
            SUCCEED: "FIND_BEST_SEAT",
            CANCEL: "FIND_BEST_SEAT",
            ABORT: "failed"
        }
    )

    sm.add_state(
        "FIND_BEST_SEAT",
        CalculateIOUsState(),
        transitions={
            SUCCEED: "POINT_TO_SEAT",
            CANCEL: "TTS_STATE",
        },
        remappings = {"object_bbox" : "object_bbox"},
    )

    sm.add_state(
        "POINT_TO_SEAT",
        PointToObjectState(),
        transitions={
            SUCCEED: "TTS_STATE",
            CANCEL: "failed",
        },
        remappings = {"object" : "seat", "detections" : "object_bbox"}
    )

    sm.add_state(
        "TTS_STATE",
        CoquiTTSState(),
        transitions={
            SUCCEED: "success",
            CANCEL: "failed",
        },
    )

    sm.add_state(
        "RECOGNITION",
        RecognitionState(),
        transitions={
            SUCCEED: "success", # All mapping to SUCCEED for now
            CANCEL: "failed",
            ABORT: "failed",
        },
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
    blackboard["rotate"] = 90
    blackboard['yaml_path'] = '/home/laser/ros2_ws/src/utbots_navigation/utbots_nav/map/pitaco_waypoints.yaml'
    blackboard['waypoint_room'] = 'room'

    blackboard["bedroom"] = "bedroom"
    blackboard["kitchen"] = "kitchen"
    blackboard["living_room"] = "living_room"
    blackboard["room"] = "room"

    # TTS blackboard variables for this task
    blackboard["come_in"] = "Hello, please come in."
    blackboard["greet"] = "I am Hestia." 
    blackboard["ask_name"] = "What is your name."
    blackboard["ask_drink"] = "What drink would you like."
    blackboard["ask_follow"] = "Please follow me."
    blackboard["tts_text"] = "come_in."
    blackboard["name"]= None
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