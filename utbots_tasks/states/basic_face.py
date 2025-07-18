import rclpy

import yasmin
from yasmin import CbState, Blackboard, StateMachine
from yasmin_ros import ActionState, ServiceState
from yasmin import State
from yasmin_ros import set_ros_loggers
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from yasmin_viewer import YasminViewerPub
from utbots_actions.action import NewFace, Recognition

from utbots_tasks.states.basic_nav import generate_rotate_in_place

from std_srvs.srv import SetBool

from math import pow, sqrt, sin, tan, radians, degrees

'''HOW TO USE, PLEASE READ

This file implements states for new_face_action, recognition_action and sub State Machines

In order to test run: ros2 launch utbots_face_recognition cam_recognition.launch.py and ros2 run utbots_tasks recognition

If you need to change between testing just the state or the hierarquical state machine comment one of the functions in the main function

'''

class SavePosition(State):
    def __init__(self) -> None:
        super().__init__([SUCCEED, CANCEL])

    def execute(self, blackboard: Blackboard) -> str:
        
        key = blackboard["people_yaw"][-1]

        # Lê quaternion aqui
        position = blackboard["current_pose"]

        for person in blackboard["person_list"]:
            
            if person["name"] == key:
                person["quat"] = position

        return SUCCEED

class IdentifyYAW(State):
    def __init__(self) -> None:
        super().__init__([SUCCEED, CANCEL])

    def execute(self, blackboard: Blackboard) -> str:
        if len(blackboard["people"]) == 0:
            print("No people for whatever reason")
            return CANCEL # No one in image
        
        # Read people direction list (if not exist create) if person does not have yaw set it up
        if "people_yaw" not in blackboard:
            print("create positions to state at during conversation")
            blackboard["people_yaw"] = ["Teste"]#["Teste"]

        people = blackboard["people"]

        width  = 1280 # CHANGE TO 1920
        theta_max = 78/2 # Logitech cam FOV 

        for person in people:
            print(person.category)
            if person.id == "Unknown" or person.id in blackboard["people_yaw"]:
                continue

            x = int((person.xmin + person.xmax) / 2)
            #y = int((person.ymin + person.ymax) / 2)
            #print(x)

            x = x - int(width/2.0)
            theta = radians(theta_max * x / (width/2))

            angle = degrees(theta)

            blackboard["people_yaw"].append(person.id)
            blackboard["rotate"] = angle
            blackboard["rotation_count"] = 0
            blackboard["people_count"] += 1
            return SUCCEED
            print(degrees(theta), end = "\n\n\n\n")
        blackboard["rotation_count"] += 45
        return CANCEL

class CheckContinuation(State):
    def __init__(self) -> None:
        super().__init__([SUCCEED, CANCEL])

    def execute(self, blackboard: Blackboard) -> str:
        if blackboard["people_count"] >= 2 or blackboard["rotation_count"] >= 360:
            return SUCCEED
        return CANCEL

class USBCamOn(ServiceState):
    def __init__(self) -> None:
        super().__init__(
            SetBool,  # srv type
            "/set_capture",  # service name
            self.create_request_handler,  # cb to create the request
            #["outcome1"],  # outcomes. Includes (SUCCEED, ABORT)
            response_handler=self.response_handler,  # cb to process the response
        )

    def create_request_handler(self, blackboard: Blackboard) -> SetBool.Request:
        req = SetBool.Request()
        req.data = True
        return req

    def response_handler(
        self, blackboard: Blackboard, response: SetBool.Response
    ) -> str:
        
        return SUCCEED

class USBCamOff(ServiceState):
    def __init__(self) -> None:
        super().__init__(
            SetBool,  # srv type
            "/set_capture",  # service name
            self.create_request_handler,  # cb to create the request
            #["outcome1"],  # outcomes. Includes (SUCCEED, ABORT)
            response_handler=self.response_handler,  # cb to process the response
        )

    def create_request_handler(self, blackboard: Blackboard) -> SetBool.Request:
        req = SetBool.Request()
        req.data = False
        return req

    def response_handler(
        self, blackboard: Blackboard, response: SetBool.Response
    ) -> str:
        
        return SUCCEED

class RecognitionState(ActionState):

    def __init__(self) -> None:
        super().__init__(
            Recognition,  # action type
            "/recognition",  # action name
            self.create_goal_handler,  # callback to create the goal
            None,  # outcomes. Includes (SUCCEED, ABORT, CANCEL)
            self.response_handler,  # callback to process the response
            None,  # callback to process the feedback
        )

    def create_goal_handler(self, blackboard: Blackboard) -> Recognition.Goal:
        goal = Recognition.Goal()

        if "img" in blackboard:
            goal.image = blackboard["img"]

        return goal

    def response_handler(self, blackboard: Blackboard, response: Recognition.Result) -> str:
        if type(response.people) is not list:
            return CANCEL

        blackboard["people"] = (response.people)
        blackboard["recognized_img"] = (response.image)
        print(blackboard["people"])
        # print(blackboard['recognized_img'])
        return SUCCEED
        '''self._node.get_logger().info("Here")
        if goal_status == 4:  # SUCCEEDED
            if "people" in blackboard:
                blackboard["people"] = (response.people)
                blackboard["recognized_img"] = (response.image)
            return SUCCEED
        elif goal_status == 5:  # CANCELED
            return CANCEL
        elif goal_status == 6:  # ABORTED
            return ABORT
        else:
            return ABORT  # fallback for unknown/error'''
        

class NewFaceState(ActionState):

    def __init__(self) -> None:
        super().__init__(
            NewFace,  # action type
            "/new_face",  # action name
            self.create_goal_handler,  # callback to create the goal
            None,  # outcomes. Includes (SUCCEED, ABORT, CANCEL)
            self.response_handler,  # callback to process the response
            None,  # callback to process the feedback
        )

    def create_goal_handler(self, blackboard: Blackboard) -> NewFace.Goal:
        goal = NewFace.Goal()

        if "n_pics" in blackboard:
            goal.n_pictures.data = blackboard["n_pics"]
        else:
            goal.n_pictures.data = 5
        
        if "operator" in blackboard:
            goal.name.data = str(blackboard["operator"])
        else:
            goal.name.data = "Operator"

        return goal

    def response_handler(self, blackboard: Blackboard, response: NewFace.Result) -> str:

        return SUCCEED

def generate_new_face_sm():
    new_face_sm = StateMachine(outcomes=[SUCCEED, CANCEL, ABORT])
    
    new_face_sm.add_state(
        "USBCAM_ON_STATE",
        USBCamOn(),
        transitions={
            SUCCEED: "NEW_FACE_STATE",
            ABORT: ABORT,
        },
    )

    new_face_sm.add_state(
        "NEW_FACE_STATE",
        NewFaceState(),
        transitions={
            SUCCEED: "USBCAM_OFF_STATE",
            CANCEL: ABORT,
            ABORT: ABORT
        },
    )

    new_face_sm.add_state(
        "USBCAM_OFF_STATE",
        USBCamOff(),
        transitions={
            SUCCEED: SUCCEED,
            ABORT: ABORT,
        },
    )

    return new_face_sm

def generate_recognition_sm():
    recognition_sm = StateMachine(outcomes=[SUCCEED, CANCEL, ABORT])
    
    recognition_sm.add_state(
        "USBCAM_ON_STATE",
        USBCamOn(),
        transitions={
            SUCCEED: "RECOGNITION_STATE",
            ABORT: ABORT,
        },
    )

    recognition_sm.add_state(
        "RECOGNITION_STATE",
        RecognitionState(),
        transitions={
            SUCCEED: "USBCAM_OFF_STATE",
            CANCEL: ABORT,
            ABORT: ABORT
        },
    )

    recognition_sm.add_state(
        "USBCAM_OFF_STATE",
        USBCamOff(),
        transitions={
            SUCCEED: SUCCEED,
            ABORT: ABORT,
        },
    )

    return recognition_sm

def state_test():
    yasmin.YASMIN_LOG_INFO("yasmin_action_client_demo")

    # Initialize ROS 2
    rclpy.init()

    # Set up ROS 2 logs
    set_ros_loggers()

    # Create a finite state machine (FSM)
    sm = StateMachine(outcomes=[SUCCEED, CANCEL])

    sm.add_state(
        "CALLING_NEW_FACE",
        NewFaceState(),
        transitions={
            SUCCEED: "CALLING_RECOGNITION", # All mapping to SUCCEED for now
            CANCEL: CANCEL,
            ABORT: SUCCEED,
        },
    )

    # Add states to the FSM
    sm.add_state(
        "CALLING_RECOGNITION",
        RecognitionState(),
        transitions={
            SUCCEED: "CAM_OFF_2", # All mapping to SUCCEED for now
            CANCEL: CANCEL,
            ABORT: SUCCEED,
        },
    )

    # Publish FSM information
    YasminViewerPub("YASMIN_ACTION_CLIENT_DEMO", sm)

    # Create an initial blackboard with the input value
    blackboard = Blackboard()
    #blackboard["n"] = 10  # Set the Fibonacci order to 10

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

def sm_test():
    yasmin.YASMIN_LOG_INFO("yasmin_sm_client_demo")

    # Initialize ROS 2
    rclpy.init()

    # Set up ROS 2 logs
    set_ros_loggers()

    # Create a finite state machine (FSM)
    sm = StateMachine(outcomes=[SUCCEED, CANCEL])

    sm.add_state(
        "NEW_FACE_SM",
        generate_new_face_sm(),
        transitions={
            SUCCEED: "RECOGNITION_SM",
            CANCEL: CANCEL,
        },
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
    YasminViewerPub("YASMIN_ACTION_CLIENT_DEMO", sm)

    # Create an initial blackboard with the input value
    blackboard = Blackboard()
    #blackboard["n"] = 10  # Set the Fibonacci order to 10

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

def yaw_test():
    yasmin.YASMIN_LOG_INFO("yasmin_sm_client_demo")

    # Initialize ROS 2
    rclpy.init()

    # Set up ROS 2 logs
    set_ros_loggers()

    node = rclpy.create_node("rotate_person")

    # Create a finite state machine (FSM)
    sm = StateMachine(outcomes=[SUCCEED, CANCEL])

    sm.add_state(
        "RECOGNITION_SM",
        generate_recognition_sm(),
        transitions={
            SUCCEED: "IDENTIFY_YAW",
            CANCEL: CANCEL,
        },
    )

    sm.add_state(
        "IDENTIFY_YAW",
        IdentifyYAW(),
        transitions={
            SUCCEED: "SAVE_POSITION", #"ROTATE_TO_PERSON",
            CANCEL: "ROTATE_45" #"ROTATE_45", # Girar 45º
        },
    )

    sm.add_state(
        "ROTATE_45",
        generate_rotate_in_place(node),
        transitions={
            SUCCEED: "CHECK_CONTINUATION",
            ABORT: CANCEL
        },
        remappings={
            "angle":"45_rotation"
        }
    )

    sm.add_state(
        "ROTATE_TO_PERSON",
        generate_rotate_in_place(node),
        transitions={
            SUCCEED: "SAVE_POSITION",
            ABORT: CANCEL
        },
    )

    sm.add_state(
        "SAVE_POSITION",
        SavePosition(),
        transitions={
            SUCCEED: "CHECK_CONTINUATION",
            CANCEL: CANCEL
        },
    )
    
    sm.add_state(
        "CHECK_CONTINUATION",
        CheckContinuation(),
        transitions={
            SUCCEED: "RECOGNITION_SM",
            CANCEL: CANCEL
        },
    )

    # Publish FSM information
    YasminViewerPub("YASMIN_ACTION_CLIENT_DEMO", sm)

    # Create an initial blackboard with the input value
    blackboard = Blackboard()

    blackboard["45_rotation"] = 45
    blackboard["people_count"] = 0
    blackboard["rotation_count"] = 0

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


def maina():
    #state_test()
    #sm_test()
    yaw_test()