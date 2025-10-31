import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
import yasmin
from yasmin import Blackboard, StateMachine, State
from yasmin_ros import ActionState, MonitorState, set_ros_loggers
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from yasmin_viewer import YasminViewerPub
from tf_transformations import quaternion_from_euler
import math

from utbots_actions.action import YOLOBatchDetection, MPPose
from utbots_msgs.msg import BoundingBox
from std_msgs.msg import String, Int32, Float32
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Point, PointStamped
from geometry_msgs.msg import Pose

from utbots_tasks.states.basic_face import USBCamOff, USBCamOn, RecognitionState
from utbots_tasks.states.basic_mediapipe import GetPersonPointState
from utbots_tasks.states.basic_nav import FollowPersonState, GoToState
from utbots_tasks.states.basic_vision import GetPersonPointState, GetPersonDistanceState, FramePersonState

from cv_bridge import CvBridge
import cv2
import time
import numpy as np

from math import pow, sqrt, sin, tan, radians, cos

class PersonToNavGoal(State):
    def __init__(self) -> None:
        super().__init__([SUCCEED, ABORT])
        self.bridge = CvBridge()

    def execute(self, blackboard: Blackboard) -> str:
        person_position = blackboard["person_position"]
        
        # If it can't calculate the persons position
        if person_position["x"] == person_position["y"] == person_position["distance"] == 0:
            pose = Pose()

            pose.position.x = 0.0
            pose.position.y = 0.0

            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0
            
            blackboard["pose"] = pose
            return SUCCEED

        theta_max = blackboard["camera_horizontal_fov"] / 2
        phi_max = blackboard["camera_vertical_fov"] / 2

        img_x_max = blackboard["camera_width"] / 2.0
        img_y_max = blackboard["camera_height"] / 2.0

        img_x = person_position["x"]
        img_y = person_position["y"]
        distance = person_position["distance"]

        # Caculate angle theta and phi
        theta = radians(theta_max * img_x / img_x_max)
        phi = radians(phi_max * img_y / img_y_max)

        # Calculate x, y and z
        z = distance * sin(phi)
        y = -distance * cos(phi) * sin(theta)
        x = distance * cos(phi) * cos(theta)

        dx = x
        dy = y # Ao usar tf o -y provavelmente pode ser transformado em y

        yaw = math.atan2(dy, dx)
        q = quaternion_from_euler(0.0, 0.0, yaw)

        print(f"Estimated distance is: {distance}")
        time.sleep(1)

        pose = Pose()

        pose.position.x = x
        pose.position.y = y # Ao usar tf o -y provavelmente pode ser transformado em y
        print(f"x is: {x}")
        print(f"y is: {y}")
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        blackboard["pose"] = pose

        return SUCCEED


def locate_person_from_face():
    yasmin.YASMIN_LOG_INFO("locate_person_from_face_demo")
    rclpy.init()
    
    node = rclpy.create_node("locate_person_from_face_sm")

     # Set up ROS 2 logs
    set_ros_loggers()

    sm = StateMachine(outcomes=[SUCCEED, CANCEL, ABORT])

    # sm.add_state(
    #     "RECOGNIZE",
    #     RecognitionState(),
    #     transitions={
    #         SUCCEED: "FIND_PEOPLE",
    #         ABORT: ABORT,
    #     }
    # )

    # sm.add_state(
    #     "FIND_PEOPLE",
    #     FindObjectState(action_server="/YOLO_batch_detection", verbose=True),
    #     transitions={
    #         SUCCEED: "FRAME_PERSON",
    #         'not_detected': ABORT,
    #         CANCEL: CANCEL,
    #         ABORT: ABORT
    #     }
    # )

    # sm.add_state(
    #     "FRAME_PERSON",
    #     FramePerson(),
    #     transitions={
    #         SUCCEED:"TRACK_PERSON_FROM_CROPPED",
    #         ABORT:ABORT
    #     }
    # )

    # sm.add_state(
    #     "TRACK_PERSON_FROM_CROPPED",
    #     GetPersonPointState(),
    #     transitions={
    #         SUCCEED:"TRACK_PERSON",
    #         ABORT:ABORT
    #     },
    #     remappings = {"mediapipe_img" : "cropped_person"}
    # )

    sm.add_state(
        "TRACK_PERSON",
        GetPersonPointState(),
        transitions={
            SUCCEED:"GET_PERSON_POSE_FROM_TORSO",
            ABORT:ABORT
        },
    )

    # Estado track person com a imagem não cropada pras partes que não terão bounding box
    sm.add_state(
        "GET_PERSON_POSE_FROM_TORSO",
        GetPersonDistanceState(),
        transitions={
            SUCCEED:"PERSON_TO_NAV_GOAL",
            ABORT:ABORT
        },
    )

    # Estado monitor de estimar a posição da pessoa a partir da posição da posição do torso dela
    # sm.add_state(
    #     "FRAME_PERSON",
    #     FramePersonState(),
    #     transitions={
    #         SUCCEED : "PERSON_TO_NAV_GOAL",
    #         ABORT : ABORT
    #     }
    # )

    sm.add_state(
        "PERSON_TO_NAV_GOAL",
        PersonToNavGoal(),
        transitions={
            SUCCEED:"NAV_FOLLOW",
            ABORT:ABORT
        },
    )

    sm.add_state(
        "NAV_FOLLOW",
        FollowPersonState(),
        transitions={
            SUCCEED:"TRACK_PERSON",
            ABORT:ABORT,
            CANCEL:ABORT
        }
    )

    blackboard = Blackboard()

    # Face recognition variables
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

    # Publish FSM information
    YasminViewerPub("YASMIN_ACTION_CLIENT_DEMO", sm)

    try:
        outcome = sm(blackboard)
        yasmin.YASMIN_LOG_INFO(outcome)
    except KeyboardInterrupt:
        if sm.is_running():
            sm.cancel_state()  # Cancel the state if interrupted

    # Shutdown ROS
    if rclpy.ok():
        rclpy.shutdown()
