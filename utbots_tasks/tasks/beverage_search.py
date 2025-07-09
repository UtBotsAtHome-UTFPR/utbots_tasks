import rclpy
from rclpy.qos import qos_profile_sensor_data
from utbots_actions.action import YOLOBatchDetection
from std_msgs.msg import String, Int32, Float32
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import math
#Monkey path (TODO:change)
import numpy as np
if not hasattr(np, 'float'):
    np.float = float
from tf_transformations import quaternion_from_euler
from tf_transformations import quaternion_multiply
from std_msgs.msg import Int32

import yaml
import yasmin
from yasmin import CbState, Blackboard, StateMachine
from yasmin_ros import ActionState, MonitorState
from yasmin_ros import set_ros_loggers
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from yasmin_viewer import YasminViewerPub

from utbots_tasks.states.basic_face import RecognitionState, NewFaceState

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

custom_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    depth=10
)

# Fora da máquina, num node rclpy normal:
def cb(msg):
    print("Msg recebida:", msg)

# from states_lib.states.basic_nav import RotateInPlaceState

# class GetCurrentPoseState(MonitorState):
#     def __init__(self) -> None:
#         super().__init__(PoseWithCovarianceStamped, 
#                          "/amcl_pose", 
#                          [SUCCEED, ABORT], 
#                          self.monitor_handler, 
#                          qos=custom_qos, 
#                          msg_queue=10, 
#                          timeout=30)
        
#     def monitor_handler(self, blackboard: Blackboard, msg: PoseWithCovarianceStamped) -> str:
#         blackboard["current_pose"] = msg
#         return SUCCEED

class GetCurrentPoseState(MonitorState):
    def __init__(self) -> None:
        super().__init__(Odometry, 
                         "/hoverboard_base_controller/odom", 
                         [SUCCEED, ABORT], 
                         self.monitor_handler, 
                         qos=custom_qos, 
                         msg_queue=10, 
                         timeout=30)
        
    def monitor_handler(self, blackboard: Blackboard, msg: PoseWithCovarianceStamped) -> str:
        blackboard["current_pose"] = msg
        return SUCCEED

class GoToWaypointState(ActionState):
    def __init__(self) -> None:
        super().__init__(
            NavigateToPose,  # action type
            "/navigate_to_pose",  # action name
            self.create_goal_handler,  # callback to create the goal
            None,  # outcomes
            None,  # callback to process the response
        )

    def create_goal_handler(self, blackboard: Blackboard) -> NavigateToPose.Goal:
        nametag = blackboard["waypoint_nametag"]
        yaml_path = blackboard["yaml_path"]
        if not nametag or not yaml_path:
            yasmin.YASMIN_LOG_ERROR("Waypoint nametag or YAML path not provided in blackboard.")
            return ABORT

        with open(blackboard["yaml_path"], 'r') as file:
            data = yaml.safe_load(file)

        for waypoint in data['waypoints']:
            if waypoint == nametag:
                pose_data = data['waypoints'][waypoint]
                pose_stamped = PoseStamped()
                # pose_stamped.header.stamp.sec = pose_data['header']['stamp']['secs']
                # pose_stamped.header.stamp.nanosec = pose_data['header']['stamp']['nsecs']
                pose_stamped.header.frame_id = 'map'

                pose_stamped.pose.position.x = pose_data['position']['x']
                pose_stamped.pose.position.y = pose_data['position']['y']
                pose_stamped.pose.position.z = pose_data['position']['z']

                pose_stamped.pose.orientation.x = pose_data['orientation']['x']
                pose_stamped.pose.orientation.y = pose_data['orientation']['y']
                pose_stamped.pose.orientation.z = pose_data['orientation']['z']
                pose_stamped.pose.orientation.w = pose_data['orientation']['w']
                print(pose_stamped)

                goal = NavigateToPose.Goal()
                goal.pose = pose_stamped
                return goal
        return ABORT

class RotateInPlaceState(ActionState):
    def __init__(self, node) -> None:
        super().__init__(
            NavigateToPose,  # action type
            "/navigate_to_pose",  # action name
            self.create_goal_handler,  # callback to create the goal
            None,  # outcomes
            None,  # callback to process the response
        )
        self.node = node
        self.current_pose = PoseWithCovarianceStamped()
        
    def create_goal_handler(self, blackboard: Blackboard) -> NavigateToPose.Goal:
        degrees = blackboard["rotate"]
        radians = math.radians(degrees)

        try:
            pose = PoseStamped()
            self.current_pose = blackboard["current_pose"]
            pose.header.frame_id = "odom"
            pose.header.stamp = self.node.get_clock().now().to_msg()

            pose.pose.position.x = self.current_pose.pose.pose.position.x
            pose.pose.position.y = self.current_pose.pose.pose.position.y
            pose.pose.position.z = 0.0

            # Apply rotation
            current_q = self.current_pose.pose.pose.orientation
            q_current = [current_q.x, current_q.y, current_q.z, current_q.w]
            q_rotate = quaternion_from_euler(0, 0, radians)
            q_new = quaternion_multiply(q_rotate, q_current)

            pose.pose.orientation.x = q_new[0]
            pose.pose.orientation.y = q_new[1]
            pose.pose.orientation.z = q_new[2]
            pose.pose.orientation.w = q_new[3]

            goal = NavigateToPose.Goal()
            goal.pose = pose
            return goal

        except Exception as e:
            self.node.get_logger().error(f"Could not compute goal: {e}")
            return ABORT

class VoteDetectionsState(ActionState):
    def __init__(self) -> None:
        super().__init__(
            YOLOBatchDetection,  # action type
            "YOLO_batch_detection",  # action name
            self.create_goal_handler,  # callback to create the goal
            None,  # outcomes. Includes (SUCCEED, ABORT, CANCEL)
            self.response_handler,  # callback to process the response
            None,  # callback to process the feedback
        )

    def create_goal_handler(self, blackboard: Blackboard) -> YOLOBatchDetection.Goal:
        goal = YOLOBatchDetection.Goal()
        goal.target_category = String()
        goal.target_category.data = blackboard["beverage"]
        goal.batch_size = Int32()
        goal.batch_size.data = blackboard["batch_size"]
        goal.iou_threshold = Float32()
        goal.iou_threshold.data = blackboard["iou_threshold"]
        goal.support_threshold = Float32()
        goal.support_threshold.data = blackboard["support_threshold"]
        return goal

    def response_handler(self, blackboard: Blackboard, response: YOLOBatchDetection.Result) -> str:
        persons = response.detected_objs.bounding_boxes
        if len(persons) > 0:
            return SUCCEED
        else:
            return CANCEL

def main():
    yasmin.YASMIN_LOG_INFO("yasmin_action_client_demo")
    rclpy.init()
    node = rclpy.create_node("receptionist_sm")

     # Set up ROS 2 logs
    set_ros_loggers()

    # Create a finite state machine (FSM)
    sm = StateMachine(outcomes=["outcome4", "outcome3"])

    sm.add_state(
        "NEW_FACE",
        NewFaceState(),
        transitions={
            SUCCEED: "GO_TO_KITCHEN", # All mapping to SUCCEED for now
            CANCEL: "outcome3",
            ABORT: "outcome3",
        },
    )

    sm.add_state(
        "GO_TO_KITCHEN",
        GoToWaypointState(),
        transitions={
            SUCCEED: "GET_CURRENT_POSE",
            ABORT: "outcome3"
        },
    )

    sm.add_state(
        "GET_CURRENT_POSE",
        GetCurrentPoseState(),
        transitions={
            SUCCEED: "ROTATE",
            ABORT: "outcome3"
        },
    )
    sm.add_state(
        "ROTATE",
        RotateInPlaceState(node),
        transitions={
            SUCCEED: "VOTE_DETECTIONS",
            CANCEL: "outcome4",
            ABORT: "outcome4",
        },
    )
    sm.add_state(
        "VOTE_DETECTIONS",
        VoteDetectionsState(),
        transitions={
            SUCCEED: "RECOGNITION",
            CANCEL: "GET_CURRENT_POSE",
            ABORT: "outcome4",
        },
    )

    sm.add_state(
        "RECOGNITION",
        RecognitionState(),
        transitions={
            SUCCEED: "outcome4", # All mapping to SUCCEED for now
            CANCEL: "outcome3",
            ABORT: "outcome3",
        },
    )

    # Publish FSM information
    YasminViewerPub("YASMIN_ACTION_CLIENT_DEMO", sm)

    # Create an initial blackboard with the input value
    blackboard = Blackboard()
    blackboard["iou_threshold"] = 0.5
    blackboard["support_threshold"] = 0.4
    blackboard["batch_size"] = 50
    blackboard["beverage"] = "person"
    blackboard["rotate"] = 90
    blackboard['yaml_path'] = '/home/laser/ros2_ws/src/utbots_navigation/utbots_nav/map/pitaco_waypoints.yaml'
    blackboard['waypoint_nametag'] = 'living_room'

    # sub = node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', cb, qos_profile_sensor_data)
    # print("created sub")
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