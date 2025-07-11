from yasmin import State, Blackboard, StateMachine
from yasmin_ros import MonitorState, ActionState
from yasmin_ros.basic_outcomes import SUCCEED, CANCEL, ABORT
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import time
import yaml
import math
from tf_transformations import quaternion_multiply, quaternion_from_euler
from rclpy.node import Node
import rclpy
from math import sin, cos

custom_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    depth=10
)

class SetInitialPose(State):
    def __init__(self, node, x, y, yaw):
        super().__init__([SUCCEED, ABORT])
        self.x, self.y, self.yaw = x, y, yaw
        self.node = node

    def execute(self, blackboard):
        pub = self.node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg() # Set to current time
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation.z = sin(self.yaw/2)
        msg.pose.pose.orientation.w = cos(self.yaw/2)
        pub.publish(msg)
        return SUCCEED

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

class GoToState(ActionState):
    def __init__(self) -> None:
         super().__init__(
            NavigateToPose,  # action type
            "/navigate_to_pose",  # action name
            self.create_goal_handler,  # callback to create the goal
            None,  # outcomes
            None,  # callback to process the response
        )

    def create_goal_handler(self, blackboard: Blackboard) -> NavigateToPose.Goal:
        goal = NavigateToPose.Goal()
        goal.pose.pose = blackboard["pose"]
        goal.pose.header.frame_id = "map"  # Set the reference frame to 'map'
        return goal

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
    
class WaitDoorOpenState(MonitorState):
    def __init__(self) -> None:
        super().__init__(LaserScan, 
                         "/scan", 
                         [SUCCEED, ABORT, CANCEL], 
                         self.monitor_handler, 
                         msg_queue=10, 
                         timeout=30)

    def monitor_handler(self, blackboard: Blackboard, msg: LaserScan) -> str:
        try:
            ranges = msg.ranges
            size = len(ranges)
            sub_vec_a = ranges[0:31]
            sub_vec_b = ranges[size - 30:size - 1]
            check_vec = sub_vec_a + sub_vec_b

            if len([i for i in check_vec if i > 1.5]) > 45:
                blackboard["log"] = "Detected open door"
                print(blackboard['log'])
                time.sleep(5)
                return SUCCEED
            else:
                return CANCEL

        except Exception as e:
            blackboard["log"] = f"Error while waiting for door: {e}"
            print(blackboard['log'])
            return ABORT
        
def generate_rotate_in_place(node: Node) -> StateMachine:
    sm = StateMachine(outcomes=[SUCCEED, CANCEL, ABORT])
    sm.add_state(
        "GET_CURRENT_POSE",
        GetCurrentPoseState(),
        transitions={
            SUCCEED: "ROTATE",
            ABORT: ABORT
        },
    )
    sm.add_state(
        "ROTATE",
        RotateInPlaceState(node),
        transitions={
            SUCCEED: SUCCEED,
            CANCEL: CANCEL,
            ABORT: ABORT,
        },
    )
    return sm