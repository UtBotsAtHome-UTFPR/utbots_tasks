from yasmin import State, Blackboard
from yasmin import ActionState, SUCCEED, ABORT
from nav2_msgs.action import NavigateToPose
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
import rclpy
import time
import yaml
import math
from tf_transformations import quaternion_from_euler
from std_msgs.msg import Int32

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
    def __init__(self) -> None:
        super().__init__(
            NavigateToPose,  # action type
            "/navigate_to_pose",  # action name
            self.create_goal_handler,  # callback to create the goal
            None,  # outcomes
            None,  # callback to process the response
        )
    
    def create_goal_handler(self, blackboard: Blackboard) -> NavigateToPose.Goal:
        degrees = blackboard["rotate"]
        radians = math.radians(degrees)

        # Create quaternion for yaw rotation
        q = quaternion_from_euler(0, 0, radians)

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = 0.0  # no translation
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        goal = NavigateToPose.Goal()
        goal.pose = pose
        return goal
    
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

        for pose_data in data['poses']:
            if pose_data['nametag'] == nametag:
                pose_stamped = PoseStamped()
                pose_stamped.header.stamp.sec = pose_data['header']['stamp']['secs']
                pose_stamped.header.stamp.nanosec = pose_data['header']['stamp']['nsecs']
                pose_stamped.header.frame_id = pose_data['header']['frame_id']

                pose_stamped.pose.position.x = pose_data['pose']['position']['x']
                pose_stamped.pose.position.y = pose_data['pose']['position']['y']
                pose_stamped.pose.position.z = pose_data['pose']['position']['z']

                pose_stamped.pose.orientation.x = pose_data['pose']['orientation']['x']
                pose_stamped.pose.orientation.y = pose_data['pose']['orientation']['y']
                pose_stamped.pose.orientation.z = pose_data['pose']['orientation']['z']
                pose_stamped.pose.orientation.w = pose_data['pose']['orientation']['w']

                return pose_stamped
        return ABORT

class WaitDoorOpenState(State):
    def __init__(self, node: Node, timeout_sec: int = 200):
        super().__init__(outcomes=['succeeded', 'timed_out'])
        self.node = node
        self.timeout_sec = timeout_sec
        self.scan = LaserScan()
    
    def on_entry(self):
        self._start_time = time.time()
        self._subscriber = self._node.create_subscription(
            Int32,
            self._topic_name,
            self._callback,
            10
        )

    def on_exit(self):
        self._node.destroy_subscription(self._subscriber)

    def _callback(self, msg: LaserScan):
        self.scan = msg

    def execute(self, blackboard: Blackboard) -> str:

        start_time = time.time()

        while (time.time() - start_time) < self.timeout_sec:
            try:
                if scan is None:
                    continue

                ranges = scan.ranges
                size = len(ranges)
                sub_vec_a = ranges[0:31]
                sub_vec_b = ranges[size - 30:size - 1]
                check_vec = sub_vec_a + sub_vec_b

                if len([i for i in check_vec if i > 1.5]) > 45:
                    blackboard["log"] = info("Detected open door")
                    time.sleep(5)
                    return SUCCEED

            except Exception as e:
                blackboard["log"] = f"Error while waiting for door: {e}"

        blackboard["log"] = "Timed out waiting for door to open"
        return ABORT