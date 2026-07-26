import py_trees
import py_trees_ros
import py_trees_ros.action_clients
import math
import yaml
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler, quaternion_multiply

# From utbots_tasks/utbots_tasks/state/basic_nav.py/GoToState
class GoToState(py_trees_ros.action_clients.FromBlackboard):
    def __init__(self, name="Go To State"):
        super().__init__(
            name=name,
            action_type=NavigateToPose,
            action_name="/navigate_to_pose",
            key="/goal" 
        )
        
        # Record what is necessary for reading and writing. 
        self.blackboard.register_key(key="pose", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="/goal", access=py_trees.common.Access.WRITE)

    def initialise(self):
        nav2_goal = NavigateToPose.Goal()

        # Reads pose from memory
        if self.blackboard.exists("pose"):
            nav2_goal.pose.pose = self.blackboard.pose
            nav2_goal.pose.header.frame_id = "map"
            
        # Writes goal 
        self.blackboard.set("/goal", nav2_goal)
        
        # Reads "/goal" and sends it to the server
        super().initialise()

# From utbots_tasks/utbots_tasks/state/basic_nav.py/SetInitialPose
class SetInitialPose(py_trees.behaviour.Behaviour):
    def __init__(self, name: str = "SetInitialPose", x: float = 0.0, y: float = 0.0, yaw: float = 0.0):
        super().__init__(name=name)
        self.x = x
        self.y = y
        self.yaw = yaw
        
        self.node = None
        self.publisher = None

    def setup(self, **kwargs):
        """
        The ROS node is received via kwargs
        """
        self.node = kwargs.get('node')
        if self.node is None:
            raise RuntimeError("Nó do ROS não foi passado no setup da árvore!")
        
        # Creation of the publisher
        self.publisher = self.node.create_publisher(
            PoseWithCovarianceStamped, 
            '/initialpose', 
            10
        )

    def update(self) -> py_trees.common.Status:
        """
        Executed when the node receives a 'tick'
        """
        if self.publisher is None or self.node is None:
            self.logger.error("Publisher or node not initialized")
            return py_trees.common.Status.FAILURE

        msg = PoseWithCovarianceStamped()
        
        # Current node timestamp
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        msg.pose.pose.position.x = float(self.x)
        msg.pose.pose.position.y = float(self.y)
        
        msg.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        
        self.publisher.publish(msg)
        
        return py_trees.common.Status.SUCCESS

# From utbots_tasks/utbots_tasks/state/basic_nav.py/GetCurrentPoseState
class GetCurrentPose(py_trees.behaviour.Behaviour):
    def __init__(self, name: str = "GetCurrentPose", qos_profile=10):
        super().__init__(name=name)
        self.qos_profile = qos_profile
        
        # Register writing key on blackboard
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("current_pose", access=py_trees.common.Access.WRITE)
        
        self.msg_received = False
        self.node = None
        self.subscriber = None

    def setup(self, **kwargs):
        """
        Called once when the tree is initialized.
        The ROS node is passed via kwargs from tree.setup().
        """
        self.node = kwargs.get('node')
        
        if self.node is None:
            raise RuntimeError("ROS node was not passed to the tree setup")
            
        self.subscriber = self.node.create_subscription(
            Odometry,
            "/hoverboard_base_controller/odom",
            self.monitor_handler,
            self.qos_profile
        )

    def initialise(self):
        """
        Called whenever the behavior is visited after being in a FAILURE, SUCCESS, or inactive state. 
        Resets the flag to wait for a new message.
        """
        self.msg_received = False

    def monitor_handler(self, msg: Odometry):
        """Subscriber callback"""
        self.blackboard.current_pose = msg
        self.msg_received = True

    def update(self) -> py_trees.common.Status:
        """
        Called at every 'tick' of the tree
        """
        if self.msg_received:
            return py_trees.common.Status.SUCCESS
        else:
            # Continues executing (waiting) until the message arrives
            return py_trees.common.Status.RUNNING

# From utbots_tasks/utbots_tasks/state/basic_nav.py/RotateInPlaceState
class RotateInPlace(py_trees_ros.actions.ActionClient):
    def __init__(self, name: str = "RotateInPlace"):
        super().__init__(
            name=name,
            action_type=NavigateToPose,
            action_name="/navigate_to_pose",
            generate_goal_fn=self.create_goal_handler
        )
        
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("rotate", access=py_trees.common.Access.READ)
        self.blackboard.register_key("current_pose", access=py_trees.common.Access.READ)
        
    def create_goal_handler(self) -> NavigateToPose.Goal:
        try:
            degrees = self.blackboard.rotate
            current_pose = self.blackboard.current_pose
            
            radians = math.radians(degrees)
            pose = PoseStamped()
            
            pose.header.frame_id = "odom"
            
            # self.node is automatically bound to this behavior 
            # when the tree.setup() method is invoked
            pose.header.stamp = self.node.get_clock().now().to_msg()

            pose.pose.position.x = current_pose.pose.pose.position.x
            pose.pose.position.y = current_pose.pose.pose.position.y
            pose.pose.position.z = 0.0

            # Apply the rotation
            current_q = current_pose.pose.pose.orientation
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
            # Safety check (in case setup() has not yet occurred)
            if hasattr(self, 'node') and self.node is not None:
                self.node.get_logger().error(f"Could not compute goal: {e}")
                
            # py_trees.common.Status.FAILURE,
            return None

# From utbots_tasks/utbots_tasks/state/basic_nav.py/RotateInPlaceState
class GoToWaypointState(py_trees_ros.actions.ActionClient):
    def __init__(self, name: str = "GoToWaypoint"):
        super().__init__(
            name=name,
            action_type=NavigateToPose,
            action_name="/navigate_to_pose",
            generate_goal_fn=self.create_goal_handler
        )
        
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("waypoint_nametag", access=py_trees.common.Access.READ)
        self.blackboard.register_key("yaml_path", access=py_trees.common.Access.READ)

    def create_goal_handler(self) -> NavigateToPose.Goal:
        try:
            nametag = self.blackboard.waypoint_nametag
            yaml_path = self.blackboard.yaml_path
        except KeyError:
            return None

        if not nametag or not yaml_path:
            return None

        try:
            with open(yaml_path, 'r') as file:
                data = yaml.safe_load(file)

            waypoints = data.get('waypoints', {})
            if nametag in waypoints:
                pose_data = waypoints[nametag]
                
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = 'map'
                
                # Fills the timestamp with the current clock of the ROS node

                if hasattr(self, 'node') and self.node is not None:
                    pose_stamped.header.stamp = self.node.get_clock().now().to_msg()

                # Position
                pose_stamped.pose.position.x = float(pose_data['position']['x'])
                pose_stamped.pose.position.y = float(pose_data['position']['y'])
                pose_stamped.pose.position.z = float(pose_data['position']['z'])

                # Orientation
                pose_stamped.pose.orientation.x = float(pose_data['orientation']['x'])
                pose_stamped.pose.orientation.y = float(pose_data['orientation']['y'])
                pose_stamped.pose.orientation.z = float(pose_data['orientation']['z'])
                pose_stamped.pose.orientation.w = float(pose_data['orientation']['w'])

                goal = NavigateToPose.Goal()
                goal.pose = pose_stamped
                return goal

        except Exception as e:
            if hasattr(self, 'node') and self.node is not None:
                self.node.get_logger().error(f"Erro ao carregar o waypoint do YAML: {e}")
            return None

        return None