import py_trees
import py_trees_ros
import py_trees_ros.action_clients
import math
import yaml
import time
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler, quaternion_multiply
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

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
            raise RuntimeError("Node was not passed to the tree setup.")
        
        # Creation of the publisher
        self.publisher = self.node.create_publisher(
            PoseWithCovarianceStamped, 
            '/initialpose', 
            10
        )

    def update(self) -> py_trees.common.Status:

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
class GetCurrentPoseState(py_trees.behaviour.Behaviour):
    def __init__(self, name: str = "GetCurrentPoseState", qos_profile=None, timeout_sec: float = 30.0):
        super().__init__(name=name)
        
        # Qos SensorData if not informed
        if qos_profile is None:
            self.qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )
        else:
            self.qos_profile = qos_profile

        self.timeout_sec = timeout_sec
        self.start_time = None
        
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("current_pose", access=py_trees.common.Access.WRITE)
        
        self.msg_received = False
        self.node = None
        self.subscriber = None

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        if self.node is None:
            raise RuntimeError("The node was not passed in the tree setup.")
            
        self.subscriber = self.node.create_subscription(
            Odometry,
            "/hoverboard_base_controller/odom",
            self.monitor_handler,
            self.qos_profile
        )

    def initialise(self):
        """Resets the states and records the time the search began."""
        self.msg_received = False
        self.start_time = time.time()

    def monitor_handler(self, msg: Odometry):
        """Callback invoked when the odometry message arrives"""
        self.blackboard.current_pose = msg
        self.msg_received = True

    def update(self) -> py_trees.common.Status:
        # If mesage received = success
        if self.msg_received:
            return py_trees.common.Status.SUCCESS

        # Verifies timeout (abort)
        if (time.time() - self.start_time) > self.timeout_sec:
            self.node.get_logger().error(
                f"[{self.name}] {self.timeout_sec}s timeout"
            )
            return py_trees.common.Status.FAILURE

        # Waiting mesage
        return py_trees.common.Status.RUNNING


# From utbots_tasks/utbots_tasks/state/basic_nav.py/GoToState
class GoToState(py_trees_ros.action_clients.FromBlackboard):
    def __init__(self, name="Go To State"):
        super().__init__(
            name=name,
            action_type=NavigateToPose,
            action_name="/navigate_to_pose",
            key="goal"  
        )
        
        # Logs access to the necessary keys in Blackboard
        self.blackboard.register_key(key="pose", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="goal", access=py_trees.common.Access.WRITE)

    def initialise(self):
        # Verifies if pose exists
        if not self.blackboard.exists("pose"):
            self.node.get_logger().error(
                f"[{self.name}] Key 'pose' was not found."
            )
            return

        # Builds navigation goal
        nav2_goal = NavigateToPose.Goal()
        nav2_goal.pose.pose = self.blackboard.pose
        nav2_goal.pose.header.frame_id = "map"
        
        # Adds the current ROS 2 node timestamp
        if hasattr(self, 'node') and self.node is not None:
            nav2_goal.pose.header.stamp = self.node.get_clock().now().to_msg()

        # Saves it to the Blackboard and lets the parent class send it to the Action Server
        self.blackboard.set("goal", nav2_goal)
        
        super().initialise()


# From utbots_tasks/utbots_tasks/state/basic_nav.py/RotateInPlaceState
# Native math functions
def euler_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return [x, y, z, w]

def multiply_quaternions(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2

    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return [x, y, z, w]

class RotateInPlaceState(py_trees_ros.action_clients.FromBlackboard):
    def __init__(self, name: str = "RotateInPlace"):
        super().__init__(
            name=name,
            action_type=NavigateToPose,
            action_name="/navigate_to_pose",
            key="goal"
        )
        
        self.blackboard.register_key("rotate", access=py_trees.common.Access.READ)
        self.blackboard.register_key("current_pose", access=py_trees.common.Access.READ)
        self.blackboard.register_key("goal", access=py_trees.common.Access.WRITE)
        
    def initialise(self):
        # Security check before accessing data
        if not self.blackboard.exists("rotate") or not self.blackboard.exists("current_pose"):
            if hasattr(self, 'node') and self.node is not None:
                self.node.get_logger().error(f"[{self.name}] 'rotate' or 'current_pose' keys not found.")
            return 

        try:
            degrees = self.blackboard.rotate
            current_pose = self.blackboard.current_pose
            
            radians = math.radians(degrees)
            pose = PoseStamped()
            pose.header.frame_id = "odom"
            
            if hasattr(self, 'node') and self.node is not None:
                pose.header.stamp = self.node.get_clock().now().to_msg()

            pose.pose.position.x = current_pose.pose.pose.position.x
            pose.pose.position.y = current_pose.pose.pose.position.y
            pose.pose.position.z = 0.0

            # current guidance
            current_q = current_pose.pose.pose.orientation
            q_current = [current_q.x, current_q.y, current_q.z, current_q.w]
            
            # Applies rotation
            q_rotate = euler_to_quaternion(0, 0, radians)
            q_new = multiply_quaternions(q_rotate, q_current)

            pose.pose.orientation.x = q_new[0]
            pose.pose.orientation.y = q_new[1]
            pose.pose.orientation.z = q_new[2]
            pose.pose.orientation.w = q_new[3]

            goal = NavigateToPose.Goal()
            goal.pose = pose
            
            self.blackboard.set("goal", goal)

        except Exception as e:
            if hasattr(self, 'node') and self.node is not None:
                self.node.get_logger().error(f"Error calculating rotation: {e}")
            return # Stops execution in the event of a mathematical error
            
        # If everything succeeded
        super().initialise()


#  From utbots_tasks/utbots_tasks/state/basic_nav.py/GoToWaypointState
class GoToWaypointState(py_trees_ros.action_clients.FromBlackboard):
    def __init__(self, name: str = "GoToWaypoint"):
        super().__init__(
            name=name,
            action_type=NavigateToPose,
            action_name="/navigate_to_pose",
            key="goal"
        )
        
        self.blackboard.register_key("waypoint_nametag", access=py_trees.common.Access.READ)
        self.blackboard.register_key("yaml_path", access=py_trees.common.Access.READ)
        self.blackboard.register_key("goal", access=py_trees.common.Access.WRITE)

    def initialise(self):
    
        if self.blackboard.exists("goal"):
            self.blackboard.unset("goal")

        if not self.blackboard.exists("waypoint_nametag") or not self.blackboard.exists("yaml_path"):
            if hasattr(self, 'node') and self.node is not None:
                self.node.get_logger().error(f"[{self.name}] 'waypoint_nametag' or 'yaml_path' not in Blackboard.")
            return

        nametag = self.blackboard.waypoint_nametag
        yaml_path = self.blackboard.yaml_path

        if not nametag or not yaml_path:
            return

        try:
            with open(yaml_path, 'r') as file:
                data = yaml.safe_load(file)

            waypoints = data.get('waypoints', {})
            
            # Verifies if waypoint exists in yaml
            if nametag not in waypoints:
                if hasattr(self, 'node') and self.node is not None:
                    self.node.get_logger().error(f"[{self.name}] Waypoint '{nametag}' not found in YAML.")
                return

            # Constructs objetive 
            pose_data = waypoints[nametag]
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = 'map'
            
            if hasattr(self, 'node') and self.node is not None:
                pose_stamped.header.stamp = self.node.get_clock().now().to_msg()

            pose_stamped.pose.position.x = float(pose_data['position']['x'])
            pose_stamped.pose.position.y = float(pose_data['position']['y'])
            pose_stamped.pose.position.z = float(pose_data['position']['z'])

            pose_stamped.pose.orientation.x = float(pose_data['orientation']['x'])
            pose_stamped.pose.orientation.y = float(pose_data['orientation']['y'])
            pose_stamped.pose.orientation.z = float(pose_data['orientation']['z'])
            pose_stamped.pose.orientation.w = float(pose_data['orientation']['w'])

            goal = NavigateToPose.Goal()
            goal.pose = pose_stamped
            
            self.blackboard.set("goal", goal)

        except Exception as e:
            if hasattr(self, 'node') and self.node is not None:
                self.node.get_logger().error(f"Error loading waypoint from YAML: {e}")
            return

        super().initialise()


class WaitDoorOpenState(py_trees.behaviour.Behaviour):
    def __init__(self, name: str = "WaitDoorOpenState", timeout_sec: float = 30.0):
        super().__init__(name=name)
        
        self.timeout_sec = timeout_sec
        
        # Variáveis de controle de tempo
        self.start_time = None
        self.door_open_time = None
        
        # Variáveis de status
        self.door_detected = False
        self.error_occurred = False

        # Configuração do Blackboard
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("log", access=py_trees.common.Access.WRITE)
                
        self.node = None
        self.subscriber = None

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        if self.node is None:
            raise RuntimeError("O nó do ROS não foi passado no setup da árvore.")
            
        # qos_profile_sensor_data é o mais indicado para tópicos de alta frequência como o /scan
        self.subscriber = self.node.create_subscription(
            LaserScan,
            "/scan",
            self.monitor_handler,
            qos_profile_sensor_data
        )

    def initialise(self):
        """Reseta os estados sempre que o nó é iniciado na árvore."""
        self.start_time = time.time()
        self.door_open_time = None
        self.door_detected = False
        self.error_occurred = False
        
        if self.node:
            self.node.get_logger().info(f"[{self.name}] Aguardando a porta abrir...")

    def monitor_handler(self, msg: LaserScan):
        """Callback do ROS: Apenas lê os dados e atualiza as flags internas."""
        # Se já detectou ou deu erro, ignora as novas leituras
        if self.door_detected or self.error_occurred:
            return

        try:
            ranges = msg.ranges
            size = len(ranges)
            sub_vec_a = ranges[0:31]
            sub_vec_b = ranges[size - 30:size - 1]
            check_vec = sub_vec_a + sub_vec_b

            # Verifica se mais de 45 feixes do laser medem mais que 1.5 metros
            if len([i for i in check_vec if i > 1.5]) > 45:
                self.blackboard.log = "Porta aberta detectada"
                if self.node:
                    self.node.get_logger().info(f"[{self.name}] {self.blackboard.log}!")
                self.door_detected = True

        except Exception as e:
            self.blackboard.log = f"Erro ao processar o scan da porta: {e}"
            if self.node:
                self.node.get_logger().error(self.blackboard.log)
            self.error_occurred = True

    def update(self) -> py_trees.common.Status:
        """Chamado a cada tick da árvore. Aqui decidimos o status."""
        
        # 1. Se ocorreu um erro no processamento do Laser
        if self.error_occurred:
            return py_trees.common.Status.FAILURE

        # 2. Se a porta foi detectada aberta
        if self.door_detected:
            # Inicia o cronômetro de 5 segundos (não-bloqueante)
            if self.door_open_time is None:
                self.door_open_time = time.time()
            
            # Se já passaram 5 segundos desde a detecção, encerra com sucesso
            if (time.time() - self.door_open_time) >= 5.0:
                return py_trees.common.Status.SUCCESS
            else:
                # Continua rodando enquanto espera os 5 segundos passarem
                return py_trees.common.Status.RUNNING

        # 3. Verifica o Timeout Geral de 30 segundos
        if (time.time() - self.start_time) > self.timeout_sec:
            self.blackboard.log = "Timeout: A porta não abriu a tempo."
            if self.node:
                self.node.get_logger().warn(f"[{self.name}] {self.blackboard.log}")
            return py_trees.common.Status.FAILURE

        # Se não deu erro, não detectou a porta e não deu timeout, continua rodando (antigo CANCEL)
        return py_trees.common.Status.RUNNING




