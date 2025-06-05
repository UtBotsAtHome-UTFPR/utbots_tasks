from yasmin import State, Blackboard
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from rclpy.qos import QoSProfile
from rclpy.task import Future
import rclpy
import time
import yaml

class GoToState(State):
    def __init__(self, node: Node):
        super().__init__(outcomes=['succeeded', 'failed'])
        self.node = node
        self.result_msg = None
        self.result_future = Future()

        # Publishers and subscribers
        self.goal_pub = self.node.create_publisher(PoseStamped, '/navigate_to_pose/goal', QoSProfile(depth=10))
        self.result_sub = self.node.create_subscription(
            String,
            '/navigate_to_pose/result',
            self.result_callback,
            QoSProfile(depth=10)
        )

    def result_callback(self, msg: String):
        if not self.result_future.done():
            self.result_future.set_result(msg)

    def execute(self, blackboard: Blackboard) -> str:
        self.node.get_logger().info("Executing state: go_to")

        # Reads the goal from the (universal) blackboard
        goal_msg = PoseStamped()
        goal = blackboard.get("goal")
        goal_msg.pose.position.x = goal.pose.position.x
        goal_msg.pose.position.y = goal.pose.position.y
        goal_msg.pose.orientation.z = goal.pose.orientation.z
        goal_msg.pose.orientation.w = goal.pose.orientation.w

        # Sends the goal to the navigation topic
        self.goal_pub.publish(goal_msg)
        self.node.get_logger().info("Goal published, waiting for result...")

        # Wait for the result message with a timeout
        rclpy.spin_until_future_complete(self.node, self.result_future, timeout_sec=600)

        if self.result_future.done():
            result_msg = self.result_future.result()
            if result_msg.data.lower() == "succeeded":
                return 'succeeded'
            else:
                return 'failed'
        else:
            self.node.get_logger().error("Timeout waiting for result")
            return 'failed'

class GoToWaypointState(GoToState):
    def __init__(self, node: Node, yaml_file_path: str):
        super().__init__(node)
        self.yaml_path = yaml_file_path

    def retrieve_waypoint(nametag: str, yaml_path: str) -> PoseStamped | None:
        if not nametag:
            return None

        with open(yaml_path, 'r') as file:
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
        return None
    
    def execute(self, blackboard: Blackboard) -> str:
        waypoint_name = blackboard.get("waypoint")
        self.node.get_logger().info(f"Retrieving waypoint: {waypoint_name}")

        waypoint_nametag = blackboard.get("waypoint_nametag")
        waypoint = self.retrieve_waypoint(waypoint_nametag, self.yaml_path)
        if waypoint is None:
            self.node.get_logger().error("Waypoint not found or invalid.")
            return 'failed'

        # Coloca o waypoint carregado no blackboard para o GoToState usar
        blackboard.set("goal", waypoint)

        # Agora reutiliza o comportamento da superclasse GoToState
        return super().execute(blackboard)
        
class WaitDoorOpenState(State):
    def __init__(self, node: Node, timeout_sec: int = 200):
        super().__init__(outcomes=['succeeded', 'timed_out'])
        self.node = node
        self.timeout_sec = timeout_sec
        self.pub_vm = node.create_publisher(String, '/voice_msgs', QoSProfile(depth=10))  # Pub VM equivalent

    def publish_message(self, text: str):
        msg = String()
        msg.data = text
        self.pub_vm.publish(msg)

    def execute(self, blackboard: Blackboard) -> str:
        self.node.get_logger().info("Waiting for door to open")
        self.publish_message("Waiting for door")

        start_time = time.time()

        while (time.time() - start_time) < self.timeout_sec:
            try:
                scan = self._wait_for_scan(timeout=5.0)
                if scan is None:
                    continue

                ranges = scan.ranges
                size = len(ranges)
                sub_vec_a = ranges[0:31]
                sub_vec_b = ranges[size - 30:size - 1]
                check_vec = sub_vec_a + sub_vec_b

                if len([i for i in check_vec if i > 1.5]) > 45:
                    self.node.get_logger().info("Detected open door")
                    self.publish_message("Detected open door")
                    time.sleep(5)
                    return "succeeded"

            except Exception as e:
                self.node.get_logger().error(f"Error while waiting for door: {e}")

        self.node.get_logger().info("Timed out waiting for door to open")
        self.publish_message("Timed out waiting for door to open")
        return "timed_out"

    def _wait_for_scan(self, timeout=5.0):
        future = Future()

        def callback(msg):
            if not future.done():
                future.set_result(msg)

        sub = self.node.create_subscription(LaserScan, '/scan', callback, QoSProfile(depth=10))
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        self.node.destroy_subscription(sub)
        return future.result() if future.done() else None

