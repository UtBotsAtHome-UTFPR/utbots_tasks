import rclpy
import py_trees
import asyncio

from rclpy.node import Node
from cv_bridge import CvBridge
from rclpy.action import ActionClient
from utbots_actions.action import MPPose
from mediapipe_track.mediapipe_pose import MediaPipePose 

class GetPersonPointLeaf(py_trees.behaviour.Behaviour):

    def __init__(self, name, node: Node, action_server="mediapipe_pose", verbose=False) -> None:

        super(GetPersonPointLeaf, self).__init__(name)

        self.node = node

        self.client = ActionClient(
            node,
            MPPose,
            action_server,
        )

        self.verbose = verbose

        self.goal_sent = False
        self.goal_done = False
        self.goal_accepted = False

        self.goal_handle = None
        self.result = None

        self.blackboard = self.attach_blackboard_client(
            name=self.name
        )

        self.blackboard.register_key(
            key="mediapipe_img",
            access=py_trees.common.Access.READ
        )

        self.blackboard.register_key(
            key="mediapipe_skeleton_img",
            access=py_trees.common.Access.WRITE
        )

        self.blackboard.register_key(
            key="mediapipe_skeleton_point",
            access=py_trees.common.Access.WRITE
        )

        self.blackboard.register_key(
            key="mediapipe_points_normalized",
            access=py_trees.common.Access.WRITE
        )


    def initialise(self):

        self.goal_sent = False
        self.goal_done = False
        self.goal_accepted = False

        self.goal_handle = None
        self.result = None


    def update(self):

        
        if not self.client.wait_for_server(timeout_sec=0.0):

            self.node.get_logger().info(
                "waiting for action server..."
            )

            return py_trees.common.Status.RUNNING


       
        if not self.goal_sent:

            if not self.blackboard.exists("mediapipe_img"):

                self.node.get_logger().error(
                    "mediapipe_img not found in blackboard"
                )

                return py_trees.common.Status.FAILURE


            goal = MPPose.Goal()

            goal.get_torso_point.data = True
            goal.get_drawn.data = True

            goal.image = self.blackboard.mediapipe_img

            future = self.client.send_goal_async(goal)

            future.add_done_callback(
                self.goal_response_callback
            )

            self.goal_sent = True

            return py_trees.common.Status.RUNNING


        if not self.goal_done:

            return py_trees.common.Status.RUNNING


        if not self.goal_accepted:

            return py_trees.common.Status.FAILURE


        return py_trees.common.Status.SUCCESS


    def goal_response_callback(self, future):

        self.goal_handle = future.result()

        if not self.goal_handle.accepted:

            self.node.get_logger().error(
                "goal rejected"
            )

            self.goal_done = True
            self.goal_accepted = False

            return


        self.goal_accepted = True

        result_future = self.goal_handle.get_result_async()

        result_future.add_done_callback(
            self.result_callback
        )


    def result_callback(self, future):

        self.result = future.result()

        response = self.result.result

        self.blackboard.mediapipe_skeleton_img = (
            response.skeleton_img
        )

        self.blackboard.mediapipe_skeleton_point = (
            response.point
        )

        self.blackboard.mediapipe_points_normalized = (
            response.skeleton_points_normalized
        )

        self.goal_done = True