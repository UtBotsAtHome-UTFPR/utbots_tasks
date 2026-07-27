import py_trees
import rclpy
from rclpy.node import Node
from rclpy import node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from std_msgs.msg import String

from utbots_actions.action import Transcription
from utbots_actions.action import InterpretNLU
from utbots_actions.action import TextToSpeech

# From utbots_tasks/utbots_tasks/state/basic_voice.py/SendTTSState 
# Uses TextToSpeech from utbots_dependencies/utbots_actions/action/TextToSpeech.action
class SendTTS(py_trees.behaviour.Behaviour):
    """
    Behavior that sends a TextToSpeech goal to the /utbots/tts action server,
    reading the text from the blackboard (key "text"). 
    Status: RUNNING, SUCCESS or FAILURE. 
    """

    def __init__(self, node: Node):
        super().__init__("SendTTS")
        self.node = node

        # Flags that monitor the goal flow
        self.goal_sent = False
        self.goal_done = False
        self.goal_accepted = False
        self.result = None
        self.goal_handle = None

        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="text", access=py_trees.common.Access.READ)

        self.action_client = ActionClient(
            self.node, 
            TextToSpeech, 
            "/utbots/tts"
        )

    def initialise(self):
        # Resets flags each time the behavior status is RUNNING (node is running)
        self.goal_sent = False
        self.goal_done = False
        self.goal_accepted = False
        self.result = None
        self.goal_handle = None

    # Called every time a "tick" occurs, while the node is active
    def update(self) -> py_trees.common.Status:

        # Wait until server "utbots/tts" exists
        if not self.action_client.wait_for_server(timeout_sec=0.0):
            self.node.get_logger().info("Waiting for action server...")
            return py_trees.common.Status.RUNNING
        
        # Send goal once - reads string from blackboard, assembles the ros 
        # message and dispatches the goal asynchronously
        if not self.goal_sent:
            goal_msg = TextToSpeech.Goal()
            text_msg = String()
            text_msg.data = str(self.blackboard.text)
            goal_msg.text = text_msg
            
            future = self.action_client.send_goal_async(goal_msg)
            future.add_done_callback(self.goal_response_callback)
            self.goal_sent = True
        
        # If not finished talking, status RUNNING
        if not self.goal_done:
            return py_trees.common.Status.RUNNING

        # Checks is result.status = SUCCESS
        if self.result.status == GoalStatus.STATUS_SUCCEEDED:
            self.node.get_logger().info("TTS action completed successfully")
            return py_trees.common.Status.SUCCESS

        # If not success: failed, aborted or cancelled
        self.node.get_logger().warn(f"TTS action failed with status: {self.result.status}")
        return py_trees.common.Status.FAILURE

    def terminate(self, new_status: py_trees.common.Status):
        # If status = INVALID, cancel goal
        if new_status == py_trees.common.Status.INVALID and self.goal_handle is not None:
            if self.goal_accepted and not self.goal_done:
                self.node.get_logger().info("Cancelling active TTS goal...")
                self.goal_handle.cancel_goal_async()

    def goal_response_callback(self, future):
        self.goal_handle = future.result()

        if not self.goal_handle.accepted:
            self.node.get_logger().error("TTS Goal rejected by server")
            self.goal_done = True
            return
        
        self.goal_accepted = True

        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        self.result = future.result()
        self.goal_done = True

class SendTTSNode(Node):

    def __init__(self):
        super().__init__("bt_node")

        # Fill blackboard with results
        blackboard = py_trees.blackboard.Client(name="Global")
        blackboard.register_key(key="text", access=py_trees.common.Access.WRITE)
        blackboard.text = "Hello, this is Hestia!"

        root = SendTTS(self)

        self.tree = py_trees.trees.BehaviourTree(root)
        self.create_timer(0.1, self.tick_tree)

    def tick_tree(self):
        self.tree.tick()

# From utbots_tasks/utbots_tasks/state/basic_voice.py/CoquiTTSState 
class CoquiTTS(py_trees.behaviour.Behaviour):

    def __init__(self, node: Node):
        super().__init__("CoquiTTS")
        self.node = node

        self.goal_sent = False
        self.goal_done = False
        self.goal_accepted = False
        self.result = None
        self.goal_handle = None

        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="tts_text", access=py_trees.common.Access.READ)

        self.action_client = ActionClient(
            self.node, 
            TextToSpeech, 
            "/utbots/tts"
        )

    def initialise(self):
        # Resets flags each time the behavior status is RUNNING 
        self.goal_sent = False
        self.goal_done = False
        self.goal_accepted = False
        self.result = None
        self.goal_handle = None

    def update(self) -> py_trees.common.Status:

        # Wait until server exists
        if not self.action_client.wait_for_server(timeout_sec=0.0):
            self.node.get_logger().info("Waiting for action server...")
            return py_trees.common.Status.RUNNING
        
        # Send goal once
        if not self.goal_sent:
            goal = TextToSpeech.Goal()

            if self.blackboard.exists("tts_text") and self.blackboard.tts_text is not None:
                goal.text.data = self.blackboard.tts_text
            else:
                goal.text.data = "Hi my name is Hestia. I have nothing to say in the moment."
            
            future = self.action_client.send_goal_async(goal)
            future.add_done_callback(self.goal_response_callback)
            self.goal_sent = True
        
        # Wait until result arrives
        if not self.goal_done:
            return py_trees.common.Status.RUNNING

        # Checks is result.status = SUCCESS
        if self.result.status == GoalStatus.STATUS_SUCCEEDED:
            self.node.get_logger().info("TTS action completed successfully")
            return py_trees.common.Status.SUCCESS

        # If not success: failed, aborted or cancelled
        self.node.get_logger().warn(f"TTS action failed with status: {self.result.status}")
        return py_trees.common.Status.FAILURE

    def terminate(self, new_status: py_trees.common.Status):
        # If status = INVALID, cancel goal
        if new_status == py_trees.common.Status.INVALID and self.goal_handle is not None:
            if self.goal_accepted and not self.goal_done:
                self.node.get_logger().info("Cancelling active TTS goal...")
                self.goal_handle.cancel_goal_async()

    # Called if server accepted (uses result_callback to monitor the audio) 
    # or refused request (sets goal_done = True)
    def goal_response_callback(self, future):
        self.goal_handle = future.result()

        if not self.goal_handle.accepted:
            self.node.get_logger().error("TTS Goal rejected by server")
            self.goal_done = True
            return
        
        self.goal_accepted = True

        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    # Called when the TTS server finishes the message. Sets goal_done = True
    def result_callback(self, future):
        self.result = future.result()
        self.goal_done = True

class CoquiTTSNode(Node):

    def __init__(self):
        super().__init__("bt_node")

        # Fill blackboard with results
        blackboard = py_trees.blackboard.Client(name="Global")
        
        blackboard.register_key(key="tts_text", access=py_trees.common.Access.WRITE)
        blackboard.tts_text = "Hello, world!"

        root = CoquiTTS(self)

        self.tree = py_trees.trees.BehaviourTree(root)
        self.create_timer(0.1, self.tick_tree)

    def tick_tree(self):
        self.tree.tick()

# From utbots_tasks/utbots_tasks/state/basic_voice.py/WhisperSTTState 
class WhisperSTT(py_trees.behaviour.Behaviour):

  def _init_(self, name, node) -> None:
    super(WhisperSTT, self)._init_(name)
    self.node = node
    self.action_client = ActionClient(
        self.node,
        Transcription,
        "/utbots/transcription",
    )

    self.goal = None
    self.goal_handle = None
    self.result_future = None
    self.goal_future = None
    self.blackboard = py_trees.blackboard.Client()
    self.blackboard.register_key(
        key="whispered", access=py_trees.common.Access.WRITE
    )
    self.blackboard.register_key(
        key="nlu_input_text", access=py_trees.common.Access.WRITE
    )

  def initialise(self) -> None:
    self.goal = Transcription.Goal()

    if not self.action_client.wait_for_server(timeout_sec=1.0):
      self.node.get_logger().error(
          f"[{self.name}] Transcription server offline!"
      )
      return

    self.goal_future = self.action_client.send_goal_async(self.goal)

  def update(self) -> py_trees.common.Status:
    if self.goal_future is None:
      return py_trees.common.Status.FAILURE

    if not self.goal_future.done():
      return py_trees.common.Status.RUNNING

    if self.goal_handle is None:
      self.goal_handle = self.goal_future.result()
      if not self.goal_handle.accepted:
        return py_trees.common.Status.FAILURE

      self.result_future = self.goal_handle.get_result_async()
      return py_trees.common.Status.RUNNING

    if not self.result_future.done():
      return py_trees.common.Status.RUNNING

    result = self.result_future.result().result

    transcribed_text = (
        result.text.data if hasattr(result.text, "data") else str(result.text)
    )
    self.blackboard.whispered = transcribed_text
    self.blackboard.nlu_input_text = transcribed_text  # Connects Whisper -> NLU

    return py_trees.common.Status.SUCCESS

  def terminate(self, new_status) -> None:
    self.goal = None
    self.goal_future = None
    self.goal_handle = None
    self.result_future = None

# From utbots_tasks/utbots_tasks/state/basic_voice.py/NLUInference 
class NLUInference(py_trees.behaviour.Behaviour):

  def _init_(self, name, node, verbose=False) -> None:
    super(NLUInference, self)._init_(name)
    self.verbose = verbose
    self.node = node
    self.action_client = ActionClient(
        self.node,
        InterpretNLU,
        "/utbots/interpret_nlu",
    )
    self.goal_handle = None
    self.goal = None
    self.goal_future = None
    self.response_future = None

    self.blackboard = py_trees.blackboard.Client()
    self.blackboard.register_key(
        key="nlu_input_text", access=py_trees.common.Access.READ
    )
    self.blackboard.register_key(
        key="nlu_output", access=py_trees.common.Access.WRITE
    )
    self.blackboard.register_key(
        key="nlu_intent", access=py_trees.common.Access.WRITE
    )
    self.blackboard.register_key(
        key="nlu_data", access=py_trees.common.Access.WRITE
    )
    self.blackboard.register_key(
        key="nlu_chat", access=py_trees.common.Access.WRITE
    )

  def initialise(self):
    self.goal = InterpretNLU.Goal()
    self.goal_handle = None
    self.goal_future = None
    self.response_future = None

    try:
      self.goal.nlu_input.data = self.blackboard.nlu_input_text or ""
    except AttributeError:
      self.goal.nlu_input.data = ""

    if not self.action_client.wait_for_server(timeout_sec=1.0):
      if self.verbose:
        self.node.get_logger().error(f"[{self.name}] NLU server offline!")
      return

    self.goal_future = self.action_client.send_goal_async(self.goal)

  def update(self) -> py_trees.common.Status:
    if self.goal_future is None:
      return py_trees.common.Status.FAILURE

    if not self.goal_future.done():
      return py_trees.common.Status.RUNNING

    if self.goal_handle is None:
      self.goal_handle = self.goal_future.result()
      if not self.goal_handle.accepted:
        return py_trees.common.Status.FAILURE

      self.response_future = self.goal_handle.get_result_async()
      return py_trees.common.Status.RUNNING

    if not self.response_future.done():
      return py_trees.common.Status.RUNNING

    response = self.response_future.result().result

    self.blackboard.nlu_output = response.nlu_output.data
    self.blackboard.nlu_intent = response.task.data
    self.blackboard.nlu_data = response.data.data
    self.blackboard.nlu_chat = response.bot_response.data

    if self.verbose:
      self.node.get_logger().info(
          f"[{self.name}] NLU Intent: {self.blackboard.nlu_intent}"
      )
      self.node.get_logger().info(
          f"[{self.name}] NLU Data: {self.blackboard.nlu_data}"
      )

    return py_trees.common.Status.SUCCESS

  def terminate(self, new_status) -> None:
    self.goal = None
    self.goal_future = None
    self.goal_handle = None
    self.response_future = None