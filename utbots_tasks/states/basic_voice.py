# from yasmin import State, Blackboard
# from yasmin import ActionState, SUCCEED, ABORT
# from nav2_msgs.action import NavigateToPose
from utbots_actions.action import TextToSpeech, Transcription, InterpretNLU
import yasmin
import rclpy
import yasmin
from yasmin import State, CbState, Blackboard, StateMachine
from yasmin_ros import ActionState
from yasmin_ros import set_ros_loggers
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
# from yasmin_viewer import YasminViewerPub

#BLACKBOARD:
# blackboard["tts_text"]=None
# blackboard["text"]
# blackboard["whispered"]
# blackboard["nlu_input_text"]
# blackboard["nlu_output"] = (
#     response.nlu_output
# )  # Store the result sequence in the blackboard

# blackboard["nlu_intent"] = (
#     response.task
# )  # Store the result sequence in the blackboard

# blackboard["nlu_data"] = (
#     response.data
# )  # Store the result sequence in the blackboard

# blackboard["waypoint_nametag"]=data

from std_msgs.msg import String

class SendTTSState(ActionState):
    def __init__(self) -> None:
         super().__init__(
            TextToSpeech,  # action type
            "/tts",  # action name
            self.create_goal_handler,  # callback to create the goal
            None,  # outcomes
            None,  # callback to process the response
        )

    def create_goal_handler(self, blackboard: Blackboard) -> TextToSpeech.Goal:
        goal = TextToSpeech.Goal()
        goal.request.text = blackboard["text"]
        return goal


class WhisperSTTState(ActionState):
    """
    Class representing the state of the Whisper action.

    Inherits from ActionState and implements methods to handle the
    Whisper action in a finite state machine.

    Attributes:
        None
    """
    
    def __init__(self) -> None:
        """
        Initializes the WhisperTTSState.

        Sets up the action type and the action name for the Whisper
        action. Initializes goal, response handler, and feedback
        processing callbacks.

        Parameters:
            None

        Returns:
            None
        """
        super().__init__(
            Transcription,  # action type
            "/utbots/transcription",  # action name
            self.create_goal_handler,  # callback to create the goal
            None,  # outcomes. Includes (SUCCEED, ABORT, CANCEL)
            self.response_handler,  # callback to process the response
            None, # self.print_feedback,  # callback to process the feedback
        )

    def create_goal_handler(self, blackboard: Blackboard) -> Transcription.Goal:
        """
        Creates the goal for the Whisper action.

        This mblackboard["tts_text"]ethod retrieves the input value from the blackboard and
        populates the Transcription goal.

        Parameters:
            blackboard (Blackboard): The blackboard containing the state
            information.

        Returns:
            Transcription.Goal: The populated goal object for the Transcription action.

        Raises:
            KeyError: If the expected key is not present in the blackboard.
        """
        goal = Transcription.Goal()
        # goal.order = blackboard["n"]  # Retrieve the input value 'n' from the blackboard
        return goal

    def response_handler(self, blackboard: Blackboard, response: Transcription.Result) -> str:
        """
        Handles the response from the Whisper action.

        This method processes the result of the Whisper action and
        stores it in the blackboard.

        Parameters:
            blackboard (Blackboard): The blackboard to store the result.
            response (Transcription.Result): The result object from the Whisper action.

        Returns:
            str: Outcome of the operation, typically SUCCEED.

        Raises:
            None
        """
        blackboard["whispered"] = (
            response.text.data
        )  # Store the result sequence in the blackboard
        print(blackboard["whispered"])
        return SUCCEED

    # def print_feedback(
    #     self, blackboard: Blackboard, feedback: Whisper.Feedback
    # ) -> None:
    #     """
    #     Prints feedback from the Whisper action.

    #     This method logs the partial sequence received during the action.

    #     Parameters:
    #         blackboard (Blackboard): The blackboard (not used in this method).
    #         feedback (Whisper.Feedback): The feedback object from the Whisper action.

    #     Returns:
    #         None

    #     Raises:
    #         None
    #     """
    #     yasmin.YASMIN_LOG_INFO(f"Received feedback: {list(feedback.sequence)}")


def whisper_print_result(blackboard: Blackboard) -> str:
    """
    Prints the result of the Whisper action.

    This function logs the final result stored in the blackboard.

    Parameters:
        blackboard (Blackboard): The blackboard containing the result.

    Returns:
        str: Outcome of the operation, typically SUCCEED.

    Raises:
        None
    """
    yasmin.YASMIN_LOG_INFO(f"Result: {blackboard['whispered']}")
    return SUCCEED


class CoquiTTSState(ActionState):
    """
    Class representing the state of the coqui-TTS action.

    Inherits from ActionState and implements methods to handle the
    TTS action in a finite state machine.

    Attributes:
        None
    """
    
    def __init__(self) -> None:
        """
        Initializes the CoquiTTSState.

        Sets up the action type and the action name for the Whisper
        action. Initializes goal, response handler, and feedback
        processing callbacks.

        Parameters:
            None

        Returns:
            None
        """
        super().__init__(
            TextToSpeech,  # action type
            "/utbots/tts",  # action name
            self.create_goal_handler,  # callback to create the goal
            None,  # outcomes. Includes (SUCCEED, ABORT, CANCEL)
            self.response_handler,  # callback to process the response
            None, # self.print_feedback,  # callback to process the feedback
        )

    def create_goal_handler(self, blackboard: Blackboard) -> TextToSpeech.Goal:
        """
        Creates the goal for the Whisper action.

        This method retrieves the input value from the blackboard and
        populates the Transcription goal.

        Parameters:
            blackboard (Blackboard): The blackboard containing the state
            information.

        Returns:
            Transcription.Goal: The populated goal object for the Transcription action.

        Raises:
            KeyError: If the expected key is not present in the blackboard.
        """
        goal = TextToSpeech.Goal()
        text = blackboard["tts_text"]
        # data = String()
        if (text is not None):
            goal.text.data = blackboard["tts_text"]  # Retrieve the input value 'n' from the blackboard
        else:
            goal.text.data =  "Hi my name is Hestia. I have nothing to say in the moment." # Retrieve the input value 'n' from the blackboard
        return goal

    def response_handler(self, blackboard: Blackboard, response: TextToSpeech.Result) -> str:
        """
        Handles the response from the Whisper action.

        This method processes the result of the Whisper action and
        stores it in the blackboard.

        Parameters:
            blackboard (Blackboard): The blackboard to store the result.
            response (Transcription.Result): The result object from the Whisper action.

        Returns:
            str: Outcome of the operation, typically SUCCEED.

        Raises:
            None
        """
        # blackboard["whispered"] = (
        #     response.text
        # )  # Store the result sequence in the blackboard
        return SUCCEED

    # def print_feedback(
    #     self, blackboard: Blackboard, feedback: Whisper.Feedback
    # ) -> None:
    #     """
    #     Prints feedback from the Whisper action.

    #     This method logs the partial sequence received during the action.

    #     Parameters:
    #         blackboard (Blackboard): The blackboard (not used in this method).
    #         feedback (Whisper.Feedback): The feedback object from the Whisper action.

    #     Returns:
    #         None

    #     Raises:
    #         None
    #     """
    #     yasmin.YASMIN_LOG_INFO(f"Received feedback: {list(feedback.sequence)}")


def coqui_print_result(blackboard: Blackboard) -> str:
    """
    Prints the result of the Whisper action.

    This function logs the final result stored in the blackboard.

    Parameters:
        blackboard (Blackboard): The blackboard containing the result.

    Returns:
        str: Outcome of the operation, typically SUCCEED.

    Raises:
        None
    """
    yasmin.YASMIN_LOG_INFO(f"Result: {blackboard['whispered']}")
    return SUCCEED

class NLUInference (ActionState):
    """
    Class representing the state of the rasa-NLU action.

    Inherits from ActionState and implements methods to handle the
    NLU action in a finite state machine.

    Attributes:
        None
    """
    
    def __init__(self) -> None:
        """
        Initializes the NLUInference.

        Sets up the action type and the action name for the Whisper
        action. Initializes goal, response handler, and feedback
        processing callbacks.

        Parameters:
            None

        Returns:
            None
        """
        super().__init__(
            InterpretNLU,  # action type
            "/utbots/interpret_nlu",  # action name
            self.create_goal_handler,  # callback to create the goal
            None,  # outcomes. Includes (SUCCEED, ABORT, CANCEL)
            self.response_handler,  # callback to process the response
            None, # self.print_feedback,  # callback to process the feedback
        )

    def create_goal_handler(self, blackboard: Blackboard) -> InterpretNLU.Goal:
        """
        Creates the goal for the Whisper action.

        This method retrieves the input value from the blackboard and
        populates the Transcription goal.

        Parameters:
            blackboard (Blackboard): The blackboard containing the state
            information.

        Returns:
            Transcription.Goal: The populated goal object for the Transcription action.

        Raises:
            KeyError: If the expected key is not present in the blackboard.
        """
        goal = InterpretNLU.Goal()
        text = blackboard["nlu_input_text"]
        # data = String()
        if (text is not None):
            goal.nlu_input.data = blackboard["nlu_input_text"]  # Retrieve the input value 'n' from the blackboard
        else:
            goal.nlu_input.data =  "" # Retrieve the input value 'n' from the blackboard
        return goal

    def response_handler(self, blackboard: Blackboard, response: InterpretNLU.Result) -> str:
        """
        Handles the response from the Whisper action.

        This method processes the result of the Whisper action and
        stores it in the blackboard.

        Parameters:
            blackboard (Blackboard): The blackboard to store the result.
            response (Transcription.Result): The result object from the Whisper action.

        Returns:
            str: Outcome of the operation, typically SUCCEED.

        Raises:
            None
        """

        # result.nlu_output.data = json.dumps(rasa_output) # Keep the full output if needed for debugging
        # result.task.data = intent
        # result.data.data = json.dumps(entities_list)

        # blackboard["nlu_intent"] = (
        #     response.text
        # )  # Store the result sequence in the blackboard

        blackboard["nlu_output"] = (
            response.nlu_output
        )  # Store the result sequence in the blackboard

        blackboard["nlu_intent"] = (
            response.task
        )  # Store the result sequence in the blackboard

        blackboard["nlu_data"] = (
            response.data
        )  # Store the result sequence in the blackboard

        # # Goal
        # std_msgs/String nlu_input
        # ---
        # # Result
        # std_msgs/String nlu_input
        # std_msgs/String nlu_output
        # std_msgs/String task
        # std_msgs/String data
        # ---
        # # Feedback

        return SUCCEED

    # def print_feedback(
    #     self, blackboard: Blackboard, feedback: Whisper.Feedback
    # ) -> None:
    #     """
    #     Prints feedback from the Whisper action.

    #     This method logs the partial sequence received during the action.

    #     Parameters:
    #         blackboard (Blackboard): The blackboard (not used in this method).
    #         feedback (Whisper.Feedback): The feedback object from the Whisper action.

    #     Returns:
    #         None

    #     Raises:
    #         None
    #     """
    #     yasmin.YASMIN_LOG_INFO(f"Received feedback: {list(feedback.sequence)}")


    def print_result(blackboard: Blackboard) -> str:
        """
        Prints the result of the Whisper action.

        This function logs the final result stored in the blackboard.

        Parameters:
            blackboard (Blackboard): The blackboard containing the result.

        Returns:
            str: Outcome of the operation, typically SUCCEED.

        Raises:
            None
        """
        yasmin.YASMIN_LOG_INFO(f"Result: {blackboard['whispered']}")
        return SUCCEED

def wait_cb(blackboard: Blackboard) -> str:
    from time import sleep
    sleep(1)
    return "waited"

def whisper_process_cb(blackboard: Blackboard) -> str:
    """
    Retrieves the next waypoint from the list of random waypoints.

    Updates the blackboard with the pose of the next waypoint.

    Args:
        blackboard (Blackboard): The blackboard instance holding current state data.

    Returns:
        str: Outcome indicating whether there is a next waypoint (HAS_NEXT) or if
             navigation is complete (END).
    """
    if blackboard["whispered"] == "" or blackboard["whispered"] is None:
        return "process_whisper1"#return to whisper

    return "process_whisper2"#pass to NLU

PROCESS_NLU=[ "greet",
    "introduce_robot",
    "affirm",
    "deny",
    "mood_great",
    "mood_unhappy",
    "follow",
    "stop",
    "go_to",
    "say_operator_name",
    "identify_operator",
    "describe_ambient",
    "default",]

def get_process_nlu():
    return PROCESS_NLU

def nlu_process_cb(blackboard: Blackboard) -> str:
    """
    Retrieves the next waypoint from the list of random waypoints.

    Updates the blackboard with the pose of the next waypoint.

    Args:
        blackboard (Blackboard): The blackboard instance holding current state data.

    Returns:
        str: Outcome indicating whether there is a next waypoint (HAS_NEXT) or if
             navigation is complete (END).
    """

    task=blackboard["nlu_intent"]
    try:
        data=blackboard["nlu_data"].rsplit("\"value\":")[1].rsplit(",")[0]
    except:
        data=None
    answer=None
    name="David"
    ambient="Living Room"
    outcome=None
    match task:
        case "greet":
            answer="Hello, my name is hestia! How are you?"
            outcome="greet"
        case "introduce_robot":
            answer="Hello, my name is hestia! Im a service robot designed by the Utbots team, from Curitiba, Brazil. \
                (bla bla bla)"
            outcome="introduce_robot"
        case "affirm":
            answer="I agree"
            outcome="affirm"
        case "deny":
            answer="I disagree"
            outcome="deny"
        case "mood_great":
            answer="Amazing!"
            outcome="mood_great"
        case "mood_unhappy":
            answer="Oh thats sad!"
            outcome="mood_unhappy"
        case "follow":
            answer="I was asked to follow the operator. Wait for follow mode to start..."
            outcome="follow"
        case "stop":
            answer="I was asked to stop! Stopping..."
            outcome="stop"
        case "go_to":
            answer=f"I was asked to go to the {data}! Navigation starting..."
            blackboard["waypoint_nametag"]=data
            outcome="go_to"
        case "say_operator_name":
            answer=f"I was asked to say the operators name! The name is {name}"
            outcome="say_operator_name"
        case "identify_operator":
            answer=f"I was asked to say the operators name! The name is {name}"
            outcome="identify_operator"
        case "describe_ambient":
            answer=f"I was asked to describe the ambient f{ambient}!"
            outcome="describe_ambient"
        case _:  # Default case
            answer="Hello, my name is hestia! I wasnt able to understand what you said to me!"
            outcome="default"
    blackboard["tts_text"]=answer
    return outcome

        # blackboard["nlu_output"] = (
        #     response.nlu_output
        # )  # Store the result sequence in the blackboard

        # blackboard["nlu_intent"] = (
        #     response.task
        # )  # Store the result sequence in the blackboard

        # blackboard["nlu_data"] = (
        #     response.data
        # )  # Store the result sequence in the blackboard


# version: "3.1"

# nlu:
# - intent: greet
#...
# - intent: introduce_robot
#...
# - intent: affirm
#...

# - intent: deny
#...
# - intent: mood_great
#...
# - intent: mood_unhappy
#...
# - intent: follow
#...
# - intent: stop
#...
# - intent: go_to
#   examples: |
#     - go to the [kitchen](room)
#     - navigate to the [living room](room)
#     - navigate to the [bedroom](room)
#     - navigate to the [office](room)
#...
# - intent: identify_operator
#...
# - intent: say_operator_name
#   examples: |
#     - My name is [James](person) 
#...
#     -I am [James](person) 
#...
#     - [James](person) 
#...
#     - My name is [Mary](person)
#...
#     -I am [Mary](person)
#...
#     - [Mary](person)
#...

# - intent: describe_ambient
#...
