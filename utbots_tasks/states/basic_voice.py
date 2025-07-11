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
    
    def __init__(self,verbose=False) -> None:
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
        self.verbose = verbose
        if(self.verbose):
            yasmin.YASMIN_LOG_INFO("NLUInference initialized")
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
            response.nlu_output.data
        )  # Store the result sequence in the blackboard

        blackboard["nlu_intent"] = (
            response.task.data
        )  # Store the result sequence in the blackboard

        blackboard["nlu_data"] = (
            response.data.data
        )  # Store the result sequence in the blackboard
        if(self.verbose):
            yasmin.YASMIN_LOG_INFO(f"NLU Output: {blackboard['nlu_output']}")
            yasmin.YASMIN_LOG_INFO(f"NLU Intent: {blackboard['nlu_intent']}")
            yasmin.YASMIN_LOG_INFO(f"NLU Data: {blackboard['nlu_data']}")
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

PROCESS_NLU=[ 
    "greet",                # 0
    "introduce_robot",      # 1
    "affirm",               # 2
    "deny",                 # 3
    "mood_great",           # 4
    "mood_unhappy",         # 5
    "follow",               # 6
    "stop",                 # 7
    "go_to",                # 8
    "say_operator_name",    # 9
    "identify_operator",    # 10
    "describe_ambient",     # 11
    "like_drink",           # 12
    "pick_object",          # 13
    "default"]             # 14

def get_process_nlu():
    return PROCESS_NLU



class NLUProcess(CbState):
    """
    Class representing the NLU process.

    This class contains methods to handle the NLU process in a finite state machine.

    Attributes:
        None
    """
    def __init__(self, verbose=False) -> None:
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
            outcomes=PROCESS_NLU, cb=self.nlu_process_cb
        )
        

        self.verbose = verbose

    def nlu_process_cb(self,blackboard: Blackboard) -> str:
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
            data=blackboard["nlu_data"].rsplit("\"value\":")[1].rsplit(",")[0].replace('"', '').strip()
        except:
            data=None
        answer=None
        name=data
        ambient=data
        outcome=None
        match task:
            case "greet":
                answer="Hello, my name is hestia!How are you?"
                outcome="greet"
            case "introduce_robot":
                answer="Hello, my name is hestia!Im a service robot designed by the Utbots team, from Curitiba, Brazil. \
                    (bla bla bla)."
                outcome="introduce_robot"
            case "affirm":
                answer="The operator affirmed my request."
                outcome="affirm"
            case "deny":
                answer="The operator denied my request."
                outcome="deny"
            case "mood_great":
                answer="Amazing!"
                outcome="mood_great"
            case "mood_unhappy":
                answer="Oh thats sad!"
                outcome="mood_unhappy"
            case "follow":
                answer="I was asked to follow the operator.Wait for follow mode to start."
                outcome="follow"
            case "stop":
                answer="I was asked to stop! Stopping."
                outcome="stop"
            case "go_to":
                answer=f"I was asked to go to the {data}!Navigation starting."
                blackboard["waypoint_nametag"]=data
                outcome="go_to"
            case "say_operator_name":
                # answer=f"I was asked to say the operators name!The name is {name}."
                answer= f'Is the operators name {name}?Please say Yes, it is or No, it isn t.'
                blackboard["name"]= name           
                outcome="say_operator_name"
            case "identify_operator":
                # answer=f"I was asked to say the operators name!The name is {name}."
                answer= f'Is the operators name {name}?Please say Yes, it is or No, it isn t.'
                blackboard["name"]= name           
                outcome="identify_operator"
            case "describe_ambient":
                answer=f"I was asked to describe the ambient {ambient}!"
                outcome="describe_ambient"
            case "like_drink":
                answer=f"The operator likes {data}, does he?Please deny or affirm the sentence."
                outcome="describe_ambient"
                blackboard["drink"]= data 
            case "pick_object":
                answer=f"I was asked to pick {data}!Please deny or confirm the request."
                outcome="describe_ambient"
            case _:  # Default case
                answer="Hello, my name is hestia!I wasn't able to understand what you said to me!"
                outcome="default"
        blackboard["tts_text"]=answer
        if(self.verbose):
            yasmin.YASMIN_LOG_INFO(f"NLU Intent: {task}")
            yasmin.YASMIN_LOG_INFO(f"NLU Data: {data}")
            yasmin.YASMIN_LOG_INFO(f"Answer: {answer}")
            yasmin.YASMIN_LOG_INFO(f"Outcome: {outcome}")
        blackboard["nlu_data"] = data
        return outcome


# def greet_and_name_cb(blackboard: Blackboard) -> str:
#     """
#     Retrieves the next waypoint from the list of random waypoints.

#     Updates the blackboard with the pose of the next waypoint.

#     Args:
#         blackboard (Blackboard): The blackboard instance holding current state data.

#     Returns:
#         str: Outcome indicating whether there is a next waypoint (HAS_NEXT) or if
#              navigation is complete (END).
#     """

#     try:
#         name=blackboard["nlu_data"]
#         blackboard["tts_text"]= f'Is the operators name {name}?Please say Yes, it is or No, it isn t.'
#         blackboard["name"]= name
#     except:
#         blackboard["tts_text"]= "I was not able to understand the operators name. Please say your name again."

#     return "to_tts"#pass to NLU

# def greet_and_name_cb(blackboard: Blackboard) -> str:
#     """
#     Retrieves the next waypoint from the list of random waypoints.

#     Updates the blackboard with the pose of the next waypoint.

#     Args:
#         blackboard (Blackboard): The blackboard instance holding current state data.

#     Returns:
#         str: Outcome indicating whether there is a next waypoint (HAS_NEXT) or if
#              navigation is complete (END).
#     """

#     try:
#         name=blackboard["nlu_data"]
#         blackboard["tts_text"]= f'Is the operators name {name}?Please say Yes, it is or No, it isn t.'
#         blackboard["name"]= name
#     except:
#         blackboard["tts_text"]= "I was not able to understand the operators name. Please say your name again."

#     return "to_tts"#pass to NLU


# version: "3.1"

# nlu:
# - intent: greet
#.
# - intent: introduce_robot
#.
# - intent: affirm
#.

# - intent: deny
#.
# - intent: mood_great
#.
# - intent: mood_unhappy
#.
# - intent: follow
#.
# - intent: stop
#.
# - intent: go_to
#   examples: |
#     - go to the [kitchen](room)
#     - navigate to the [living room](room)
#     - navigate to the [bedroom](room)
#     - navigate to the [office](room)
#.
# - intent: identify_operator
#.
# - intent: say_operator_name
#   examples: |
#     - My name is [James](person) 
#.
#     -I am [James](person) 
#.
#     - [James](person) 
#.
#     - My name is [Mary](person)
#.
#     -I am [Mary](person)
#.
#     - [Mary](person)
#.

# - intent: describe_ambient
#.

def generate_ask_name_sm():
    ask_name_sm = StateMachine(outcomes=[SUCCEED, CANCEL, ABORT])
    ask_name_sm.add_state(
        "ASK_SOMETHING",
        CoquiTTSState(),
        transitions={
            SUCCEED: "CALLING_WHISPER",
            CANCEL: ABORT,
        },
        remappings={"tts_text":"ask_name"}
    )

    ask_name_sm.add_state(
        "CALLING_WHISPER",
        WhisperSTTState(),
        transitions={
            SUCCEED: "WHISPER_PROCESS",
            CANCEL: ABORT,
            ABORT: ABORT,
        },
    )

    ask_name_sm.add_state(
        "WHISPER_PROCESS",
        CbState(["process_whisper1","process_whisper2","process_whisper3"],whisper_process_cb),
        transitions={
            "process_whisper1": "CALLING_WHISPER",
            "process_whisper2": "NLU_INFERENCE",
            # "process_whisper3": "outcome4",

        },
    )

    ask_name_sm.add_state(
        "NLU_INFERENCE",
        NLUInference(),
        transitions={
            SUCCEED: "NLU_PROCESS",
            CANCEL: ABORT,
            ABORT: ABORT,
        },
        remappings={"nlu_input_text": "whispered"},
    )

    ask_name_sm.add_state(
        "NLU_PROCESS",
        NLUProcess(True),  # Set verbose to True for detailed logging     
        transitions={
            PROCESS_NLU[0]: "ASK_SOMETHING",
            PROCESS_NLU[1]: "ASK_SOMETHING",
            PROCESS_NLU[2]: "ASK_SOMETHING",
            PROCESS_NLU[3]: "ASK_SOMETHING",
            PROCESS_NLU[4]: "ASK_SOMETHING",
            PROCESS_NLU[5]: "ASK_SOMETHING",
            PROCESS_NLU[6]: "ASK_SOMETHING",
            PROCESS_NLU[7]: "ASK_SOMETHING",
            PROCESS_NLU[8]: "ASK_SOMETHING",
            PROCESS_NLU[9]: "VERIFY",
            PROCESS_NLU[10]: "VERIFY",
            PROCESS_NLU[11]: "ASK_SOMETHING",
            PROCESS_NLU[12]: "ASK_SOMETHING",
            PROCESS_NLU[13]: "ASK_SOMETHING",
            PROCESS_NLU[14]: "ASK_SOMETHING",
        },
    )

    ask_name_sm.add_state(
        "VERIFY",
        CoquiTTSState(),
        transitions={
            SUCCEED: "CALLING_WHISPER_VER",
            CANCEL: ABORT,
        },
    )

    ask_name_sm.add_state(
        "CALLING_WHISPER_VER",
        WhisperSTTState(),
        transitions={
            SUCCEED: "WHISPER_PROCESS_VER",
            CANCEL: ABORT,
            ABORT: ABORT,
        },
    )

    ask_name_sm.add_state(
        "WHISPER_PROCESS_VER",
        CbState(["process_whisper1","process_whisper2","process_whisper3"],whisper_process_cb),
        transitions={
            "process_whisper1": "CALLING_WHISPER_VER",
            "process_whisper2": "NLU_INFERENCE_VER",
            # "process_whisper3": "outcome4",

        },
    )

    ask_name_sm.add_state(
        "NLU_INFERENCE_VER",
        NLUInference(),
        transitions={
            SUCCEED: "NLU_PROCESS_VER",
            CANCEL: ABORT,
            ABORT: ABORT,
        },
        remappings={"nlu_input_text": "whispered"},
    )

    ask_name_sm.add_state(
        "NLU_PROCESS_VER",
        NLUProcess(True),  # Set verbose to True for detailed logging     
        transitions={
            PROCESS_NLU[0]: "ASK_SOMETHING",
            PROCESS_NLU[1]: "ASK_SOMETHING",
            PROCESS_NLU[2]: SUCCEED,
            PROCESS_NLU[3]: "ASK_SOMETHING",
            PROCESS_NLU[4]: "ASK_SOMETHING",
            PROCESS_NLU[5]: "ASK_SOMETHING",
            PROCESS_NLU[6]: "ASK_SOMETHING",
            PROCESS_NLU[7]: "ASK_SOMETHING",
            PROCESS_NLU[8]: "ASK_SOMETHING",
            PROCESS_NLU[9]: "ASK_SOMETHING",
            PROCESS_NLU[10]: "ASK_SOMETHING",
            PROCESS_NLU[11]: "ASK_SOMETHING",
            PROCESS_NLU[12]: "ASK_SOMETHING",
            PROCESS_NLU[13]: "ASK_SOMETHING",
            PROCESS_NLU[14]: "ASK_SOMETHING",
        },
    )
    return ask_name_sm

def generate_ask_drink_sm():
    ask_drink_sm = StateMachine(outcomes=[SUCCEED, CANCEL, ABORT])
    ask_drink_sm.add_state(
        "ASK_SOMETHING",
        CoquiTTSState(),
        transitions={
            SUCCEED: "CALLING_WHISPER",
            CANCEL: ABORT,
        },
        remappings={"tts_text":"ask_drink"}
    )

    ask_drink_sm.add_state(
        "CALLING_WHISPER",
        WhisperSTTState(),
        transitions={
            SUCCEED: "WHISPER_PROCESS",
            CANCEL: ABORT,
            ABORT: ABORT,
        },
    )

    ask_drink_sm.add_state(
        "WHISPER_PROCESS",
        CbState(["process_whisper1","process_whisper2","process_whisper3"],whisper_process_cb),
        transitions={
            "process_whisper1": "CALLING_WHISPER",
            "process_whisper2": "NLU_INFERENCE",
            # "process_whisper3": "outcome4",

        },
    )

    ask_drink_sm.add_state(
        "NLU_INFERENCE",
        NLUInference(),
        transitions={
            SUCCEED: "NLU_PROCESS",
            CANCEL: ABORT,
            ABORT: ABORT,
        },
        remappings={"nlu_input_text": "whispered"},
    )

    ask_drink_sm.add_state(
        "NLU_PROCESS",
        NLUProcess(True),  # Set verbose to True for detailed logging     
        transitions={
            PROCESS_NLU[0]: "ASK_SOMETHING",
            PROCESS_NLU[1]: "ASK_SOMETHING",
            PROCESS_NLU[2]: "ASK_SOMETHING",
            PROCESS_NLU[3]: "ASK_SOMETHING",
            PROCESS_NLU[4]: "ASK_SOMETHING",
            PROCESS_NLU[5]: "ASK_SOMETHING",
            PROCESS_NLU[6]: "ASK_SOMETHING",
            PROCESS_NLU[7]: "ASK_SOMETHING",
            PROCESS_NLU[8]: "ASK_SOMETHING",
            PROCESS_NLU[9]: "ASK_SOMETHING",
            PROCESS_NLU[10]: "ASK_SOMETHING",
            PROCESS_NLU[11]: "VERIFY",
            PROCESS_NLU[12]: "ASK_SOMETHING",
        },
    )

    ask_drink_sm.add_state(
        "VERIFY",
        CoquiTTSState(),
        transitions={
            SUCCEED: "CALLING_WHISPER_VER",
            CANCEL: ABORT,
        },
    )

    ask_drink_sm.add_state(
        "CALLING_WHISPER_VER",
        WhisperSTTState(),
        transitions={
            SUCCEED: "WHISPER_PROCESS_VER",
            CANCEL: ABORT,
            ABORT: ABORT,
        },
    )

    ask_drink_sm.add_state(
        "WHISPER_PROCESS_VER",
        CbState(["process_whisper1","process_whisper2","process_whisper3"],whisper_process_cb),
        transitions={
            "process_whisper1": "CALLING_WHISPER_VER",
            "process_whisper2": "NLU_INFERENCE_VER",
            # "process_whisper3": "outcome4",

        },
    )

    ask_drink_sm.add_state(
        "NLU_INFERENCE_VER",
        NLUInference(),
        transitions={
            SUCCEED: "NLU_PROCESS_VER",
            CANCEL: ABORT,
            ABORT: ABORT,
        },
        remappings={"nlu_input_text": "whispered"},
    )

    ask_drink_sm.add_state(
        "NLU_PROCESS_VER",
        NLUProcess(True),  # Set verbose to True for detailed logging     
        transitions={
            PROCESS_NLU[0]: "ASK_SOMETHING",
            PROCESS_NLU[1]: "ASK_SOMETHING",
            PROCESS_NLU[2]: SUCCEED,
            PROCESS_NLU[3]: "ASK_SOMETHING",
            PROCESS_NLU[4]: "ASK_SOMETHING",
            PROCESS_NLU[5]: "ASK_SOMETHING",
            PROCESS_NLU[6]: "ASK_SOMETHING",
            PROCESS_NLU[7]: "ASK_SOMETHING",
            PROCESS_NLU[8]: "ASK_SOMETHING",
            PROCESS_NLU[9]: "ASK_SOMETHING",
            PROCESS_NLU[10]: "ASK_SOMETHING",
            PROCESS_NLU[11]: "ASK_SOMETHING",
            PROCESS_NLU[12]: "ASK_SOMETHING",
        },
    )
    return ask_drink_sm