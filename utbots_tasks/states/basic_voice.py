# from yasmin import State, Blackboard
# from yasmin import ActionState, SUCCEED, ABORT
# from nav2_msgs.action import NavigateToPose
from utbots_actions.action import TextToSpeech, Transcription
import yasmin
import rclpy
import yasmin
from yasmin import State, CbState, Blackboard, StateMachine
from yasmin_ros import ActionState
from yasmin_ros import set_ros_loggers
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
# from yasmin_viewer import YasminViewerPub

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


class WhisperTTSState(ActionState):
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

        This mblackboard["stt_text"]ethod retrieves the input value from the blackboard and
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
            response.text
        )  # Store the result sequence in the blackboard
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

    def create_goal_handler(self, blackboard: Blackboard) -> Transcription.Goal:
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
        goal = Transcription.Goal()
        text = blackboard["stt_text"]
        # data = String()
        if (text is not None):
            goal.text.data = blackboard["stt_text"]  # Retrieve the input value 'n' from the blackboard
        else:
            goal.text.data =  "Hi my name is Hestia. I have nothing to say in the moment." # Retrieve the input value 'n' from the blackboard
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

