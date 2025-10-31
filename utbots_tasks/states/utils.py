import yasmin
from yasmin import Blackboard, StateMachine, State
from yasmin_ros import set_ros_loggers
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from yasmin_viewer import YasminViewerPub

class CheckIterations(State):
    """
    Class to check the number of iterations and decide whether to repeat or continue the state machine.

    Blackboard:
        iterations (int): The maximum number of iterations allowed.

    Attributes:
        None
    """
    def __init__(self) -> None:
        super().__init__(outcomes=["continue", "repeat"])

    def execute(self, blackboard: Blackboard) -> str:
        max_iterations = blackboard["iterations"]
        current_iteration = 0
        if current_iteration < max_iterations:
            current_iteration += 1
            return "continue"
        else:
            return "repeat"