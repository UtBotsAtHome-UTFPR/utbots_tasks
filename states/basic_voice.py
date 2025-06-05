from yasmin import State, Blackboard
from yasmin import ActionState, SUCCEED, ABORT
from nav2_msgs.action import NavigateToPose


class GoToState(ActionState):
    def __init__(self) -> None:
         super().__init__(
            TextToSpeech,  # action type
            "/tts",  # action name
            self.create_goal_handler,  # callback to create the goal
            None,  # outcomes
            None,  # callback to process the response
        )

    def create_goal_handler(self, blackboard: Blackboard) -> NavigateToPose.Goal:
        goal = NavigateToPose.Goal()
        goal.request.text = blackboard["text"]
        return goal
