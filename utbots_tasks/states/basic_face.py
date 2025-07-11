import rclpy

import yasmin
from yasmin import CbState, Blackboard, StateMachine
from yasmin_ros import ActionState
from yasmin_ros import set_ros_loggers
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from yasmin_viewer import YasminViewerPub
from utbots_actions.action import NewFace, Recognition

class RecognitionState(ActionState):

    def __init__(self) -> None:
        super().__init__(
            Recognition,  # action type
            "/recognition",  # action name
            self.create_goal_handler,  # callback to create the goal
            None,  # outcomes. Includes (SUCCEED, ABORT, CANCEL)
            self.response_handler,  # callback to process the response
            None,  # callback to process the feedback
        )

    def create_goal_handler(self, blackboard: Blackboard) -> Recognition.Goal:
        goal = Recognition.Goal()

        if "img" in blackboard:
            goal.image = blackboard["img"]

        return goal

    def response_handler(self, blackboard: Blackboard, response: Recognition.Result) -> str:
        if type(response.people) is not list:
            return CANCEL

        blackboard["people"] = (response.people)
        blackboard["recognized_img"] = (response.image)
        print(blackboard["people"])
        # print(blackboard['recognized_img'])
        return SUCCEED
        '''self._node.get_logger().info("Here")
        if goal_status == 4:  # SUCCEEDED
            if "people" in blackboard:
                blackboard["people"] = (response.people)
                blackboard["recognized_img"] = (response.image)
            return SUCCEED
        elif goal_status == 5:  # CANCELED
            return CANCEL
        elif goal_status == 6:  # ABORTED
            return ABORT
        else:
            return ABORT  # fallback for unknown/error'''
        

class NewFaceState(ActionState):

    def __init__(self) -> None:
        super().__init__(
            NewFace,  # action type
            "/new_face",  # action name
            self.create_goal_handler,  # callback to create the goal
            None,  # outcomes. Includes (SUCCEED, ABORT, CANCEL)
            self.response_handler,  # callback to process the response
            None,  # callback to process the feedback
        )

    def create_goal_handler(self, blackboard: Blackboard) -> NewFace.Goal:
        goal = NewFace.Goal()

        if "n_pics" in blackboard:
            goal.n_pictures.data = blackboard["n_pics"]
        else:
            goal.n_pictures.data = 5
        
        if "operator" in blackboard:
            goal.name.data = str(blackboard["operator"])
        else:
            goal.name.data = "Operator"

        return goal

    def response_handler(self, blackboard: Blackboard, response: NewFace.Result) -> str:

        return SUCCEED

def main():
    """
    Main function to execute the ROS 2 action client demo.

    This function initializes the ROS 2 client, sets up the finite state
    machine, adds the states, and starts the action processing.

    Parameters:
        None

    Returns:
        None

    Raises:
        KeyboardInterrupt: If the user interrupts the execution.
    """
    yasmin.YASMIN_LOG_INFO("yasmin_action_client_demo")

    # Initialize ROS 2
    rclpy.init()

    # Set up ROS 2 logs
    set_ros_loggers()

    # Create a finite state machine (FSM)
    sm = StateMachine(outcomes=[SUCCEED, CANCEL])

    sm.add_state(
        "CALLING_NEW_FACE",
        NewFaceState(),
        transitions={
            SUCCEED: "CALLING_RECOGNITION", # All mapping to SUCCEED for now
            CANCEL: CANCEL,
            ABORT: SUCCEED,
        },
    )

    # Add states to the FSM
    sm.add_state(
        "CALLING_RECOGNITION",
        RecognitionState(),
        transitions={
            SUCCEED: SUCCEED, # All mapping to SUCCEED for now
            CANCEL: CANCEL,
            ABORT: SUCCEED,
        },
    )

    # Publish FSM information
    YasminViewerPub("YASMIN_ACTION_CLIENT_DEMO", sm)

    # Create an initial blackboard with the input value
    blackboard = Blackboard()
    #blackboard["n"] = 10  # Set the Fibonacci order to 10

    # Execute the FSM
    try:
        outcome = sm(blackboard)
        yasmin.YASMIN_LOG_INFO(outcome)
    except KeyboardInterrupt:
        if sm.is_running():
            sm.cancel_state()  # Cancel the state if interrupted

    # Shutdown ROS 2
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()