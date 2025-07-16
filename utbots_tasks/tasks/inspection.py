import rclpy
#Monkey path (TODO:change)
import numpy as np
if not hasattr(np, 'float'):
    np.float = float
import yasmin
from yasmin import Blackboard, StateMachine
from yasmin_ros import set_ros_loggers
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from yasmin_viewer import YasminViewerPub

from utbots_tasks.states.basic_nav import GoToWaypointState, WaitDoorOpenState, SetInitialPose

def main():
    yasmin.YASMIN_LOG_INFO("yasmin_action_client")
    rclpy.init()
    node = rclpy.create_node("inspection_sm")

    # Set up ROS 2 logs
    set_ros_loggers()

    # Create a finite state machine (FSM)
    sm = StateMachine(outcomes=["success", "failed"])

    sm.add_state(
        "SET_INIT_POSE",
        SetInitialPose(node, 0.0, 0.0, 0.0),
        transitions={
            SUCCEED: "WAIT_DOOR",
            ABORT: "failed"
        }
    )

    sm.add_state(
        "WAIT_DOOR",
        WaitDoorOpenState(),
        transitions={
            SUCCEED: "GO_TO_WAYPOINT1",
            CANCEL: "WAIT_DOOR",
            ABORT: "failed"
        }
    )

    sm.add_state(
        "GO_TO_WAYPOINT1",
        GoToWaypointState(),
        transitions={
            SUCCEED: "GO_TO_WAYPOINT2",
            ABORT: "failed"
        }
    )

    sm.add_state(
        "GO_TO_WAYPOINT2",
        GoToWaypointState(),
        transitions={
            SUCCEED: "success",
            ABORT: "failed"
        },
        remappings={"waypoint_nametag":"waypoint_exit_door"}
    )

    # Publish FSM information
    YasminViewerPub("YASMIN_ACTION_CLIENT_DEMO", sm)

    blackboard = Blackboard()
    blackboard["waypoint_nametag"] = "entrance"
    blackboard["waypoint_exit_door"] = "inspection"
    blackboard['yaml_path'] = '/home/laser/ros2_ws/src/utbots_navigation/utbots_nav/map/arena_filled_waypoints.yaml'

    try:
        outcome = sm(blackboard)
        yasmin.YASMIN_LOG_INFO(outcome)
    except KeyboardInterrupt:
        if sm.is_running():
            sm.cancel_state()  # Cancel the state if interrupted

    # Shutdown ROS
    if rclpy.ok():
        rclpy.shutdown()

