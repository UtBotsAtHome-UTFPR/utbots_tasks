# door - gpsr e laundry

# espera a porta abrir e vai para um destino

import py_trees

from leaves.bt_basic_nav import SetInitialPose, WaitDoorOpenState, GoToWaypointState

def create_wait_and_go_subtree() -> py_trees.behaviour.Behaviour:
    
    subtree = py_trees.composites.Sequence(
        name="Subtree: Wait Door Open and Go To Waypoint", 
        memory=True
    )

    set_initial_pose_leaf = SetInitialPose(
        name="Set_Pose"
    )

    wait_door_leaf = WaitDoorOpenState(
        name="Wait_Door",
        timeout_sec=30.0  
    )
    
    go_to_waypoint_leaf = GoToWaypointState(
        name="Go_To_Waypoint"
    )

    subtree.add_children([set_initial_pose_leaf, wait_door_leaf, go_to_waypoint_leaf])

    return subtree