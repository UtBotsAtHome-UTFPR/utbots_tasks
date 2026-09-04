# vai para cada canto da sala - já mapeado rs 

# FALTANDO
# ver miro :)
# main rs 


import py_trees
from py_trees import decorators

from leaves.bt_basic_nav import SetInitialPose, GoToWaypointState, RotateInPlaceState, GetCurrentPoseState
from leaves.bt_take_pictures import TakePicturesState

def create_main_tree_clean_up(node) -> py_trees.behaviour.Behaviour:
#----------------------------------------------------------------------------------------
    analyze_room_sequence = py_trees.composites.Sequence(
            name="Subtree: listen and understand", 
            memory=True
    )

    initial_pose = SetInitialPose(
            name="1_Set_Initial_pose", 
            node=node
    )

    go_waypoint = GoToWaypointState(
            name="2_Go_To_Waypoint", 
            node=node
    )

    get_pose = GetCurrentPoseState(
            name="2_Get_Current_Pose", 
            node=node
        )

    # associar as fotos com o waypoint 
    take_pictures = TakePicturesState(
            name="3_Take_Pictures", 
            node=node
    )

    # colocar na main os graus
    rotate = RotateInPlaceState(
            name="3_Take_Pictures", 
            node=node
    )

    analyze_room_sequence.add_children([
            initial_pose,
            go_waypoint,
            get_pose,
            take_pictures,
            rotate,
            get_pose,
            take_pictures,
            rotate,
            get_pose,
            take_pictures,
            rotate,
            get_pose,
            take_pictures,
            rotate,
    ])

    # fila de blackboard na main 

    return