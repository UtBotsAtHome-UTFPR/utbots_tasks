
# time: 7 minutes max + 1 extra minute REVER
# pausar o timer toda vez que voltar o instruction point e recomeçar quando receber nova instrução?

# outside the arena - when the door opens, navigate to instruction point inside arena: 
# subtree "wait_door_and_go.py"

# 3 tentativas para entender o comando - um por um 

import py_trees

from subtrees.listen_and_repeat import create_ask_listen_and_repeat_subtree
from subtrees.wait_door_and_go import create_wait_and_go_subtree
 

def create_main_tree(node) -> py_trees.behaviour.Behaviour:
    

    wait_and_go_subtree = create_wait_and_go_subtree(node)
    listen_repeat_subtree = create_ask_listen_and_repeat_subtree(node)
    

    retry_listen = py_trees.decorators.Retry(
        name="Retry_Listen_Understand_3x",
        child=listen_repeat_subtree,
        num_retries=2  
    )

    
    main_sequence = py_trees.composites.Sequence(
        name="Main_Sequence",
        memory=True
    )
    main_sequence.add_children([wait_and_go_subtree, retry_listen])
    

    root_with_timeout = py_trees.decorators.Timeout(
        name="Global_Timeout_7_Min",
        child=main_sequence,
        duration=420.0  
    )
    
    return root_with_timeout


