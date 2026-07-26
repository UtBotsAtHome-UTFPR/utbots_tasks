import py_trees
import rclpy

from leaves.bt_basic_voice import NLUInference, WhisperSTT

def create_listen_and_understand_subtree(node) -> py_trees.behaviour.Behaviour:
    """
    Creates a sub-tree that first transcribes audio (Whisper STT)
    and then interprets the intent/data (NLU Inference).
    """
    subtree = py_trees.composites.Sequence(
        name="Subtree: Listen and Understand", memory=True
    )

    stt_leaf = WhisperSTT(name="1_Whisper_STT", node=node)

    nlu_leaf = NLUInference(name="2_NLU_Inference", node=node, verbose=True)

    subtree.add_children([stt_leaf, nlu_leaf])

    return subtree