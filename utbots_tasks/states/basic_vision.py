import rclpy
import yasmin
from yasmin import Blackboard, StateMachine
from yasmin_ros import ActionState, set_ros_loggers
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from utbots_actions.action import YOLOBatchDetection
from std_msgs.msg import String, Int32, Float32
from yasmin_viewer import YasminViewerPub

class FindObjectState(ActionState):
    """
    Class for searching and retrieving detections for one or more objects with several YOLO detections

    Attributes:
        action_server (str : "YOLO_batch_detection"): address of the YOLO action server
        verbose (bool : False): show debug information
    """
    def __init__(self, action_server="YOLO_batch_detection", verbose=False,) ->None:
        super().__init__(
            YOLOBatchDetection,
            action_server,
            self.create_goal_handler,
            [SUCCEED, 'not_detected', CANCEL, ABORT],
            self.response_handler,
            None,
        )
        self.verbose = verbose

    def create_goal_handler(self, blackboard: Blackboard) -> YOLOBatchDetection.Goal:
        goal = YOLOBatchDetection.Goal()
        objects = blackboard["objects"]

        if self.verbose:
            yasmin.YASMIN_LOG_INFO(f"[DEBUG] Target objects: {objects}")
        
        if isinstance(objects, str):
            goal.target_categories = [String(data=objects)]
        elif isinstance(objects, list):
            goal.target_categories = [String(data=obj) for obj in objects]
        else:
            raise ValueError("Expected 'objects' to be a string or a list.")
        
        goal.batch_size = Int32()
        goal.batch_size.data = blackboard["batch_size"]
        goal.iou_threshold = Float32()
        goal.iou_threshold.data = blackboard["iou_threshold"]
        goal.support_threshold = Float32()
        goal.support_threshold.data = blackboard["support_threshold"]
        
        if self.verbose:
            yasmin.YASMIN_LOG_INFO(f"[DEBUG] Goal message: {goal}")

        return goal

    def response_handler(self, blackboard: Blackboard, response: YOLOBatchDetection.Result) -> str:
        detections = response.detected_objs.bounding_boxes
        blackboard["annotated_img"] = response.annotated_image
        blackboard["detections"] = detections if detections else []
        
        if self.verbose:
            yasmin.YASMIN_LOG_INFO(f"[DEBUG] Detections: ")
            for detection in detections:
                yasmin.YASMIN_LOG_INFO(f"[DEBUG] {detection}")
        
        return SUCCEED if detections else "not_detected"
    
def main():
    yasmin.YASMIN_LOG_INFO("yasmin_action_client_demo")
    rclpy.init()
    node = rclpy.create_node("receptionist_sm")

     # Set up ROS 2 logs
    set_ros_loggers()

    sm = StateMachine(outcomes=[SUCCEED, CANCEL, ABORT])
    sm.add_state(
        "FIND_PEOPLE",
        FindObjectState(action_server="/yolo_node_coco/YOLO_batch_detection", verbose=True),
        transitions={
            SUCCEED: "FIND_SEAT",
            'not_detected': "FIND_SEAT",
            CANCEL: CANCEL,
            ABORT: ABORT
        },
        remappings={"objects": "person", "detections": "bboxes2"},
    )
    sm.add_state(
        "FIND_SEAT",
        FindObjectState(action_server="/yolo_node_coco/YOLO_batch_detection", verbose=True),
        transitions={
            SUCCEED: "FIND_DRINKS",
            'not_detected': "FIND_DRINKS",
            CANCEL: CANCEL,
            ABORT: ABORT
        },
        remappings={"objects": "seat", "detections": "bboxes1"},
    )
    sm.add_state(
        "FIND_DRINKS",
        FindObjectState(action_server="/yolo_node_home/YOLO_batch_detection", verbose=True),
        transitions={
            SUCCEED: SUCCEED,
            'not_detected': SUCCEED,
            CANCEL: CANCEL,
            ABORT: ABORT
        },
        remappings={"objects": "drinks", "detections": "bboxes1"}, 
    )

    blackboard = Blackboard()
    blackboard["iou_threshold"] = 0.5
    blackboard["support_threshold"] = 0.4
    blackboard["batch_size"] = 50
    blackboard['person'] = ['person']
    blackboard['seat'] = ['chair', 'couch']
    blackboard['drinks'] = ['drinks-drinks-cofee', 'drinks-coke', 'drinks-fanta', 'drinks-kuat', 'drinks-milk', 'drinks-orange_juice']

    # Publish FSM information
    YasminViewerPub("YASMIN_ACTION_CLIENT_DEMO", sm)

    try:
        outcome = sm(blackboard)
        yasmin.YASMIN_LOG_INFO(outcome)
    except KeyboardInterrupt:
        if sm.is_running():
            sm.cancel_state()  # Cancel the state if interrupted

    # Shutdown ROS
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == "__main__":
    main()

