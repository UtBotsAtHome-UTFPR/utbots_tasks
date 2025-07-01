from yasmin import Blackboard
from yasmin_ros import ActionState
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from utbots_actions.action import YOLOBatchDetection
from std_msgs.msg import String, Int32, Float32

class FindObjectState(ActionState):
    def __init__(self) -> None:
        super().__init__(
            YOLOBatchDetection,  # action type
            "YOLO_batch_detection",  # action name
            self.create_goal_handler,  # callback to create the goal
            None,  # outcomes. Includes (SUCCEED, ABORT, CANCEL)
            self.response_handler,  # callback to process the response
            None,  # callback to process the feedback
        )

    def create_goal_handler(self, blackboard: Blackboard) -> YOLOBatchDetection.Goal:
        goal = YOLOBatchDetection.Goal()
        goal.target_category = String()
        goal.target_category.data = blackboard["beverage"]
        goal.batch_size = Int32()
        goal.batch_size.data = blackboard["batch_size"]
        goal.iou_threshold = Float32()
        goal.iou_threshold.data = blackboard["iou_threshold"]
        goal.support_threshold = Float32()
        goal.support_threshold.data = blackboard["support_threshold"]
        return goal

    def response_handler(self, blackboard: Blackboard, response: YOLOBatchDetection.Result) -> str:
        detections = response.detected_objs.bounding_boxes
        if len(detections) > 0:
            blackboard["detections"] = detections
            return SUCCEED
        else:
            return "not_detected"