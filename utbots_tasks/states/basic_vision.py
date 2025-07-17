import yasmin
from yasmin import Blackboard
from yasmin_ros import ActionState
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from utbots_actions.action import YOLOBatchDetection
from std_msgs.msg import String, Int32, Float32
from utbots_msgs.msg import BoundingBoxes

class FindObjectState(ActionState):
    def __init__(self, remappings: dict = None, action_server="YOLO_batch_detection") ->None:
        super().__init__(
            YOLOBatchDetection,
            action_server,
            self.create_goal_handler,
            None,
            self.response_handler,
            None,
        )
        self.remappings = remappings or {}

    def _remap(self, key: str) -> str:
        return self.remappings.get(key, key)

    def create_goal_handler(self, blackboard: Blackboard) -> YOLOBatchDetection.Goal:
        goal = YOLOBatchDetection.Goal()
        objects = blackboard[self._remap("objects")]
        if isinstance(objects, str):
            goal.target_categories = [String(data=objects)]
            yasmin.YASMIN_LOG_INFO(objects)
        elif isinstance(objects, list):
            yasmin.YASMIN_LOG_INFO(f"Target objects: {objects}")
            goal.target_categories = [String(data=obj) for obj in objects]
        else:
            raise ValueError("Expected 'objects' to be a string or a list.")
        goal.batch_size = Int32()
        goal.batch_size.data = blackboard["batch_size"]
        goal.iou_threshold = Float32()
        goal.iou_threshold.data = blackboard["iou_threshold"]
        goal.support_threshold = Float32()
        goal.support_threshold.data = blackboard["support_threshold"]
        return goal

    def response_handler(self, blackboard: Blackboard, response: YOLOBatchDetection.Result) -> str:
        detections = response.detected_objs.bounding_boxes
        yasmin.YASMIN_LOG_INFO(f"{detections}")
        target_key = self._remap("detections")
        blackboard[target_key] = detections if detections else []
        return SUCCEED if detections else CANCEL