import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
import yasmin
from yasmin import Blackboard, StateMachine, State
from yasmin_ros import ActionState, MonitorState, set_ros_loggers
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from yasmin_viewer import YasminViewerPub

from utbots_actions.action import YOLOBatchDetection, MPPose
from utbots_msgs.msg import BoundingBox
from std_msgs.msg import String, Int32, Float32
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Point, PointStamped

from utbots_tasks.states.basic_face import USBCamOff, USBCamOn, RecognitionState
from utbots_tasks.states.basic_mediapipe import GetPersonPointState

from cv_bridge import CvBridge
import cv2
import time
import numpy as np

custom_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    depth=10
)

class EstimateGraspPoint(MonitorState):
    """
    Class for estimating the grasp point in a point cloud from a 
    detections segmentation mask or bounding box if no mask available.
    """
    def __init__(self) -> None:
        super().__init__(Image, 
                         "/kinect2/sd/image_depth_rect",
                         [SUCCEED, ABORT], 
                         self.monitor_handler,
                         qos=custom_qos,
                         msg_queue=10, 
                         timeout=30)
        self.cvBridge = CvBridge()

    def get_depth_at(self, cv_image, x, y):
        if cv_image[int(y)][int(x)] > 0:
            return cv_image[int(y)][int(x)] / 1000  # millimeter to meter conversion
        return None
    
    def calculate_gaussian_depth(self, msg, xs, ys, center_x, center_y):
        cv_image = self.cvBridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        depths = []
        sigma = 0.1  # You may tune this value
        # Gaussian sum: weighted average where weights are Gaussian centered at (center_x, center_y)
        weights = []

        for x, y in zip(xs, ys):
            px = int(x)
            py = int(y)
            depth = self.get_depth_at(cv_image, px, py)
            print(depth)
            if depth:
                depths.append(depth)
                dx = x - center_x
                dy = y - center_y
                w = np.exp(-(dx**2 + dy**2) / (2 * sigma**2))
                weights.append(w)
        
        print("Depths and weights:")
        print(weights, depths)
        if depths:
            weights = np.array(weights)
            depths = np.array(depths)
            if weights.sum() > 0:
                grasp_depth = float(np.sum(depths * weights) / np.sum(weights))
            else:
                grasp_depth = self.get_depth_at(cv_image, center_x, center_y)
        else:
            grasp_depth = self.get_depth_at(cv_image, center_x, center_y)

        return grasp_depth

    def monitor_handler(self, blackboard: Blackboard, msg: Image) -> str:
        detections = blackboard["detections"][0]
        dimg_width = msg.width
        dimg_height = msg.height
        # Try to extract xs and ys from segmentation masks (image mask), fallback to detections.xyxyn if needed
        try:
            # If detections has a mask attribute (e.g., detections.mask is a numpy array or similar)
            if hasattr(detections, "mask") and detections.mask is not None:
            # Assume detections.mask is a binary mask (numpy array) with the same size as rgb_image
                mask = detections.mask
                mask = self.cvBridge.imgmsg_to_cv2(mask, desired_encoding="mono8")
                # Binarize mask: consider pixels > 0 as True
                mask = (mask > 0)
                if isinstance(mask, np.ndarray):
                    # Find nonzero (True) pixel coordinates
                    ys, xs = np.nonzero(mask)
                    xs = xs.tolist()
                    ys = ys.tolist()    
                    # Normalize mask coordinates to the depth image dimensions
                    mask_height, mask_width = mask.shape
                    xs = [x / mask_width * dimg_width for x in xs]
                    ys = [y / mask_height * dimg_height for y in ys]
                    # print(xs, ys)
                else:
                    raise AttributeError
            else:
                raise AttributeError
        except AttributeError:
            # Fallback: use normalized bounding box fields
            xs = []
            ys = []
            if hasattr(detections, "xminn") and hasattr(detections, "xmaxn") and hasattr(detections, "yminn") and hasattr(detections, "ymaxn"):
                
                # Convert normalized bbox to pixel coordinates
                xmin = int(detections.xminn * dimg_width)
                xmax = int(detections.xmaxn * dimg_width)
                ymin = int(detections.yminn * dimg_height)
                ymax = int(detections.ymaxn * dimg_height)
                # Fill xs and ys with all pixel coordinates inside the bbox
                xs = list(range(xmin, xmax))
                ys = list(range(ymin, ymax))

        if xs and ys:
            min_x = min(xs)
            max_x = max(xs)
            min_y = min(ys)
            max_y = max(ys)
            center_x = (min_x + max_x) / 2
            center_y = (min_y + max_y) / 2
            print("Object center at: ", center_x, center_y)

            grasp_depth = self.calculate_gaussian_depth(msg, xs, ys, center_x, center_y)

            if grasp_depth:
                point = Point()
                
                
                # Convert pixel coordinates (center_x, center_y) to camera-centered 3D meter coordinates 
                # using spherical to cartesian coordinates transformation. Grasp depth is already metric 
                # from the camera.
                
                fov_x = blackboard["fov_hor"]  # Horizontal FOV in degrees
                fov_y = blackboard["fov_ver"]  # Vertical FOV in degrees

                # Calculate angles theta (horizontal) and phi (vertical) from the optical axis
                x_max = dimg_width / 2.0
                y_max = dimg_height / 2.0
                theta_max = fov_x / 2.0
                phi_max = fov_y / 2.0

                x = center_x - x_max
                y = center_y - y_max

                theta = np.deg2rad(theta_max * x / x_max)
                phi = np.deg2rad(phi_max * y / y_max)

                rho = grasp_depth

                print("Spherical coordinates (rho, theta, phi): ")
                print(rho, theta, phi)

                # Spherical to Cartesian conversion
                Y = rho * np.sin(phi)
                X = np.sqrt(rho**2 - Y**2) * np.sin(theta)
                Z = X / np.tan(theta) if np.abs(theta) > 1e-6 else np.sqrt(rho**2 - X**2 - Y**2)

                # Remap coordinates
                # - Image coordinates: x right, y down, z forward
                # - ROS coordinates: x forward, y left, z up
                point.x = X#Z
                point.y = Y#-X
                point.z = Z#-Y

                blackboard["grasp_point"] = PointStamped()
                blackboard["grasp_point"].point = point
                blackboard["grasp_point"]._header = getattr(blackboard["grasp_point"], "_header", None)
                blackboard["grasp_point"].header = getattr(blackboard["grasp_point"], "header", None)
                blackboard["grasp_point"].header = getattr(blackboard["grasp_point"], "header", None) or type("Header", (), {})()
                blackboard["grasp_point"].header.frame_id = "kinect2_link"
            
            
            else:
                print("F")

                return ABORT
        else:
            print("G")

            return ABORT

        return SUCCEED

class FindObjectState(ActionState):
    """
    Class for searching and retrieving detections for one or more objects with several YOLO detections

    Attributes:
        action_server (str : "YOLO_batch_detection"): address of the YOLO action server
        verbose (bool : False): show debug information
    """
    def __init__(self, action_server="YOLO_batch_detection", verbose=False) ->None:
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
        blackboard["detections"] = detections if detections else BoundingBox()
        blackboard["annotated_img"] = response.annotated_image
        
        if self.verbose:
            yasmin.YASMIN_LOG_INFO(f"[DEBUG] Detections: ")
            for detection in detections:
                yasmin.YASMIN_LOG_INFO(f"[DEBUG] {detection}")

        yasmin.YASMIN_LOG_INFO(f"[DEBUG] {detections}")
        
        return SUCCEED if detections else "not_detected"

class FramePerson(State):
    def __init__(self) -> None:
        super().__init__([SUCCEED, ABORT])
        self.bridge = CvBridge()

    def execute(self, blackboard: Blackboard) -> str:
        # yasmin.YASMIN_LOG_INFO("Executing state FOO")
        people = blackboard["people"] # Face recognition output
        detections = blackboard["detections"] # Yolo output

        img = blackboard["annotated_img"]
        cv_image = self.bridge.imgmsg_to_cv2(img, desired_encoding="bgr8")

        face_bbox = None
        for person in people:
            if person.id != "Unknown":
                face_bbox = {}
                face_bbox["xmin"] = person.xmin
                face_bbox["xmax"] = person.xmax
                face_bbox["ymin"] = person.ymin
                face_bbox["ymax"] = person.ymax
                face_bbox["name"] = person.category
        
        if not face_bbox:
            return ABORT

        people_bbox = []
        for detection in detections:
            people_bbox.append({})

            people_bbox[-1]["xmin"] = detection.xmin
            people_bbox[-1]["xmax"] = detection.xmax
            people_bbox[-1]["ymin"] = detection.ymin
            people_bbox[-1]["ymax"] = detection.ymax

        face_height = face_bbox["ymax"] - face_bbox["ymin"]

        extracted_bbox = None
        max_intersect = 0
        for bbox in people_bbox:

            inter_x_min = max(face_bbox["xmin"], bbox["xmin"])
            inter_y_min = max(face_bbox["ymin"], bbox["ymin"])
            inter_x_max = min(face_bbox["xmax"], bbox["xmax"])
            inter_y_max = min(face_bbox["ymax"], bbox["ymin"] + face_height) # In case someone is behind a seated known person

            inter_width = max(0, inter_x_max - inter_x_min)
            inter_height = max(0, inter_y_max - inter_y_min)

            inter_area = inter_width * inter_height

            area1 = (face_bbox["xmax"] - face_bbox["xmin"]) * (face_bbox["ymax"] - face_bbox["ymin"])
            area2 = (bbox["xmax"] - bbox["xmin"]) * (bbox["ymax"] - bbox["ymin"])

            union_area = area1 + area2 - inter_area

            # Compute IoU
            if union_area == 0:
                return 0.0
            if (inter_area / union_area) > max_intersect:
                print("Entering")
                max_intersect = inter_area / union_area
                extracted_bbox = bbox

        h, w, a = cv_image.shape

        mask = np.zeros((h, w), dtype=np.uint8)

        mask[extracted_bbox["ymin"]:extracted_bbox["ymax"], extracted_bbox["xmin"]:extracted_bbox["xmax"]] = 255

        mask_img = cv2.bitwise_and(cv_image, cv_image, mask=mask)
        
        identified_image = self.bridge.cv2_to_imgmsg(mask_img, encoding="bgr8")

        blackboard["cropped_person"] = identified_image
        # TODO: passar essa imagem pro mediapipe estimar a pose da pessoa
        # cv2.imshow("img", mask_img)
        # cv2.waitKey(0)

        return SUCCEED

class GetPersonPositionState(MonitorState):
    def __init__(self) -> None:
        super().__init__(Image, 
                         "/kinect2/sd/image_depth_rect", 
                         [SUCCEED, ABORT], 
                         self.monitor_handler,
                         qos=custom_qos,
                         msg_queue=10, 
                         timeout=30)
        self.cvBridge = CvBridge()
        
    def monitor_handler(self, blackboard: Blackboard, msg: Image) -> str:

        cv_image = self.cvBridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        # Determine the pixels for the skeleton positions 
        person_points = blackboard["mediapipe_points_normalized"]

        valid_positions = []
        for point in person_points.points:
            # If point is at the image (mediapipe creates estimated points outside)
            if point.x > 0 and point.x < 1 and point.y > 0 and point.y < 1:
                point.x = point.x * msg.width
                point.y = point.y * msg.height
                
                # Kinect depth sensor has less width than rgb and it's filled with 0
                if cv_image[int(point.x)][int(point.y)] > 0:
                    point.z = cv_image[int(point.x)][int(point.y)] / 1000 # millimiter to meter conversion
                    valid_positions.append(point)
        
        if not valid_positions:
            return ABORT
        
        distance = 0
        for point in valid_positions:
            distance += point.z
        distance /= len(valid_positions)

        print(f"Estimated distance is: {distance}")
        time.sleep(1)

        # Convert distance to x/y/z coordinates (y doesn't matter but is needed for estimation)

        return SUCCEED

def locate_person_from_face():
    yasmin.YASMIN_LOG_INFO("locate_person_from_face_demo")
    rclpy.init()
    
    node = rclpy.create_node("locate_person_from_face_sm")

     # Set up ROS 2 logs
    set_ros_loggers()

    sm = StateMachine(outcomes=[SUCCEED, CANCEL, ABORT])

    sm.add_state(
        "RECOGNIZE",
        RecognitionState(),
        transitions={
            SUCCEED: "FIND_PEOPLE",
            ABORT: ABORT,
        }
    )

    sm.add_state(
        "FIND_PEOPLE",
        FindObjectState(action_server="/YOLO_batch_detection", verbose=True),
        transitions={
            SUCCEED: "FRAME_PERSON",
            'not_detected': ABORT,
            CANCEL: CANCEL,
            ABORT: ABORT
        }
    )

    sm.add_state(
        "FRAME_PERSON",
        FramePerson(),
        transitions={
            SUCCEED:"TRACK_PERSON_FROM_CROPPED",
            ABORT:ABORT
        }
    )

    sm.add_state(
        "TRACK_PERSON_FROM_CROPPED",
        GetPersonPointState(),
        transitions={
            SUCCEED:"TRACK_PERSON",
            ABORT:ABORT
        },
        remappings = {"mediapipe_img" : "cropped_person"}
    )

    # Estado track person com a imagem não cropada pras partes que não terão bounding box
    sm.add_state(
        "TRACK_PERSON",
        GetPersonPointState(),
        transitions={
            SUCCEED:"GET_PERSON_POSE_FROM_TORSO",
            ABORT:ABORT
        },
    )

    # Estado monitor de estimar a posição da pessoa a partir da posição da posição do torso dela
    sm.add_state(
        "GET_PERSON_POSE_FROM_TORSO",
        GetPersonPositionState(),
        transitions={
            SUCCEED:"TRACK_PERSON",
            ABORT:ABORT
        },
    )

    blackboard = Blackboard()

    # Face recognition variables
    blackboard["person_name"] = "Teste"
    blackboard['objects'] = ['person']

    # Yolo variables
    blackboard["batch_size"] = 10
    blackboard["iou_threshold"] = 0.5
    blackboard["support_threshold"] = 0.6

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
    
def find_seat_sm():
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

def get_object_point():
    yasmin.YASMIN_LOG_INFO("get_object_point_demo")
    rclpy.init()
    
    node = rclpy.create_node("get_object_point_sm")
    grasp_point_pub = node.create_publisher(PointStamped, "grasp_point", 10)

     # Set up ROS 2 logs
    set_ros_loggers()

    sm = StateMachine(outcomes=[SUCCEED, CANCEL, ABORT])

    sm.add_state(
        "FIND_OBJECT",
        FindObjectState(action_server="/YOLO_batch_detection", verbose=False),
        transitions={
            SUCCEED: "ESTIMATE_GRASP_POINT",
            'not_detected': ABORT,
            CANCEL: CANCEL,
            ABORT: ABORT
        },
        remappings={"objects": "object", "detections": "bboxes"},
    )

    sm.add_state(
        "ESTIMATE_GRASP_POINT",
        EstimateGraspPoint(),
        transitions={
            SUCCEED: SUCCEED,
            ABORT: ABORT
        }
    )

    blackboard = Blackboard()

    # Yolo variables
    blackboard["object"] = "person"
    blackboard["batch_size"] = 10
    blackboard["iou_threshold"] = 0.5
    blackboard["support_threshold"] = 0.6
    blackboard["fov_hor"] = 70.0  # Horizontal FOV of the depth camera in degrees
    blackboard["fov_ver"] = 60.0  # Vertical FOV of

    # Publish FSM information
    YasminViewerPub("YASMIN_ACTION_CLIENT_DEMO", sm)

    try:
        outcome = sm(blackboard)
        yasmin.YASMIN_LOG_INFO(outcome)
        if outcome == SUCCEED:
            point = blackboard["grasp_point"]
            yasmin.YASMIN_LOG_INFO(f"Grasp point: x={point.point.x}, y={point.point.y}, z={point.point.z}")
            # Publish the grasp point to a ROS 2 topic
            grasp_point_pub.publish(point)
            yasmin.YASMIN_LOG_INFO("Published grasp point to 'grasp_point' topic.")
    except KeyboardInterrupt:
        if sm.is_running():
            sm.cancel_state()  # Cancel the state if interrupted

    # Shutdown ROS
    if rclpy.ok():
        rclpy.shutdown()

def main():
    #find_seat_sm()
    # locate_person_from_face()
    get_object_point()

if __name__ == "__main__":
    main()

