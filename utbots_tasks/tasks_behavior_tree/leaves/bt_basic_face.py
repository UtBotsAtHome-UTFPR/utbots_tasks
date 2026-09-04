import py_trees
import py_trees_ros.action_clients
import py_trees_ros.service_clients
from std_srvs.srv import SetBool
import math

from utbots_actions.action import NewFace, Recognition


class IdentifyYAW(py_trees.behaviour.Behaviour):
    def __init__(self, name: str = "IdentifyYAW"):
        super().__init__(name=name)
        
        self.blackboard = self.attach_blackboard_client(name=self.name)
        
        self.blackboard.register_key("people", access=py_trees.common.Access.READ)
        self.blackboard.register_key("people_yaw", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("rotate", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("rotation_count", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("people_count", access=py_trees.common.Access.WRITE)

    def update(self) -> py_trees.common.Status:
        
        # If there are no people or key doesnt exist, return FAILURE
        if not self.blackboard.exists("people") or len(self.blackboard.people) == 0:
            self.logger.info("No people for whatever reason")
            return py_trees.common.Status.FAILURE 

        if not self.blackboard.exists("people_yaw"):
            self.logger.info("Create positions to stare at during conversation")
            self.blackboard.people_yaw = []
            
        if not self.blackboard.exists("people_count"):
            self.blackboard.people_count = 0
            
        if not self.blackboard.exists("rotation_count"):
            self.blackboard.rotation_count = 0

        people = self.blackboard.people

        width = 1280.0  # CHANGE TO 1920
        theta_max = 78.0 / 2.0  # Logitech cam FOV 

        for person in people:
            self.logger.info(f"Detected category: {person.category}")
            
            # Skips unknown individuals or people who have already been mapped.
            if person.category == "Unknown" or person.category in self.blackboard.people_yaw:
                continue

            # Calculates the center of the person's bounding box
            x = int((person.xmin + person.xmax) / 2)

            # Centers the X-axis
            x = x - int(width / 2.0)
            
            # Calculates the required degree of rotation
            theta = math.radians(theta_max * x / (width / 2.0))
            angle = math.degrees(theta)
            
            self.logger.info(f"Calculated Angle: {-angle}")

            # Updates blackboard
            self.blackboard.people_yaw.append(person.category)
            self.blackboard.rotate = -angle
            self.blackboard.rotation_count = 0
            self.blackboard.people_count += 1
            
            # Return success
            return py_trees.common.Status.SUCCESS

        # If the loop finishes and no new/valid person is found, the robot searches by rotating 45°.
        self.blackboard.rotation_count += 45
        self.blackboard.rotate = 45.0
        
        return py_trees.common.Status.FAILURE


class CheckContinuation(py_trees.behaviour.Behaviour):
    def __init__(self, name: str = "CheckContinuation"):
        super().__init__(name=name)
        
        self.blackboard = self.attach_blackboard_client(name=self.name)
        
        self.blackboard.register_key("people_count", access=py_trees.common.Access.READ)
        self.blackboard.register_key("rotation_count", access=py_trees.common.Access.READ)

    def update(self) -> py_trees.common.Status:
        
        # Verifies if keys exist
        if not self.blackboard.exists("people_count") or not self.blackboard.exists("rotation_count"):
            self.logger.warning("The 'people_count' or 'rotation_count' keys do not yet exist in the Blackboard.")
            return py_trees.common.Status.FAILURE

        # Verification logic
        if self.blackboard.people_count >= 2 or self.blackboard.rotation_count >= 360:
            return py_trees.common.Status.SUCCESS
            
        return py_trees.common.Status.FAILURE


class USBCamOn(py_trees_ros.service_clients.FromBlackboard):
    def __init__(self, name: str = "USBCamOn"):
        super().__init__(
            name,
            SetBool,
            "/set_capture",
            "cam_request"
        )

        self.blackboard.register_key("cam_request", access=py_trees.common.Access.WRITE)

    def initialise(self):
        request = SetBool.Request()
        request.data = True
        self.blackboard.set("cam_request", request)
        super().initialise()


class USBCamOff(py_trees_ros.service_clients.FromBlackboard):
    def __init__(self, name: str = "USBCamOff"):
        super().__init__(
            name,
            SetBool,
            "/set_capture",
            "request_off" 
        )
        
        self.blackboard.register_key("request_off", access=py_trees.common.Access.WRITE)

    def initialise(self):
        request = SetBool.Request()
        request.data = False
        self.blackboard.set("request_off", request)
        super().initialise()


class RecognitionState(py_trees_ros.action_clients.FromBlackboard):
    def __init__(self, name: str = "RecognitionState"):
        super().__init__(
            name=name,
            action_type=Recognition, 
            action_name="/recognition",
            action_goal_variable="recognition_goal"
        )
        
        self.blackboard.register_key("img", access=py_trees.common.Access.READ)
        self.blackboard.register_key("people", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("recognized_img", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("recognition_goal", access=py_trees.common.Access.WRITE)

    def initialise(self):

        goal = Recognition.Goal()

        if self.blackboard.exists("img"):
            goal.image = self.blackboard.img
        else:
            self.logger.warning(f"[{self.name}] 'img' key not found. Sending meta without image.")

        self.blackboard.set("recognition_goal", goal)

        super().initialise()

    def update(self) -> py_trees.common.Status:

        status = super().update()

        # If action returned success, we process the answer
        if status == py_trees.common.Status.SUCCESS:
            
            result = self.result_message

            if not isinstance(result.people, list):
                self.logger.error(f"[{self.name}] The format of 'people' is invalid (it is not a list).")
                return py_trees.common.Status.FAILURE

            self.blackboard.people = result.people
            self.blackboard.recognized_img = result.image
            
            self.logger.info(f"[{self.name}] Recognized individuals: {self.blackboard.people}")
            return py_trees.common.Status.SUCCESS

        return status


class NewFaceState(py_trees_ros.action_clients.FromBlackboard):
    def __init__(self, name: str = "NewFaceState"):
        super().__init__(
            name,
            NewFace,
            "/new_face",
            "new_face_goal" 
        )
        
        self.blackboard.register_key("n_pics", access=py_trees.common.Access.READ)
        self.blackboard.register_key("operator", access=py_trees.common.Access.READ)
        
        self.blackboard.register_key("new_face_goal", access=py_trees.common.Access.WRITE)

    def initialise(self):
        goal = NewFace.Goal()

        if self.blackboard.exists("n_pics"):
            goal.n_pictures.data = self.blackboard.n_pics
        else:
            goal.n_pictures.data = 5
            
        if self.blackboard.exists("operator"):
            goal.name.data = str(self.blackboard.operator)
        else:
            goal.name.data = "Operator"

        self.blackboard.set("new_face_goal", goal)

        super().initialise()

    def update(self) -> py_trees.common.Status:
        status = super().update()

        if status == py_trees.common.Status.SUCCESS:
            self.logger.info(f"[{self.name}] New face successfully registered!")
            
            return py_trees.common.Status.SUCCESS

        return status




