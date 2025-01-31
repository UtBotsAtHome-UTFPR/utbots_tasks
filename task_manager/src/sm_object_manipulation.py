#!/usr/bin/env python3

import rospy
import smach
import smach_ros
import actionlib
import rospkg
import yaml
from cv_bridge import CvBridge
import cv2
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from utbots_actions.msg import YOLODetectionAction, YOLODetectionGoal, Extract3DPointAction
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from smach_ros import SimpleActionState
from reportlab.lib.pagesizes import A4
from reportlab.pdfgen import canvas
from reportlab.lib.units import inch
import numpy as np


global bboxes
bboxes = None

# Initialize CvBridge to convert ROS image message to OpenCV image
bridge = CvBridge()

# Initialize the rospkg instance
rospack = rospkg.RosPack()

# Get the path to the desired package (replace 'your_package' with the actual package name)
package_path = rospack.get_path('task_manager')

def retrieve_waypoint(nametag):
    # Open and read the YAML file
    with open(f"{package_path}/waypoints.yaml", 'r') as file:
        data = yaml.safe_load(file)

    # Find the pose with the given nametag
    for pose_data in data['poses']:
        if pose_data['nametag'] == nametag:
            # Create a PoseStamped message
            pose_stamped = PoseStamped()

            # Fill in the header
            pose_stamped.header.seq = pose_data['header']['seq']
            pose_stamped.header.stamp.secs = pose_data['header']['stamp']['secs']
            pose_stamped.header.stamp.nsecs = pose_data['header']['stamp']['nsecs']
            pose_stamped.header.frame_id = pose_data['header']['frame_id']

            # Fill in the pose (position and orientation)
            pose_stamped.pose.position.x = pose_data['pose']['position']['x']
            pose_stamped.pose.position.y = pose_data['pose']['position']['y']
            pose_stamped.pose.position.z = pose_data['pose']['position']['z']

            pose_stamped.pose.orientation.x = pose_data['pose']['orientation']['x']
            pose_stamped.pose.orientation.y = pose_data['pose']['orientation']['y']
            pose_stamped.pose.orientation.z = pose_data['pose']['orientation']['z']
            pose_stamped.pose.orientation.w = pose_data['pose']['orientation']['w']

            return pose_stamped

# This state will send a nav goal to the place the objects are located
class go_to_shelf(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['succeeded', 'aborted'],
                            input_keys=['waypoint'])
        self.result = "" 
        self.pub = rospy.Publisher('/bridge_navigate_to_pose/goal', PoseStamped, queue_size=1)

    def execute(self, userdata):

        rospy.loginfo('Executing state go_to_shelf')

        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.client.wait_for_server()


        try:
            # Sleep briefly to ensure the message is sent
            rospy.sleep(2)
            rospy.loginfo("Publish waypoint")
            
            odom_msg = retrieve_waypoint("shelf")

            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()

            goal.target_pose.pose.position.x = odom_msg.pose.position.x
            goal.target_pose.pose.position.y = odom_msg.pose.position.y
            goal.target_pose.pose.position.z = 0.0

            quaternion = np.array([odom_msg.pose.orientation.w, odom_msg.pose.orientation.x, odom_msg.pose.orientation.y, odom_msg.pose.orientation.z])

            goal.target_pose.pose.orientation.x = quaternion[1]
            goal.target_pose.pose.orientation.y = quaternion[2]
            goal.target_pose.pose.orientation.z = quaternion[3]
            goal.target_pose.pose.orientation.w = quaternion[0]

            self.client.send_goal(goal)

            finished = self.client.wait_for_result()

            if not finished: rospy.logerr("Action server not available")
            else:
                rospy.loginfo(self.client.get_result())

            if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                return 'succeeded'
            return 'aborted'
        
        except rospy.ROSInterruptException:
            return 'aborted'

        
# This state will save the labeled image with objects detected by YOLOv8
class detection_log(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['log_saved', 'aborted'],
                            input_keys=['labeled_img', 'bboxes'])        

    def execute(self, userdata):
        
        rospy.loginfo('Executing state get_objects_bbox')

        try:
            global bboxes

            log_img = userdata.labeled_img
            bboxes = userdata.bboxes.bounding_boxes

            # Convert the ROS Image message to an OpenCV image
            cv_image = bridge.imgmsg_to_cv2(log_img)

            # Save the image as a temporary file (as reportlab requires an image file)
            image_filename = "/tmp/log_img.jpg"
            cv2.imwrite(image_filename, cv_image)

            current_time = rospy.Time.now()

            # Create a new PDF file
            pdf_file = f"/home/laser/catkin_ws/src/utbots_tasks/task_manager/logs/object_recognition_manipulation_log_{current_time}.pdf"
            c = canvas.Canvas(pdf_file, pagesize=A4)

            # Insert an image (you can adjust the image size by scaling)
            image_path = "/tmp/log_img.jpg"
            c.drawImage(image_path, x=85, y=440, width=6*inch, height=4.5*inch)

            # Add some text
            c.setFont("Helvetica", 12)

            text_height = 420
            c.drawString(100, text_height, "Objects recognized:")
            text_height -= 15

            for bbox in bboxes:
                text = f"- {bbox.Class}"
                c.drawString(100, text_height, text)
                text_height -= 12

            # Finalize the PDF file
            c.showPage()
            c.save()
            
            return 'log_saved'
        
        except rospy.ROSInterruptException:
            return 'aborted'
        
# This state selects the object for manipulation
class select_object(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['get_object_point', 'all_objects', 'aborted'],
                            output_keys=['selected_bbox'])

    def execute(self, userdata):

        rospy.loginfo('Executing state select_object')

        global bboxes

        if bboxes == []:
            return 'all_objects'

        try:
            userdata.selected_bbox = bboxes[0]
            bboxes = bboxes[1:]
            return 'get_object_point'
        
        except rospy.ROSInterruptException:
            return 'aborted'
        
# This state exctracts the 3D point for a given object
class get_object_point(smach_ros.SimpleActionState):
    def __init__(self):
        smach_ros.SimpleActionState.__init__(self, 'extract_3d_point', Extract3DPointAction, 
                                             goal_cb=self.goal_callback, 
                                             result_slots=['Point', 'Success'],
                                             input_keys=['selected_bbox'],
                                             output_keys=['Point'])

    def goal_callback(self, userdata, goal):
        # Use the goal_data from previous state to set the goal for the action
        goal.Object_bbox = userdata.selected_bbox
        return goal

def main():

    rospy.init_node('sm_object_manipulation', anonymous=True)

    client_yolo = actionlib.SimpleActionClient('YOLO_detection', YOLODetectionAction)
    client_yolo.wait_for_server()

    rospy.loginfo("[TASK] YOLO Detection server up")

    client_ext = actionlib.SimpleActionClient('extract_3d_point', Extract3DPointAction)
    client_ext.wait_for_server()

    rospy.loginfo("[TASK] Extract 3D Point server up")

    rospy.loginfo("[TASK] Initiating Task Object Recognition and Manipulation")

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['done', 'failed'])

    # Define the goal for the action client
    # nav_goal = 
    yolo_goal = YOLODetectionGoal()

    # Open the container
    with sm:
        # # State that goes to the shelf
        # smach.StateMachine.add('GO_TO_SHELF', 
        #                         go_to_shelf(),
        #                         transitions={'succeeded': 'GET_DETECTION',
        #                                     'aborted': 'failed'})

        # State that gets the object detections from YOLO
        smach.StateMachine.add('GET_DETECTION', 
                               SimpleActionState('YOLO_detection',
                                                 YOLODetectionAction, 
                                                 goal=yolo_goal,
                                                 result_slots=['LabeledImage', 'DetectedObjs', 'Success']),
                               transitions={'succeeded': 'DETECTION_LOG',
                                            'aborted': 'failed',
                                            'preempted': 'DETECTION_LOG'},
                               remapping={'LabeledImage': 'labeled_img', 
                                          'DetectedObjs': 'bboxes'})
        
         # State that logs the detection
        smach.StateMachine.add('DETECTION_LOG', 
                               detection_log(), 
                               transitions={'log_saved': 'SELECT_OBJECT',
                                            'aborted': 'failed'},
                               remapping={'labeled_img': 'labeled_img', 
                                          'bboxes': 'bboxes'})
        
        # State that logs that selects the current object for manipulation
        smach.StateMachine.add('SELECT_OBJECT', 
                               select_object(), 
                               transitions={'all_objects' : 'done',
                                            'get_object_point': 'GET_OBJECT_POINT',
                                            'aborted': 'failed'},
                               remapping={'selected_bbox' : 'selected_bbox'})
        
        # State that extracts the 3D point for a given object
        smach.StateMachine.add('GET_OBJECT_POINT', 
                               get_object_point(),
                               transitions={'succeeded': 'SELECT_OBJECT', #'PICK_OBJECT',
                                            'aborted': 'failed',#'PICK_OBJECT',
                                            'preempted': 'SELECT_OBJECT'},#'PICK_OBJECT'},
                               remapping={'selected_bbox': 'selected_bbox', 
                                          'Point': 'point'})

        # # State that executes the object manipulation
        # smach.StateMachine.add('PICK_OBJECT', 
        #                        answers_log(), 
        #                        transitions={'pick_up_object': 'select_object',
        #                                     'aborted': 'failed'},
        #                        remapping={'nlu_input': 'nlu_input', 
        #                                   'nlu_output': 'nlu_output'})

    sis = smach_ros.IntrospectionServer('object_manipulation', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()
    print(outcome)

    rospy.spin()
    sis.stop()

if __name__ == "__main__":
    main()
