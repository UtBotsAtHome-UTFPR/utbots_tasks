import rospy
import smach
import smach_ros
import actionlib

import utbots_actions.msg
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Empty
from utbots_actions.msg import YOLODetectionAction, YOLODetectionGoal, Extract3DPointAction, Extract3DPointGoal
from smach_ros import SimpleActionState
from std_msgs.msg import String

import numpy as np
from scipy.spatial.transform import Rotation as R
import cv2
from cv_bridge import CvBridge
from reportlab.lib.pagesizes import A4
from reportlab.pdfgen import canvas
from reportlab.lib.units import inch
import time
# State that calls the adding new images phase

'''/usb_cam/start_capture                           
/usb_cam/stop_capture'''


class new_face(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['new_face_added', 'failed'])

        rospy.loginfo('Checking new_face action and camera')

        rospy.wait_for_service('/usb_cam/start_capture')
        rospy.wait_for_service('/usb_cam/stop_capture')

        self.client = actionlib.SimpleActionClient('new_face', utbots_actions.msg.new_faceAction)
        self.client.wait_for_server()

    def execute(self, userdata):

        rospy.loginfo('Executing state new_face')

        goal = utbots_actions.msg.new_faceGoal()
        
        goal.n_pictures.data = 1
        goal.name.data = "Operator"

        try:
            usb_cam = rospy.ServiceProxy('/usb_cam/start_capture', Empty)
            usb_cam()
        except rospy.ServiceException as e:
            rospy.logerr('Camera service failed')
            return 'failed'
        
        time.sleep(1)

        self.client.send_goal_and_wait(goal)

        if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            success = True
        
        try:
            usb_cam = rospy.ServiceProxy('/usb_cam/stop_capture', Empty)
            usb_cam()
        except rospy.ServiceException as e:
            rospy.logerr('Camera service failed')
            return 'failed'
        if success:
            return 'new_face_added'
        return 'failed'
        

# State that calls the training phase
class train(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['training_done', 'failed'])

        rospy.loginfo('Checking train action')       

        self.client = actionlib.SimpleActionClient('train', utbots_actions.msg.trainAction)
        self.client.wait_for_server()

    def execute(self, userdata):

        rospy.loginfo('Executing state train')

        goal = utbots_actions.msg.trainGoal()

        self.client.send_goal_and_wait(goal)

        if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            return 'training_done'
        return 'failed'

# State that calls the recognition phase and publishes marked image and people
class recognize(smach.State):
    def __init__(self):

        smach.State.__init__(self, outcomes=['recognized', 'failed'],
                             output_keys=['labeled_img', 'bboxes'])

        rospy.loginfo('Checking recognize action')

        self.client = actionlib.SimpleActionClient('recognize', utbots_actions.msg.recognitionAction)
        self.client.wait_for_server()


    def execute(self, userdata):

        rospy.loginfo('Executing state recognize')

        success = False

        goal = utbots_actions.msg.recognitionGoal()
        goal.ExpectedFaces.data = 1

        try:
            usb_cam = rospy.ServiceProxy('/usb_cam/start_capture', Empty)
            usb_cam()
        except rospy.ServiceException as e:
            rospy.logerr('Camera service failed')
            return 'failed'
    
        rospy.sleep(2)

        self.client.send_goal_and_wait(goal, rospy.Duration(3))

        if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            success = True
        
        try:
            usb_cam = rospy.ServiceProxy('/usb_cam/stop_capture', Empty)
            usb_cam()
        except rospy.ServiceException as e:
            rospy.logerr('Camera service failed')
            return 'failed'
        if success:
            userdata.labeled_img = self.client.get_result().Image
            userdata.bboxes = self.client.get_result().People
            return 'recognized'
        return 'failed'
    
class init_pose(smach.State):
    def __init__(self):

        smach.State.__init__(self, outcomes=['succeeded'])
        self.pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)

    def execute(self, userdata):

        pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)

        init_msg = PoseWithCovarianceStamped()
        init_msg.header.frame_id = "map"

        odom_msg = rospy.wait_for_message('/odom', Odometry)

        init_msg.pose.pose.position.x = odom_msg.pose.pose.position.x
        init_msg.pose.pose.position.y = odom_msg.pose.pose.position.y
        init_msg.pose.pose.orientation.x = odom_msg.pose.pose.orientation.x
        init_msg.pose.pose.orientation.y = odom_msg.pose.pose.orientation.y
        init_msg.pose.pose.orientation.z = odom_msg.pose.pose.orientation.z
        init_msg.pose.pose.orientation.w = odom_msg.pose.pose.orientation.w

        rospy.loginfo("setting initial pose")
        self.pub.publish(init_msg)
            
        return 'succeeded'

class turn_around(smach.State):
    def __init__(self):

        smach.State.__init__(self, outcomes=['succeeded', 'failed'])

        rospy.loginfo('Checking navigation action')

        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.client.wait_for_server()

    def execute(self, userdata):

        rospy.loginfo('Executing state turn_around')

        odom_msg = rospy.wait_for_message('/odom', Odometry)

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = odom_msg.pose.pose.position.x
        goal.target_pose.pose.position.y = odom_msg.pose.pose.position.y
        goal.target_pose.pose.position.z = 0.0

        quaternion = np.array([odom_msg.pose.pose.orientation.w, odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z])


        rotation_180_x = R.from_euler('x', 180, degrees=True).as_quat()

        # Multiply the quaternions to apply the 180-degree rotation
        flipped_quaternion = R.from_quat(rotation_180_x) * R.from_quat(quaternion)

        # Get the result as a quaternion
        flipped_quaternion = flipped_quaternion.as_quat()

        goal.target_pose.pose.orientation.x = flipped_quaternion[1]
        goal.target_pose.pose.orientation.y = flipped_quaternion[2]
        goal.target_pose.pose.orientation.z = flipped_quaternion[3]
        goal.target_pose.pose.orientation.w = flipped_quaternion[0]

        self.client.send_goal(goal)

        finished = self.client.wait_for_result()

        if not finished: rospy.logerr("Action server not available")
        else:
            rospy.loginfo(self.client.get_result())

        if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            return 'succeeded'
        return 'failed'

global bboxes
bboxes = None

# Initialize CvBridge to convert ROS image message to OpenCV image
bridge = CvBridge()

class detection_log(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['log_saved', 'aborted'],
                            input_keys=['labeled_img', 'bboxes', 'yolo_bboxes', 'yolo_labeled_img'])        

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


            yolo_log = userdata.yolo_labeled_img
            cv_image = bridge.imgmsg_to_cv2(yolo_log)
            image_filename = "/tmp/yolo_log_img.jpg"
            cv2.imwrite(image_filename, cv_image)
            image_path = "/tmp/yolo_log_img.jpg"
            c.drawImage(image_path, x=85, y=50, width=6*inch, height=4.5*inch)

            # Add some text
            c.setFont("Helvetica", 12)

            text_height = 420
            c.drawString(100, text_height, "Objects recognized:")
            text_height -= 15
            for bbox in bboxes:
                text = f"- {bbox.Class}: {bbox.id}"
                c.drawString(100, text_height, text)
                text_height -= 12
            
            text = f"- {len(userdata.yolo_bboxes.bounding_boxes)} people were detected"
            
            c.drawString(100, text_height, text)

            # Finalize the PDF file
            c.showPage()
            c.save()
            pub_text.publish("Done")
            return 'log_saved'
        
        except rospy.ROSInterruptException:
            return 'aborted'

# Transformar isso numa concurrency state machine pois as tarefas são perfeitamente cíclicas
def main():

    rospy.init_node('face_recognition_task')

    global pub_text 
    pub_text = rospy.Publisher("/robot_speech", String, queue_size=1)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['done', 'failed'])

    yolo_goal = YOLODetectionGoal()
    yolo_goal.TargetCategory.data = "person"

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('INITIAL_POSE', init_pose(), 
                               transitions={'succeeded':'NEW_FACE'})
        
        smach.StateMachine.add('NEW_FACE', new_face(), 
                               transitions={'new_face_added':'TRAIN',
                                            'failed':'failed'})
        
        smach.StateMachine.add('TRAIN', train(), 
                               transitions={'training_done':'TURN_AROUND',# 'TURN_AROUND' for the actual task
                                            'failed':'failed'})
        
        smach.StateMachine.add('TURN_AROUND', turn_around(), 
                              transitions={'succeeded':'RECOGNIZE',
                                            'failed':'failed'})

        smach.StateMachine.add('RECOGNIZE', recognize(), 
                               transitions={'recognized':'GET_DETECTION',
                                            'failed':'RECOGNIZE'})
        
        smach.StateMachine.add('GET_DETECTION', 
                               SimpleActionState('YOLO_detection',
                                                 YOLODetectionAction, 
                                                 goal=yolo_goal,
                                                 result_slots=['LabeledImage', 'DetectedObjs', 'Success']),
                               transitions={'succeeded': 'DETECTION_LOG',
                                            'aborted': 'failed',
                                            'preempted': 'DETECTION_LOG'},
                               remapping={'LabeledImage': 'yolo_labeled_img', 
                                          'DetectedObjs': 'yolo_bboxes'})

        smach.StateMachine.add('DETECTION_LOG', detection_log(),
                               transitions={'aborted': 'failed',
                                            'log_saved':'done'})

    # Execute SMACH plan
    outcome = sm.execute()
    print (outcome)

if __name__ == '__main__':
    main()