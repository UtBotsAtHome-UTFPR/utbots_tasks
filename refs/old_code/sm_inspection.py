import rospy
import smach
import smach_ros
import actionlib

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String

import time


class set_nav_goal(smach.State):
    def __init__(self):

        smach.State.__init__(self, outcomes=['succeeded'], output_keys=["goal1", "goal2"])

    def execute(self, userdata):

        rospy.loginfo("Setting nav goals")

        goal1 = PoseStamped()
        goal1.header.frame_id = "map"

        goal1.pose.position.x = 6.840
        goal1.pose.position.y = 2.289

        goal1.pose.orientation.z = 0.828
        goal1.pose.orientation.w = 0.561

        goal2 = PoseStamped()
        goal2.header.frame_id = "map"

        goal2.pose.position.x = -0.149
        goal2.pose.position.y = 7.331

        goal2.pose.orientation.z = -0.991
        goal2.pose.orientation.w = 0.133

        userdata.goal1 = goal1
        userdata.goal2 = goal2
        
        return "succeeded"

class wait_door_open(smach.State):
    def __init__(self):

        smach.State.__init__(self, outcomes=['succeeded', 'timed_out'])
        self.time_out = 200

    def execute(self, userdata):

        rospy.loginfo("Waiting for door to open")
        pub_vm.publish("Waiting for door")

        start_time = time.time()
        curr_time = float('inf')

        while(start_time + self.time_out < curr_time):
            
            scan = rospy.wait_for_message('/scan', LaserScan)
            length = len(scan.ranges)
            # Check if the door is open and return succeeded
            #check_vec = scan.ranges[(length/2) - 30:(length/2) + 30]
            size = len(scan.ranges)
            sub_vec_a = scan.ranges[0:31]
            sub_vec_b = scan.ranges[size - 30:size -1]

            check_vec = sub_vec_a + sub_vec_b

            if len([i for i in check_vec if i > 1.5]) > 45:
                rospy.loginfo("Detected open door")
                time.sleep(5)
                pub_vm.publish("Detected open door")
                return "succeeded"
            curr_time = time.time()

        rospy.loginfo("Timed out waiting for door to open")
        pub_vm.publish("Timed out waiting for door to open")

        return "timed_out"
    

# Estado de ir até um ponto (meio da sala e fora da sala)

class go_to(smach.State):
    def __init__(self):

        smach.State.__init__(self, outcomes=['succeeded', 'failed'], input_keys=['goal'])

        self.goal_pub = rospy.Publisher('/bridge_navigate_to_pose/goal', PoseStamped, queue_size=1)

    def execute(self, userdata):

        pub_vm.publish("Going to objective")

        rospy.loginfo('Executing state go_to')

        goal = PoseStamped()

        goal.pose.position.x = userdata.goal.pose.position.x
        goal.pose.position.y = userdata.goal.pose.position.y

        goal.pose.orientation.z = userdata.goal.pose.orientation.z
        goal.pose.orientation.w = userdata.goal.pose.orientation.w

        self.goal_pub.publish(goal)

        turn_success = rospy.wait_for_message('/bridge_navigate_to_pose/result', String, timeout=600)

        if turn_success.data.lower() == "succeeded":
            return 'succeeded'
        return 'failed'

# Wait for voice message
class wait_vm(smach.State):
    def __init__(self):

        smach.State.__init__(self, outcomes=['succeeded', 'failed'], input_keys=['keyword'])

    def execute(self, userdata):

        rospy.loginfo('Executing state wait_vm')
        pub_vm.publish("Waiting for go ahead voice message")

        while(True):
            msg = rospy.wait_for_message('/utbots/voice/stt/whispered', String)
            if "go ahead" in msg.data.lower():
                pub_vm.publish("I heard go ahead")
                time.sleep(3)
                return "succeeded"

        return 'failed'

# Estado de ir até um ponto fora


def main():

    rospy.init_node('face_recognition_task')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['done', 'failed'])

    global pub_vm
    pub_vm = rospy.Publisher("/utbots/voice/tts/robot_speech", String, queue_size=1)

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('NAV_GOAL_SETUP', set_nav_goal(), 
                                transitions={'succeeded':"WAIT_DOOR_OPEN"},
                                remapping={"goal1":"goal1",
                                          "goal2":"goal2"})
        smach.StateMachine.add('WAIT_DOOR_OPEN', wait_door_open(), 
                                transitions={'succeeded':"GO_TO1", 
                                            'timed_out':"failed"})
        
        smach.StateMachine.add('GO_TO1', go_to(), 
                                transitions={'succeeded':"WAIT_VM", 
                                            'failed':"failed"},
                                remapping={"goal":"goal1"})
        
        smach.StateMachine.add('WAIT_VM', wait_vm(), 
                                transitions={'succeeded':"GO_TO2", 
                                            'failed':"failed"},
                                remapping={"goal":"goal1"})

        smach.StateMachine.add('GO_TO2', go_to(), 
                                transitions={'succeeded':"done", 
                                            'failed':"failed"},
                                remapping={"goal":"goal2"})

    # Execute SMACH plan
    outcome = sm.execute()
    print (outcome)

if __name__ == '__main__':
    main()