#!/usr/bin/venv_utbots_tasks/bin/python

import rospy
import smach
import smach_ros
import actionlib
from utbots_actions.msg import InterpretNLUAction, InterpretNLUGoal
from smach_ros import SimpleActionState
from fpdf import FPDF

number_of_questions = 6

# This state will count the questions
class question_counter(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['action_understood', 'all_answered','aborted'],
                            input_keys=['question_counter'],
                            output_keys=['question_counter_out'])

    def execute(self, userdata):

        rospy.loginfo('Executing state question_counter')

        if userdata.question_counter >= number_of_questions:
            return 'all_answered'

        try:
            userdata.question_counter_out = userdata.question_counter + 1
            return 'action_understood'
        
        except rospy.ROSInterruptException:
            return 'aborted'

# This state will log the answers
class answers_log(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['log_saved', 'aborted'],
                            input_keys=['nlu_input', 'nlu_output'])        

    def execute(self, userdata):
        global question_number, log
        
        rospy.loginfo('Executing state answers_log')

        try:
            question = userdata.nlu_input.data
            answer = userdata.nlu_output.data

            question = question.replace("data: ", "")
            answer = answer.replace("data: ", "")

            rospy.loginfo(f"Question: {question}")
            rospy.loginfo(f"Answer: {answer}")

            question_log =  str(question_number) + '- ' + question + '\n' + answer + '\n'
            rospy.loginfo(f"Question log: {question_log}")

            question_number += 1
            log += question_log
            
            return 'log_saved'
        
        except rospy.ROSInterruptException:
            return 'aborted'


def main():

    rospy.init_node('smach_answer_questions', anonymous=True)

    global log
    global question_number
    log = str('')
    question_number = 1

    client = actionlib.SimpleActionClient('interpret_nlu', InterpretNLUAction)
    client.wait_for_server()

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['done', 'failed'])
    sm.userdata.counter = 0

    # Define the goal for the action client
    goal = InterpretNLUGoal()

    # Open the container
    with sm:
        # State that counts the questions
        smach.StateMachine.add('QUESTION_COUNTER', 
                               question_counter(), 
                               transitions={'action_understood': 'LISTEN_ANSWER',
                                            'all_answered': 'done',
                                            'aborted': 'failed'},
                               remapping={'question_counter': 'counter',
                                          'question_counter_out': 'counter'})

        # State that executes the InterpretNLUAction
        smach.StateMachine.add('LISTEN_ANSWER', 
                               SimpleActionState('interpret_nlu',
                                                 InterpretNLUAction, 
                                                 goal=goal,
                                                 result_slots=['NLUInput', 'NLUOutput']),
                               transitions={'succeeded': 'ANSWERS_LOG',
                                            'aborted': 'ANSWERS_LOG',
                                            'preempted': 'ANSWERS_LOG'},
                               remapping={'NLUInput': 'nlu_input', 
                                          'NLUOutput': 'nlu_output'})

        # State that logs the answers
        smach.StateMachine.add('ANSWERS_LOG', 
                               answers_log(), 
                               transitions={'log_saved': 'QUESTION_COUNTER',
                                            'aborted': 'failed'},
                               remapping={'nlu_input': 'nlu_input', 
                                          'nlu_output': 'nlu_output'})

    sis = smach_ros.IntrospectionServer('answer_questions', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()
    print(outcome)
    print(log)
    log = ''.join(c for c in log if ord(c) < 256)

    # Save log to PDF
    current_time = rospy.Time.now()
    pdf = FPDF()
    pdf.add_page()
    pdf.set_font("Times", size=10)
    pdf.multi_cell(0, 5, log)
    pdf.output(f"/home/laser/catkin_ws/src/utbots_tasks/task_manager/logs/answer_questions_log_{current_time}.pdf")

    rospy.spin()
    sis.stop()

if __name__ == "__main__":
    main()
