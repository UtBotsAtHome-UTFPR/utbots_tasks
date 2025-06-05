import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/robo/david_ws/src/utbots_tasks/install/utbots_tasks'
