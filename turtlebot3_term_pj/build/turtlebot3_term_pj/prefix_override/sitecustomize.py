import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/hk/turtlebot3_term_pj/install/turtlebot3_term_pj'
