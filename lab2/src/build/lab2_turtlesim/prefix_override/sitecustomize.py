import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/cc/ee106a/fa26/class/ee106a-aec/ros_workspaces/lab2/src/install/lab2_turtlesim'
