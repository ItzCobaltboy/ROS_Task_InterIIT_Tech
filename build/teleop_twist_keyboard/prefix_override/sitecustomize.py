import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/cobaltboy/ROS2/ROS_Task_InterIIT_Tech/install/teleop_twist_keyboard'
