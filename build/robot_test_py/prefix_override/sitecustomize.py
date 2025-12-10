import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/w/work/moveit2_handeye_calib/install/robot_test_py'
