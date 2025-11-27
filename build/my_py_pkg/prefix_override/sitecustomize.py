import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/m/Public/ros2-humble-workspace/install/my_py_pkg'
