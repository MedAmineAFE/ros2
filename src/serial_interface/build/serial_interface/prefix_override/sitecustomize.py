import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rpi/ros2_ws/src/serial_interface/install/serial_interface'
