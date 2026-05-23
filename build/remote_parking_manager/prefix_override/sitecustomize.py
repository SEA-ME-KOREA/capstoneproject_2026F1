import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jihyun/remote_parking_ws_1/install/remote_parking_manager'
