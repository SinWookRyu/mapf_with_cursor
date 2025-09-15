import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/mnerd/mapf_sim_web_ws/install/mapf_simulator'
