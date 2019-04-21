#!/usr/bin/env python
from gudrun.imu import IMUNode
IMUNode()



# def get_port_address(verbose=False):
#     import os.path
#     from subprocess import check_output
    
#     ss_config_path = os.path.join(os.path.dirname(__file__), 'device_search_string.txt')

#     if os.path.isfile(ss_config_path):
#         with open(ss_config_path, 'r') as f:
#             search_string = f.read().strip()
#     else:
#         search_string = 'Arduino_LLC_Arduino_Leonardo_8037:2341'
#         from warnings import warn
#         warn('No %s; using default search string of %s.' % (ss_config_path, search_string))

#     if verbose: print('Searching for device "%s" ...' % search_string)
    
#     cmd = ['rosrun', 'gudrun', 'get_usb_device_by_ID.py', search_string]
#     if verbose: print('$ ' + ' '.join(cmd))
#     addr = check_output(cmd).strip()

#     if addr == 'device_not_found':
#         raise IOError("Device '%s' wasn't found." % search_string)
    
#     return addr


# if __name__ == '__main__':
#     ros_publish()
