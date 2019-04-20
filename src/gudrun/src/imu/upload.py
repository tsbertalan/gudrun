#!/usr/bin/env python
from __future__ import print_function
from subprocess import check_output
import os.path
ss_config_path = 'device_search_string.txt'
if os.path.isfile(ss_config_path):
    with open(ss_config_path, 'r') as f:
        search_string = f.read().strip()
else:
    search_string = 'Arduino_LLC_Arduino_Leonardo_8037:2341'

print('Searching for device "%s" ...' % search_string)
cmd = ['rosrun', 'gudrun', 'get_usb_device_by_ID.py', search_string]
print('$ ' + ' '.join(cmd))

port = check_output(cmd).strip()

if port == 'device_not_found':
    raise IOError("Device '%s' wasn't found." % search_string)
else:
    print('    Found device at %s.' % port)

cmd = [
    'rosrun', 'gudrun', 'upload_with_specified_vid_pid.py', 
    '--product=%d' % 8035, 
    '--vendor=%d'  % 2341,
    '--port=%s' % port,
]
print('\nUploading ...')
print('$ ' + ' '.join(cmd))
check_output(cmd)

search_string = 'Arduino_LLC_Arduino_Leonardo_8035:2341'

with open(ss_config_path, 'w') as f:
    print('Writing search string "%s" to file %s ...' % (search_string, ss_config_path))
    f.write(search_string)


