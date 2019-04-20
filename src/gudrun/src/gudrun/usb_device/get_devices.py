from __future__ import print_function
from subprocess import check_output


def all_device_paths():
    devices = [
        l.split('#')
        for l in check_output(
            'rosrun gudrun list_usb_devices.sh'.split()
        ).split('\n')
        if len(l.strip()) > 0
    ]
    full_devices_map = {}
    for row in devices:
        path, serial_id, modelid, model, vendorid, vendor = row
        sid = '%s_%s:%s' % (serial_id.strip(), modelid.strip(), vendorid.strip())
        if sid not in full_devices_map:
            full_devices_map[sid] = []
        full_devices_map[sid].append(path.strip()) 

    return full_devices_map

def preferred_device_paths():

    full_devices_map = all_device_paths()

    def priority(path):
        """Choose the *best* path.

        Some devices (like a virgin unflashed Pro Micro) have multiple paths
        associated with them, and the last to be listed is not necessarily
        the one they'll respond to when flashing.
        """
        out = 0
        good = 'USB', 'ACM'
        bad = 'input', 'mouse', 'event', '/dev/sd'
        for g in good:
            if g in path:
                out -= 1
        for b in bad:
            if b in path:
                out += 1
        return out
        
    devices_map = {
        sid: sorted(paths, key=priority)[0]
        for (sid, paths) in full_devices_map.items()
    }

    friendly_names = dict(
        ultrasound_uno='1a86_USB2.0-Serial',
        encoder_mega='Arduino__www.arduino.cc__0042_7543933363535110C102',
        encoder_micro='Arduino_LLC_Arduino_Leonardo_8036:2341',
        motor_control='Arduino_LLC_Arduino_Leonardo_8037:2341', 
        servo_maestro='Pololu_Corporation_Pololu_Micro_Maestro_6-Servo_Controller_00203742',
    )
    for k, v in friendly_names.items():
        if v in devices_map:
            devices_map[k] = devices_map[v]

    return devices_map


def device_path(device_id):
    return preferred_device_paths().get(device_id, 'device_not_found')

