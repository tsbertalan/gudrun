#!/usr/bin/env python
from __future__ import print_function
from subprocess import check_output


def get_all_device_paths():
    devices = [
        l.split('#')
        for l in check_output(
            'rosrun gudrun_sensors list_usb_devices.sh'.split()
        ).split('\n')
        if len(l.strip()) > 0
    ]
    devices_map = {}
    for row in devices:
        path, serial_id = row
        devices_map[serial_id.strip()] = path.strip()

    friendly_names = dict(
        ultrasound_uno='1a86_USB2.0-Serial',
        encoder_mega='Arduino__www.arduino.cc__0042_7543933363535110C102',
        servo_maestro='Pololu_Corporation_Pololu_Micro_Maestro_6-Servo_Controller_00203742',
    )
    for k, v in friendly_names.items():
        if v in devices_map:
            devices_map[k] = devices_map[v]

    return devices_map


def get_device_path(device_id):
    return get_all_device_paths().get(device_id, 'device_not_found')


def main():
    from sys import argv

    if len(argv) < 2:
        devices_map = get_all_device_paths()
        from pprint import pprint
        pprint(devices_map)

    else:
        print(get_device_path(argv[1]))


if __name__ == '__main__':
    main()
