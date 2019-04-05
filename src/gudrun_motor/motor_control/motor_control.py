from __future__ import print_function
import serial, time

from subprocess import check_output

PORT = check_output(['rosrun', 'gudrun_sensors', 'get_usb_device_by_ID.py', 'motor_control']).strip()
BAUDRATE = 115200

ser = serial.Serial(PORT, BAUDRATE)

def checksum(msg):
    t = 0
    for c in msg:
        if isinstance(msg, str):
            t += ord(c)
        else:
            t += c
    return t & 0xff

HEADER = 0x7E

def write(c):
    # print('Writing "%s"' % c, end=' ')
    b = ser.write(chr(c))
    # print('(%d bytes)' % b)


def send(a, b):
    c = checksum([HEADER, a, b])
    print('Sending a=%s, b=%s; checksum=%s' % (a, b, c))
    write(HEADER)
    write(a)
    write(b)
    write(c)
    write(42)
    time.sleep(.01)

    print('Response:')
    response = []
    while ser.in_waiting > 0:
        data = ser.readline()
        response.append(data)
    print(''.join(['    %s' % r for r in response]))
    print('%d lines' % (len(response),))

write(42)
send(180, 180)
