#!/bin/bash
# https://unix.stackexchange.com/a/144735

for sysdevpath in $(find /sys/bus/usb/devices/usb*/ -name dev); do
    (
        syspath="${sysdevpath%/dev}"
        devname="$(udevadm info -q name -p $syspath)"
        [[ "$devname" == "bus/"* ]] && continue
        eval "$(udevadm info -q property --export -p $syspath)"
        [[ -z "$ID_SERIAL" ]] && continue
        echo "/dev/$devname # $ID_SERIAL # $ID_MODEL_ID # $ID_MODEL # $ID_VENDOR_ID # $ID_VENDOR"
    )
done

## sample output from `udevadm info -q property --export -p $syspath`:

# DEVNAME='/dev/ttyACM0'
# DEVPATH='/devices/pci0000:00/0000:00:14.0/usb1/1-9/1-9:1.0/tty/ttyACM0'
# ID_BUS='usb'
# ID_MM_CANDIDATE='1'
# ID_MODEL='Arduino_Leonardo'
# ID_MODEL_ENC='Arduino\x20Leonardo'
# ID_MODEL_FROM_DATABASE='Sunrise Point-H USB 3.0 xHCI Controller'
# ID_MODEL_ID='8037'
# ID_PATH='pci-0000:00:14.0-usb-0:9:1.0'
# ID_PATH_TAG='pci-0000_00_14_0-usb-0_9_1_0'
# ID_PCI_CLASS_FROM_DATABASE='Serial bus controller'
# ID_PCI_INTERFACE_FROM_DATABASE='XHCI'
# ID_PCI_SUBCLASS_FROM_DATABASE='USB controller'
# ID_REVISION='0100'
# ID_SERIAL='Arduino_LLC_Arduino_Leonardo'
# ID_TYPE='generic'
# ID_USB_CLASS_FROM_DATABASE='Miscellaneous Device'
# ID_USB_DRIVER='cdc_acm'
# ID_USB_INTERFACES=':020200:0a0000:'
# ID_USB_INTERFACE_NUM='00'
# ID_USB_PROTOCOL_FROM_DATABASE='Interface Association'
# ID_VENDOR='Arduino_LLC'
# ID_VENDOR_ENC='Arduino\x20LLC'
# ID_VENDOR_FROM_DATABASE='Arduino SA'
# ID_VENDOR_ID='2341'
# MAJOR='166'
# MINOR='0'
# SUBSYSTEM='tty'
# TAGS=':systemd:'
# USEC_INITIALIZED='1403561126'
