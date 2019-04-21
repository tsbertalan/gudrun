from __future__ import print_function
from serial import Serial
import inspect
from subprocess import check_output
import os.path

DEFAULT_SS_CONFIG_PATH_TEMPLATE = '{MODULE_DIR}/device_search_string.txt'
DEFAULT_SEARCH_STRING_BASE = 'Arduino_LLC_Arduino_Leonardo_'
DEFAULT_PRODUCT = 8035
DEFAULT_VENDOR = 2341


class USBDevice(object):

    # Defaults that should be overridden (at least the product).
    product = 8035
    vendor = 2341
    baud = 115200
    timeout = 0.1

    @property
    def _here(self):
        """Find out the what file the current USBDevice subclass is in."""
        if type(self) == USBDevice:
            raise RuntimeError('You should subclass USBDevice.')
        return inspect.getfile(type(self))

    def _write_search_string(self, product, vendor, base=DEFAULT_SEARCH_STRING_BASE):
        """Put a "search string" file in the current directory.

        To be called after flashing firmware, when we (should) know what the device VID and PID are.

        Use the get_usb_device_by_ID.py wrapper around the list_usb_devices.sh script to see what the
        currently-connected devices are.
        """
        ss_config_path = self._locate_search_string_file(ss_config_path=DEFAULT_SS_CONFIG_PATH_TEMPLATE)

        search_string = '%s%s:%d' % (base, product, vendor)

        with open(ss_config_path, 'w') as f:
            print('Writing search string "%s" to file %s.' % (search_string, ss_config_path))
            f.write(search_string)

    def _locate_search_string_file(self, ss_config_path=DEFAULT_SS_CONFIG_PATH_TEMPLATE):
        """Try to find a search string file in the current directory."""

        filename = self._here

        module_dir = os.path.dirname(filename)
        ss_config_path = ss_config_path.replace('{MODULE_DIR}', module_dir)

        return ss_config_path

    def _get_search_string_from_file(self, ss_config_path=DEFAULT_SS_CONFIG_PATH_TEMPLATE, allow_default_ss=False):
        """Given a SS file, extract the SS or (optionally, for first flashes) use a default."""

        ss_config_path = self._locate_search_string_file(ss_config_path=ss_config_path)
        
        if os.path.isfile(ss_config_path):
            print('  Found search string file at %s.' % ss_config_path)
            with open(ss_config_path, 'r') as f:
                search_string = f.read().strip()
        else:
            if allow_default_ss:
                search_string = '%s%s:%d' % (DEFAULT_SEARCH_STRING_BASE, DEFAULT_PRODUCT, DEFAULT_VENDOR)
                print('  Couldn\'t find search string file; using default.')
            else:
                raise IOError('  Couldn\'t find search string file at %s. Has the device been flashed before?' % ss_config_path)
        print('  Decided that search_string is', search_string)

        return search_string

    def _write_firmware(self, product, vendor, port=None, do_write_search_string=True, caller_subdirs=['firmware']):
        """Find and upload Arduino code to the device."""
        import os
        from gudrun.usb_device.usb_firmware import upload

        caller_location = self._here
        caller_dir = os.path.dirname(caller_location)
        firmware_location = os.path.join(caller_dir, *caller_subdirs)
        print('Changing working directory to', firmware_location)
        os.chdir(firmware_location)

        if port is None: port = self._get_port_from_search_string(self._get_search_string_from_file(allow_default_ss=True))

        print('Uploading to %s with product=%d and vendor=%d...' % (port, product, vendor))

        upload(pid=product, vid=vendor, port=port)

        if do_write_search_string:
            self._write_search_string(product, vendor)

    def _get_port_from_search_string(self, search_string):
        """Search for a device with the given search string."""
        from gudrun.usb_device.get_devices import device_path

        print('Searching for device "%s" ...' % search_string, end=' ')

        port = device_path(search_string)

        if port == 'device_not_found':
            print()
            raise IOError("Device '%s' wasn't found." % search_string)
        else:
            print('found device at %s.' % port)

        return port

    def __init__(self, allow_default_ss=False, connect=True):

        if self.product == DEFAULT_PRODUCT and self.vendor == DEFAULT_VENDOR:
            from warnings import warn
            warn('Product and vendor should be changed from their defaults of %s and %s.' % (self.product, self.vendor))

        if connect:
            search_string = self._get_search_string_from_file(allow_default_ss=allow_default_ss)
            self.port = self._get_port_from_search_string(search_string)
            print('Connecting to port=%s with baud=%s and timeout=%s ...' % (self.port, self.baud, self.timeout), end=' ')
            self.ser = Serial(self.port, self.baud, timeout=self.timeout)
            print('connected.')

    def flash(self, **kw):
        """Disconnect serial and flash firmware.

        class should probably be constructed with allow_default_ss=True
        so that never-before-flashed Arduinos can be used.
        """
        if hasattr(self, 'ser'):
            print('Disconnecting from %s before flashing.' % self.port)
            self.ser.close()
        self._write_firmware(self.product, self.vendor, **kw)
