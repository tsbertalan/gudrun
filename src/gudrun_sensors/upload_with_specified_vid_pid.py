#!/usr/bin/env python
from __future__ import print_function
from os import system, readlink
from os.path import dirname, join
from time import sleep
from subprocess import check_output, STDOUT, CalledProcessError

def get_CPE_output(e):
    out = []
    import sys
    if hasattr(e, 'output'):
        out.append('Output:\n>>>>\n{}\n<<<<'.format('\n'.join([
                '    %s' % l
                for l in 
                e.output.decode(sys.getfilesystemencoding()).split('\n')
        ])))
    if hasattr(e, 'stderr'):
        out.append('Additional stderr:\n>>>>\n{}\n<<<<'.format(e.stderr.decode(sys.getfilesystemencoding())))
    return '\n'.join(out)


def cmd(*parts):
    print('\n$ ' + ' '.join(parts))
    return check_output(parts, stderr=STDOUT)


def get_boards_txt_location():
    link = cmd('which', 'arduino').strip()
    dest = readlink(link)
    if not dest.startswith('/'):
        arduino_dir = join(
            dirname(link),
            dirname(dest),
        )
    else:
        arduino_dir = dirname(dest)

    boardstxt = join(
        arduino_dir,
        'hardware', 'arduino', 'avr', 'boards.txt'
    ).strip()

    return boardstxt


def get_boards_txt(do_backup=True):
    print('\nGetting boards.txt contents.')
    boardstxt = get_boards_txt_location()

    if do_backup:
        r = cmd('cp', boardstxt, boardstxt+'.bak')
        print(r)

    with open(boardstxt, 'r') as f:
        original_file = f.read()

    return boardstxt, original_file


def change_vendor_product(pre_string='leonardo', vid=None, pid=None, do_backup=True):
    print('\nModding boards.txt.')

    boardstxt, original_file = get_boards_txt(do_backup=do_backup)

    new_id = dict(vid=vid, pid=pid)
    print('Changing IDs for %s to %s.' % (pre_string, new_id))

    new_file = []
    for l in original_file.split('\n'):
        for k in 'vid', 'pid':
            if new_id[k] is not None:
                xid = new_id[k]
                if not xid.startswith('0x'):
                    xid = '0x%s' % xid
                if '%s.build.%s=' % (pre_string, k) in l:
                    print('From:', l)
                    l = '%s.build.%s=%s' % (pre_string, k, xid)
                    print('To:', l)
        new_file.append(l)
    new_file = '\n'.join(new_file)

    with open(boardstxt, 'w') as f:
        f.write(new_file)


def restore_backup():
    boardstxt = get_boards_txt_location()
    print('\nRestoring backup to %s' % boardstxt)
    cmd('cp', boardstxt, '/tmp/boards.txt.modified')
    return cmd('cp', boardstxt+'.bak', boardstxt)


class modified_boards_txt(object):

    def __init__(self, pre_string='leonardo', vid=None, pid=None):
        self.pre_string = pre_string
        self.vid = vid
        self.pid = pid

    def __enter__(self):
        print('>>>> Modifying boards.txt. ...')
        change_vendor_product(pre_string=self.pre_string, vid=self.vid, pid=self.pid, do_backup=True)
        print('>>>> boards.txt is modified.')

    def __exit__(self, type, value, traceback):
        print('<<<< Restoring backup. ...')
        restore_backup()
        print('<<<< Backup restored.')


def upload(port=None, **kw):
    print('This is upload_with_specified_vid_pid.upload')
    from glob import glob

    
    inofiles = glob('*.ino')
    pdefiles = glob('*.pde')
    if len(inofiles) > 0:
        inofile = inofiles[0]
    elif len(pdefiles) > 0:
        inofile = pdefiles[0]
    else:
        raise Exception('No .ino or .pde files found!')



    print('Found inofile:', inofile)

    with modified_boards_txt(**kw):
        try:
            res = cmd('arduino', '--port', port, '--board', 'arduino:avr:leonardo', '--upload', inofile)
            # res = system(' '.join(['arduino', '--port', port, '--board', 'arduino:avr:leonardo', '--upload', inofile]))

        except CalledProcessError as e:
            import sys
            full_output = get_CPE_output(e)
            # new_e = CalledProcessError('%s\n%s' % (e.returncode, full_output), e.cmd, e.output)
            new_e = RuntimeError('%s:\n%s' % (e, full_output))
            raise new_e

        print('upload_result:', res)



if __name__ == '__main__':
    print('upload_with_specified_vid_pid')
    # change_vendor_product(pid='8037')

    # with modified_boards_txt(pid='8037'):
    #   for k in range(10, 0, -1):
    #       print(k)
    #       sleep(1)


    import argparse
    parser = argparse.ArgumentParser(description='Upload ino file with specified product and vendor ID.')
    parser.add_argument('--product', default='8036')
    parser.add_argument('--vendor', default='2341')
    parser.add_argument('--port', default=None)
    args = parser.parse_args()

    if args.port is None:
        args.port = cmd('rosrun', 'gudrun_sensors', 'get_usb_device_by_ID.py', 'Arduino_LLC_Arduino_Leonardo_8036:2341').strip()
        print('No port was specified, so getting default as', args.port)
    upload(pid=args.product, vid=args.vendor, port=args.port)
    