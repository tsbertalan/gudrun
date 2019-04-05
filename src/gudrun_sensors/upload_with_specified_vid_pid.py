#!/usr/bin/env python
from __future__ import print_function
from os import system, readlink
from os.path import dirname, join
from time import sleep
from subprocess import check_output

def cmd(*parts):
	print('\n$ ' + ' '.join(parts))
	return check_output(parts)


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
		change_vendor_product(pre_string=self.pre_string, vid=self.vid, pid=self.pid, do_backup=True)

	def __exit__(self, type, value, traceback):
		restore_backup()

def upload(port=None, **kw):
	from glob import glob

	
	inofile = glob('*.ino')[0]

	with modified_boards_txt(**kw):
		cmd('arduino', '--port', port, '--board', 'arduino:avr:leonardo', '--upload', inofile)



if __name__ == '__main__':
	# change_vendor_product(pid='8037')

	# with modified_boards_txt(pid='8037'):
	# 	for k in range(10, 0, -1):
	# 		print(k)
	# 		sleep(1)


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
	