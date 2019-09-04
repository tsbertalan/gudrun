from __future__ import print_function
from copy import deepcopy
from pprint import pprint
import rosbag, numpy as np, tqdm
from scipy.misc import imsave

from  cv_bridge import CvBridge, CvBridgeError



def get_data_from_bag(bag_file_path):
    bag = rosbag.Bag(bag_file_path)

    topics = [
        '/controller_odom/twist_actual',
        '/steering_smoothed',
        '/ackermann_cmd',
        '/tf',
        '/d400_camera_throttled',
        '/speed_control/measured',
    ]

    seen = set()


    header_template = {
        'stamp': {
            'secs': int,
            'nsecs': int,
        },
        'frame_id': None,
        'time_nsec': [],
        }

    cv_bridge = CvBridge()


    # '/tf'
    # rospy.Time[1567452558986454777]
    # transforms: 
    #   - 
    #     header: 
    #       seq: 0
    #       stamp: 
    #         secs: 1567452558
    #         nsecs: 994924784
    #       frame_id: "t265_odom_frame"
    #     child_frame_id: "t265_pose_frame"
    #     transform: 
    #       translation: 
    #         x: 0.000461905176053
    #         y: 0.00186651595868
    #         z: -0.000260996981524
    #       rotation: 
    #         x: 0.0200446005911
    #         y: 0.0196481198072
    #         z: -0.000724234210793
    #         w: 0.999605774879
    tf_item_template = {
        'header': deepcopy(header_template),
        'child_frame_id': None,
        'transform': {
            'translation': {'x': None, 'y': None, 'z': None},
            'rotation': {'x': None, 'y': None, 'z': None, 'w': None},
        },
    }

    data = {
        # '/d400_camera_throttled'
        # header: 
        #   seq: 3359
        #   stamp: 
        #     secs: 1567452559
        #     nsecs: 924329655
        #   frame_id: "d400_color_optical_frame"
        # height: 480
        # width: 640
        # encoding: "rgb8"
        # is_bigendian: 0
        # step: 1920
        # data: [ ... truncated ... ]
        '/d400_camera_throttled': {
            'header': deepcopy(header_template),
            'height': None,
            'width': None,
            'encoding': None,
            'data': np.ndarray,
        },
        '/tf': [],

        # '/steering_smoothed'
        # rospy.Time[1567452558990950896]
        # data: 0.0
        '/steering_smoothed': {'data': float},

        # '/speed_control/measured'
        # rospy.Time[1567452558991076391]
        # data: 0.0
        '/speed_control/measured': {'data': float},

        # '/ackermann_cmd'
        # rospy.Time[1567452574990623258]
        # header: 
        #   seq: 1
        #   stamp: 
        #     secs: 1567452574
        #     nsecs: 990479946
        #   frame_id: "odom"
        # drive: 
        #   steering_angle: 0.0
        #   steering_angle_velocity: 0.0
        #   speed: 0.10000000149
        #   acceleration: 0.0
        #   jerk: 0.0
        '/ackermann_cmd': {
            'header': deepcopy(header_template),
            'drive': {
                'steering_angle': float,
                'steering_angle_velocity': float,
                'speed': float,
                'acceleration': float,
                'jerk': float,
            }
        },
        # '/controller_odom/twist_actual'
        # rospy.Time[1567452558990769671]
        # header: 
        #   seq: 1
        #   stamp: 
        #     secs: 1567452558
        #     nsecs: 990611076
        #   frame_id: ''
        # twist: 
        #   twist: 
        #     linear: 
        #       x: 0.0
        #       y: 0.0
        #       z: 0.0
        #     angular: 
        #       x: 0.0
        #       y: 0.0
        #       z: 0.0
        #   covariance: [0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.15]
        '/controller_odom/twist_actual': {
            'header': deepcopy(header_template),
            'twist': {
                'twist': {
                    'linear': {'x': None, 'y': None, 'z': None},
                    'angular': {'x': None, 'y': None, 'z': None},
                },
                'covariance': np.asarray,
            }
        }
    }

    def parse_message(topic, msg, time):

        def walk(obj, key, storage):
            # print('Walking', storage, 'with key', key)
            value = storage.get(key)
            datum = getattr(obj, key, None)
            if datum is None:
                return
            if value is None:
                storage[key] = datum
            elif isinstance(value, dict):
                scan(datum, value)
            elif isinstance(value, list):
                # print('Creating a', type(value[-1]))
                last = value[-1]
                if isinstance(last, np.ndarray):
                    if topic == '/d400_camera_throttled':
                        datum = cv_bridge.imgmsg_to_cv2(msg)
                    t = np.asarray
                else:
                    t = type(last)
                converted = t(datum)
                value.append(converted)
            elif isinstance(value, type) or callable(value):
                if value == np.ndarray and type(datum) == str:
                    datum = cv_bridge.imgmsg_to_cv2(msg)
                    value = np.asarray
                to_save = value(datum)
                storage[key] = [to_save]

        def scan(datum, storage):
            for key in storage.keys():
                walk(datum, key, storage)

        toplevel_storage = data.get(topic, None)

        if toplevel_storage is not None:
            if topic == '/tf':
                parsed = []
                for transform in msg.transforms:
                    storage = deepcopy(tf_item_template)
                    scan(transform, storage)
                    parsed.append(storage)
                data['/tf'].append(parsed)

            else:
                scan(msg, toplevel_storage)

            if hasattr(msg, 'header'):
                toplevel_storage['header']['time_nsec'].append(time.to_nsec())


    for topic, msg, time in bag.read_messages(topics=topics):
        parse_message(topic, msg, time)
        seen.add(topic)
 
    np.savez_compressed(bag_file_path, data)
 

# modify the default parameters of np.load so we can load a .npz with dicts in python2.
np_load_old = np.load
np.load = lambda *a,**k: np_load_old(*a, allow_pickle=True, **k)

if __name__ == '__main__':
    from glob import glob
    folder = '/home/tsbertalan/Dropbox/data/gudrun/september_2/'
    paths = glob(folder + '*.bag')
    for path in tqdm.tqdm(paths, unit='bag'):
        import os
        if not os.path.exists(path + '.npz'):
            get_data_from_bag(path)

        os.system('mkdir -p "%s_images"' % (path))


        data = np.load(path + '.npz')['arr_0'].reshape((1,))[0]
        images = data['/d400_camera_throttled']['data']
        for i, image in enumerate(tqdm.tqdm(images, unit='images', desc='PNG files')):
            from os.path import join, basename, splitext
            imname = ''.join(list(splitext(basename(path))[:-1]) + ['-%05d' % i, '.png'])
            impath = join(path + '_images', imname)
            imsave(impath, image)
            