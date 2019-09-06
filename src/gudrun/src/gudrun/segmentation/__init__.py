from __future__ import print_function
from os.path import dirname, join
import numpy as np

from time import time

import rospy, message_filters
from sensor_msgs.msg import Image, CameraInfo

import cv2
from cv_bridge import CvBridge


class Segmenter(object):

    def __init__(self):

        self._here = dirname(__file__)

        rospy.loginfo('Importing tensorflow for segmentation ...')
        import tensorflow as tf
        rospy.loginfo('Imported tensorflow for segmentation.')

        load_keras_allinone = False
        if load_keras_allinone:
            # This doesn't work when trained in 3.6 and loaded in 2.7.
            model = tf.keras.models.load_model(join(self._here, 'Unet.h5'))
            predict = model.predict

        else:
            self.sess = tf.InteractiveSession()

            # Load the graph definition (tensors and variables).
            tf.train.import_meta_graph(join(self._here, 'Unet_model.ckpt.meta'))
            graph = tf.get_default_graph()

            input_tensor = graph.get_tensor_by_name('input_tensor:0')
            probabilities = graph.get_tensor_by_name('probabilities:0')

            # Load the weights (variable values).
            saver = tf.train.Saver()
            saver.restore(self.sess, join(self._here, 'Unet_model.ckpt'))

            def predict(x):
                return self.sess.run(probabilities, feed_dict={input_tensor: x})

        self._predict = predict

    def __call__(self, data):
        if len(data.shape) == 3:
            data = data[np.newaxis, ...]

        predictions = self._predict(data)

        return np.squeeze(predictions)

    def test(self):

        import matplotlib.pyplot as plt

        # Load the data.
        X = np.load(join(self._here, 'X12.npz'))['X']
        print('Data shape:', X.shape)

        from time import time
        s = time()
        predictions = self(X)
        d = time() - s
        print('Did %d predictions in %.2f seconds (%f img/s; %.4f s/img).' % (len(X), d, len(X)/d, d/len(X)))

        X0 = np.expand_dims(X[0], 0)
        X0 = X0 + np.random.normal(size=X0.shape)
        s = time()
        once = self(X0)
        d = time() - s
        print('Did 1 predictions in %.4f seconds.' % (d,))

        print(predictions.shape)

        fig, axes = plt.subplots(nrows=3, ncols=4)
        for ax, img in zip(axes.ravel(), predictions):
            ax.imshow(img[:, :, 0])
            ax.set_xticks([])
            ax.set_yticks([])
        fig.tight_layout()
        fig.subplots_adjust(wspace=0, hspace=0)

        fig.savefig(join(self._here, 'predictions12.png'))


class SegmentationNode(object):

    def __init__(self):
        
        self.segment = Segmenter()

        self._cv_bridge = CvBridge()

        rospy.init_node('segmentation_node')

        # Recieve all three messages at the same time.
        color_sub = message_filters.Subscriber('camera/image', Image)
        depth_sub = message_filters.Subscriber('camera/depth', Image)
        info_sub = message_filters.Subscriber('camera/camera_info', CameraInfo)
        ts = message_filters.TimeSynchronizer([color_sub, depth_sub, info_sub], queue_size=1)
        ts.registerCallback(self.callback)

        self.undrivable_depth_pub = rospy.Publisher('undrivable/depth/image_raw', Image, queue_size=4)
        self.undrivable_depth_camera_info_pub = rospy.Publisher('undrivable/depth/camera_info', CameraInfo, queue_size=4)

        rospy.spin()

    def callback(self, image_message, depth_message, camera_info_message):

        # Don't do any work if no one is paying attention to the results.
        if self.undrivable_depth_pub.get_num_connections() == 0:
            return None

        color_ar = self._cv_bridge.imgmsg_to_cv2(image_message)
        s = time()
        probabilities = self.segment(color_ar.astype('float32') / 255.)
        drivable = probabilities[:, :, 0] > rospy.get_param('~drivable_threshold', 0.5)
        rospy.loginfo('Segmentation took %.4f seconds.' % (time() - s,))

        depth_ar = self._cv_bridge.imgmsg_to_cv2(depth_message)
        
        undrivable = np.array(depth_ar).reshape((-1,))
        undrivable[drivable.ravel()] = 0
        undrivable = undrivable.reshape(depth_ar.shape)

        # npoints = (undrivable > 0).sum()
        # rospy.loginfo('Raw depth has %d points.' % npoints)

        # drivable_frac = float(drivable.sum()) / drivable.size
        # rospy.loginfo('%.1f%% of RGB pixels are drivable.' % (drivable_frac * 100,))

        subsample = 6
        if subsample > 1:
            xbin = subsample
            ybin = subsample
            undrivable = undrivable[::ybin, ::xbin]
            camera_info_message.height = undrivable.shape[0]
            camera_info_message.width = undrivable.shape[1]
            camera_info_message.binning_x = xbin
            camera_info_message.binning_y = ybin

        # npoints_decimated = (undrivable > 0).sum()
        # rospy.loginfo('Decimating obstacle cloud to %d points.' % (npoints_decimated,))

        response = self._cv_bridge.cv2_to_imgmsg(undrivable, encoding='16UC1')
        response.header = depth_message.header

        self.undrivable_depth_pub.publish(response)
        self.undrivable_depth_camera_info_pub.publish(camera_info_message)


if __name__ == '__main__':

    segmenter = Segmenter()

    segmenter.test()