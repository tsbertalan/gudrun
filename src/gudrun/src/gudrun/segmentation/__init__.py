from __future__ import print_function
from os.path import dirname, join
import numpy as np

from time import time

import rospy
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError


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
        rospy.Subscriber('camera/image', Image, self.receive_image)

        self.probabilities_publisher = rospy.Publisher('segmentation_probabilities', Image, queue_size=0)

        rospy.spin()

    def receive_image(self, image_message):

        # Don't do any work if no one is paying attention the results.
        if self.probabilities_publisher.get_num_connections() == 0:
            return None

        image = self._cv_bridge.imgmsg_to_cv2(image_message)
        rospy.loginfo('Image size is %s.' % (image.shape,))
        s = time()
        probabilities = self.segment(image.astype('float32') / 255.)
        drivable = probabilities[:, :, 0]
        rospy.loginfo('Segmentation took %.4f seconds.' % (time() - s,))

        s = time()
        response = (drivable * 255).astype('uint8'); encoding = 'mono8'
        #response = np.dstack([response]*3); encoding = 'rgb8'
        response = self._cv_bridge.cv2_to_imgmsg(response, encoding)
        response.header.stamp = rospy.get_rostime()
        response.header.frame_id = image_message.header.frame_id

        self.probabilities_publisher.publish(response)
        rospy.loginfo('Publishing took %.4f seconds.' % (time() - s,))


if __name__ == '__main__':

    segmenter = Segmenter()

    segmenter.test()