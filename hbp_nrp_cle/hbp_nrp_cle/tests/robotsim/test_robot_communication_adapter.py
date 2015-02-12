'''
Tests the ROS communication adapter
'''

import unittest
import time
import numpy as np
#import os

#import cv2
#import cv_bridge
from hbp_nrp_cle.tests.robotsim.ros_test_topics import ROSComTest
from hbp_nrp_cle.robotsim.RosCommunicationAdapter import RosCommunicationAdapter
from testfixtures import log_capture, LogCapture


__author__ = 'LarsPfotzer'


class TestSequenceFunctions(unittest.TestCase):
    '''
    Tests the ROS communication adapter
    '''
    def setUp(self):
        '''
        Set up of the test
        '''
        with LogCapture('hbp_nrp_cle.robotsim.RosCommunicationAdapter') as l:
            # Create first instance of the ROSCommunicationAdapter
            self.rca1 = RosCommunicationAdapter()
            self.rca1.initialize("MyROSComAdapter1")

            # Create float publisher and image subscriber in rca1
            self.rca1_pub_float = self.rca1.register_publish_topic(
                ROSComTest.float_test)
            #        self.rca1_sub_image = self.rca1.register_subscribe_topic(
            #            ROSComTest.image_test)

            # Create image publisher and float subscriber in rca1
            #        self.rca1_pub_image = self.rca1.register_publish_topic(
            #            ROSComTest.image_test)
            self.rca1_sub_float = self.rca1.register_subscribe_topic(
                ROSComTest.float_test)

            # Initialize test values (float and cv image)
            self.test_float = np.float32(1.2)
            #        image_file = os.path.dirname(os.path.abspath(__file__)) + \
            #            '/mouse_eyes_output.png'
            #        self.test_image = cv2.imread(image_file, cv2.IMREAD_COLOR)

            # Wait until subscriber and publisher are initialized!
            time.sleep(0.5)
        l.check(('hbp_nrp_cle.robotsim.RosCommunicationAdapter', 'INFO', 
                 'Robot comunication adapter initialized'),
                ('hbp_nrp_cle.robotsim.RosCommunicationAdapter', 'INFO', 
                 'ROS publisher created: \
topic name = /float_test, topic type = <class \'std_msgs.msg._Float32.Float32\'>'),
                ('hbp_nrp_cle.robotsim.RosCommunicationAdapter', 'INFO', 
                 'ROS subscriber created: \
topic name = /float_test, topic type = <class \'std_msgs.msg._Float32.Float32\'>'))

    @log_capture('hbp_nrp_cle.robotsim.RosCommunicationAdapter')
    def test_float_publisher(self, logcapture):
        '''
        Tests the sending of float values
        '''
        # Publish float value through ROS topic
        self.rca1_pub_float.send_message(self.test_float)

        # Wait until subscriber gets the value (TODO: timeout!)
        while not self.rca1_sub_float.changed:
            time.sleep(1)

        # Check, if values are equal
        self.assertEqual(self.rca1_sub_float.value.data, self.test_float)

#    def test_image_subscriber(self):
#        '''
#        Tests the reception of images
#        '''
#        # Create CvBridge for converting cv images to ROS image msgs and back
#        bridge = cv_bridge.CvBridge()
#
#        # Publish image through ROS topic
#        self.rca1_pub_image.send_message(bridge.cv2_to_imgmsg(
#            self.test_image, "bgr8"))
#
#        # Wait until subscriber gets the value (TODO: timeout!)
#        while not self.rca1_sub_image.changed:
#            time.sleep(1)
#
#        # Check, if images are equal
#        received_image = bridge.imgmsg_to_cv2(self.rca1_sub_image.value)
#        self.assertEqual(received_image.tostring(),
#                         self.test_image.tostring())
#
#        # Check, if received image is not black
#        ch1 = cv2.split(received_image)[0]
#        cnt = cv2.countNonZero(ch1)
#        self.assertNotEqual(cnt, 0)
        logcapture.check(('hbp_nrp_cle.robotsim.RosCommunicationAdapter', 'DEBUG',
                          'ROS message published: topic value = 1.2'),
                         ('hbp_nrp_cle.robotsim.RosCommunicationAdapter', 'DEBUG',
                          'ROS subscriber callback'))

    def test_refresh_buffers(self):
        '''
        Tests the refresh buffers function
        '''
        self.rca1.refresh_buffers(0.1)

    def test_is_alive(self):
        '''
        Tests whether the ROS communication adapter is alive
        '''
        print self.rca1.is_alive

if __name__ == '__main__':
    unittest.main()
