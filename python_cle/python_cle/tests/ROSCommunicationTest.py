import unittest
import rospy
import time
import numpy as np
import cv2
import os
from cv_bridge import CvBridge, CvBridgeError

from python_cle.tests.ros_com_adapter_test import ROSComTest
from python_cle.robotsim.RosCommunicationAdapter import RosCommunicationAdapter

__author__ = 'LarsPfotzer'

class TestSequenceFunctions(unittest.TestCase):

    def setUp(self):
        # Create first instance of the ROSCommunicationAdapter
        self.rca1 = RosCommunicationAdapter()
        self.rca1.initialize("MyROSComAdapter1")
        
        # Create float publisher and image subscriber in rca1
        self.rca1_pub_float = self.rca1.register_publish_topic(ROSComTest.float_test)
        self.rca1_sub_image = self.rca1.register_subscribe_topic(ROSComTest.image_test)
        
        # Create image publisher and float subscriber in rca1
        self.rca1_pub_image = self.rca1.register_publish_topic(ROSComTest.image_test)
        self.rca1_sub_float = self.rca1.register_subscribe_topic(ROSComTest.float_test)

        # Initialize test values (float and cv image)
        self.test_float = np.float32(1.2)
        image_file = os.path.dirname(os.path.abspath(__file__)) + '/mouse_eyes_output.png'
        self.test_image = cv2.imread(image_file, cv2.IMREAD_COLOR)

        # Wait until subscriber and publisher are initialized!
        time.sleep(0.5)
        
    def test_float_publisher(self):
        # Publish float value through ROS topic
        self.rca1_pub_float.send_message(self.test_float)

        # Wait until subscriber gets the value (TODO: timeout!)
        while not self.rca1_sub_float.changed:
            time.sleep(1)
        
        # Check, if values are equal
        self.assertEqual(self.rca1_sub_float.value.data, self.test_float)
        
    def test_image_subscriper(self):
        # Create CvBridge for converting cv images to ROS image msgs and back
        bridge = CvBridge()

        # Publish image through ROS topic
        self.rca1_pub_image.send_message(bridge.cv2_to_imgmsg(self.test_image, "bgr8")) 

        # Wait until subscriber gets the value (TODO: timeout!)
        while not self.rca1_sub_image.changed:
            time.sleep(1)
        
        # Check, if images are equal
        received_image = bridge.imgmsg_to_cv2(self.rca1_sub_image.value)
        self.assertEqual(received_image.tostring(), self.test_image.tostring())
        
        # Check, if received image is not black
        ch1 = cv2.split(received_image)[0]
        cnt = cv2.countNonZero(ch1)
        self.assertNotEqual(cnt, 0)

if __name__ == '__main__':
    suite = unittest.TestLoader().loadTestsFromTestCase(TestSequenceFunctions)
    unittest.TextTestRunner(verbosity=2).run(suite)
