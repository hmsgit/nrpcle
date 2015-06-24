'''
Tests the ROS communication adapter
'''

import unittest
import rospy
from mock import patch
from hbp_nrp_cle.robotsim.RobotInterface import PreprocessedTopic, Topic
from hbp_nrp_cle.robotsim.RosCommunicationAdapter import RosCommunicationAdapter, RosPublishedPreprocessedTopic, \
    RosPublishedTopic, RosSubscribedPreprocessedTopic, RosSubscribedTopic
from testfixtures import LogCapture


__author__ = 'LarsPfotzer'


class TestRosCommunicationAdapter(unittest.TestCase):
    """
    Tests the ROS communication adapter module
    """

    # Tests for ROSCommunicationAdapter
    @patch('hbp_nrp_cle.robotsim.RosCommunicationAdapter.rospy.init_node')
    def test_rca_initialize(self, mock_init_node):
        with LogCapture('hbp_nrp_cle.robotsim.RosCommunicationAdapter') as logcapture:
            rca = RosCommunicationAdapter()
            rca.initialize("test_node")
            self.assertEquals(mock_init_node.call_count, 1)

            mock_init_node.side_effect = rospy.exceptions.ROSException
            rca.initialize("test_node")
            self.assertEquals(mock_init_node.call_count, 2)
            logcapture.check(('hbp_nrp_cle.robotsim.RosCommunicationAdapter', 'INFO',
                              'Robot comunication adapter initialized'),
                             ('hbp_nrp_cle.robotsim.RosCommunicationAdapter', 'WARNING',
                              'ROS node already initialized')
            )

    @patch('hbp_nrp_cle.robotsim.RosCommunicationAdapter.rospy.init_node')
    @patch('hbp_nrp_cle.robotsim.RosCommunicationAdapter.rospy.Publisher')
    def test_rca_create_topic_publisher(self, mock_rospy_publisher, mock_init_node):
        rca = RosCommunicationAdapter()
        rca.initialize("test_node")

        r = rca.create_topic_publisher(PreprocessedTopic(
            'preprocessed_topic', 'topic_type', lambda x: ()
        ), '')
        self.assertIsInstance(r, RosPublishedPreprocessedTopic)

        r = rca.create_topic_publisher(Topic('topic', 'topic_type'), '')
        self.assertIsInstance(r, RosPublishedTopic)

        self.assertEquals(mock_init_node.call_count, 1)
        self.assertGreater(mock_rospy_publisher.call_count, 0)

    @patch('hbp_nrp_cle.robotsim.RosCommunicationAdapter.rospy.init_node')
    @patch('hbp_nrp_cle.robotsim.RosCommunicationAdapter.rospy.Subscriber')
    def test_rca_create_topic_subscriber(self, mock_rospy_subscriber, mock_init_node):
        rca = RosCommunicationAdapter()
        rca.initialize("test_node")

        r = rca.create_topic_subscriber(PreprocessedTopic(
            'preprocessed_topic', 'topic_type', lambda x: ()
        ), '')
        self.assertIsInstance(r, RosSubscribedPreprocessedTopic)

        r = rca.create_topic_subscriber(Topic('topic', 'topic_type'), '')
        self.assertIsInstance(r, RosSubscribedTopic)

        self.assertEquals(mock_init_node.call_count, 1)
        self.assertGreater(mock_rospy_subscriber.call_count, 0)

    @patch('hbp_nrp_cle.robotsim.RosCommunicationAdapter.rospy.is_shutdown')
    @patch('hbp_nrp_cle.robotsim.RosCommunicationAdapter.rospy.init_node')
    def test_rca_is_alive(self, mock_init_node, mock_is_shutdown):
        rca = RosCommunicationAdapter()
        rca.initialize("test_node")
        mock_is_shutdown.return_value = False
        self.assertTrue(rca.is_alive)
        mock_is_shutdown.return_value = True
        self.assertFalse(rca.is_alive)

        self.assertEquals(mock_init_node.call_count, 1)
        self.assertEqual(mock_is_shutdown.call_count, 2)

    @patch('hbp_nrp_cle.robotsim.RosCommunicationAdapter.rospy.Publisher')
    @patch('hbp_nrp_cle.robotsim.RosCommunicationAdapter.rospy.init_node')
    def test_rca_refresh_buffers(self, mock_init_node, mock_publisher):
        rca = RosCommunicationAdapter()
        t1, t2 = Topic('a', 'b'), Topic('c', 'd')
        rca.register_publish_topic(t1)
        rca.register_publish_topic(t2)
        rca.initialize("test_node")
        rca.refresh_buffers(0.1)

        for key in rca.subscribed_topics:
            self.assertTrue(rca.subscribed_topics[key].changed)

        self.assertEquals(mock_init_node.call_count, 1)
        self.assertEquals(mock_publisher.call_count, 2)

    # Tests for RosPublishedTopic
    @patch('hbp_nrp_cle.robotsim.RosCommunicationAdapter.rospy.Publisher')
    def test_rpt_init(self, mock_rospy_publisher):
        RosPublishedTopic(Topic('topic_name', 'topic_type'))
        self.assertEquals(mock_rospy_publisher.call_count, 1)
        self.assertEquals(mock_rospy_publisher.call_args_list[0][0][0], 'topic_name')
        self.assertEquals(mock_rospy_publisher.call_args_list[0][0][1], 'topic_type')
        self.assertRaises(AssertionError, RosPublishedTopic, None)

    @patch('hbp_nrp_cle.robotsim.RosCommunicationAdapter.rospy.Publisher')
    def test_rpt_send_message(self, mock_rospy_publisher):
        rpt = RosPublishedTopic(Topic('topic_name', 'topic_type'))
        pub = mock_rospy_publisher.return_value
        rpt.send_message('message')
        self.assertEquals(pub.publish.call_count, 1)
        self.assertEquals(pub.publish.call_args_list[0][0][0], 'message')
        rpt.unregister()
        rpt.send_message('message')
        self.assertEquals(pub.publish.call_count, 1)

    @patch('hbp_nrp_cle.robotsim.RosCommunicationAdapter.rospy.Publisher')
    def test_rpt_unregister(self, mock_rospy_publisher):
        rpt = RosPublishedTopic(Topic('topic_name', 'topic_type'))
        pub = mock_rospy_publisher.return_value
        rpt.unregister()
        self.assertEquals(pub.unregister.call_count, 1)

    # Tests for RosPublishedPreprocessedTopic
    @patch('hbp_nrp_cle.robotsim.RosCommunicationAdapter.rospy.Publisher')
    def test_rppt_init(self, mock_rospy_publisher):
        self.assertRaises(AssertionError, RosPublishedPreprocessedTopic, None)
        t = PreprocessedTopic('a', 'b', lambda x: ())
        rppt = RosPublishedPreprocessedTopic(t)
        self.assertEquals(t.pre_processor, rppt._RosPublishedPreprocessedTopic__pre_processor)
        self.assertEquals(mock_rospy_publisher.call_count, 1)

    @patch('hbp_nrp_cle.robotsim.RosCommunicationAdapter.rospy.Publisher')
    def test_rppt_send_message(self, mock_rospy_publisher):
        with LogCapture('hbp_nrp_cle.robotsim.RosCommunicationAdapter') as logcapture:
            rppt = RosPublishedPreprocessedTopic(PreprocessedTopic('a', 'b', lambda x: x))
            oldp = rppt._RosPublishedTopic__pub
            rppt._RosPublishedTopic__pub = None
            rppt.send_message('message')
            rppt._RosPublishedTopic__pub = oldp
            rppt.send_message('message')
            logcapture.check(('hbp_nrp_cle.robotsim.RosCommunicationAdapter', 'INFO',
                              'ROS publisher created: topic name = a, topic type = b'),
                             ('hbp_nrp_cle.robotsim.RosCommunicationAdapter', 'ERROR',
                              'Trying to publish messages on an unregistered topic'),
                             ('hbp_nrp_cle.robotsim.RosCommunicationAdapter', 'DEBUG',
                              'ROS message published: topic value = message'))
            self.assertEquals(mock_rospy_publisher.call_count, 1)

    # Tests for RosSubscribedTopic
    @patch('hbp_nrp_cle.robotsim.RosCommunicationAdapter.rospy.Subscriber')
    def test_rst_init(self, mock_rospy_subscriber):
        RosSubscribedTopic(Topic('topic_name', 'topic_type'))
        self.assertEquals(mock_rospy_subscriber.call_count, 1)
        self.assertEquals(mock_rospy_subscriber.call_args_list[0][0][0], 'topic_name')
        self.assertEquals(mock_rospy_subscriber.call_args_list[0][0][1], 'topic_type')
        self.assertRaises(AssertionError, RosSubscribedTopic, None)

    @patch('hbp_nrp_cle.robotsim.RosCommunicationAdapter.rospy.Subscriber')
    def test_rst_callback(self, mock_rospy_subscriber):
        rst = RosSubscribedTopic(Topic('topic_name', 'topic_type'))
        rst._callback('data')
        self.assertTrue(rst.changed)
        self.assertEquals(rst.value, 'data')
        self.assertEquals(mock_rospy_subscriber.call_count, 1)

    @patch('hbp_nrp_cle.robotsim.RosCommunicationAdapter.rospy.Subscriber')
    def test_rst_reset_changed(self, mock_rospy_subscriber):
        rst = RosSubscribedTopic(Topic('topic_name', 'topic_type'))
        rst.reset_changed()
        self.assertFalse(rst.changed)
        self.assertEquals(mock_rospy_subscriber.call_count, 1)

    @patch('hbp_nrp_cle.robotsim.RosCommunicationAdapter.rospy.Subscriber')
    def test_rst_reset(self, mock_rospy_subscriber):
        rst = RosSubscribedTopic(Topic('topic_name', 'topic_type'))
        res = rst.reset(None)
        self.assertFalse(rst.changed)
        self.assertFalse(res.changed)
        self.assertEquals(mock_rospy_subscriber.call_count, 1)

    # Tests for RosSubscribedPreprocessedTopic
    @patch('hbp_nrp_cle.robotsim.RosCommunicationAdapter.rospy.Subscriber')
    def test_rspt_init(self, mock_rospy_subscriber):
        self.assertRaises(AssertionError, RosSubscribedPreprocessedTopic, None)
        t = PreprocessedTopic('a', 'b', lambda x: ())
        rppt = RosSubscribedPreprocessedTopic(t)
        self.assertEquals(t.pre_processor, rppt._RosSubscribedPreprocessedTopic__pre_processor)
        self.assertEquals(mock_rospy_subscriber.call_count, 1)

    @patch('hbp_nrp_cle.robotsim.RosCommunicationAdapter.rospy.Subscriber')
    def test_rspt_callback(self, mock_rospy_subscriber):
        rst = RosSubscribedPreprocessedTopic(PreprocessedTopic('topic_name', 'topic_type', lambda x: x))
        rst._callback('data')
        self.assertTrue(rst.changed)
        self.assertEquals(rst.value, 'data')
        self.assertEquals(mock_rospy_subscriber.call_count, 1)

if __name__ == '__main__':
    unittest.main()