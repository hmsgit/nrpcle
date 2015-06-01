from hbp_nrp_cle.robotsim.LocalGazebo import LocalGazeboServerInstance

import unittest
from mock import patch, MagicMock

__author__ = 'Alessandro Ambrosano'


class TestLocalGazeboServerInstance(unittest.TestCase):

    def setUp(self):
        self.instance = LocalGazeboServerInstance()

    @patch('hbp_nrp_cle.robotsim.LocalGazebo.os')
    def test_start(self, mocked_os):
        self.instance.start('')
        mocked_os.system.assert_any_call('/etc/init.d/gzserver start')

    @patch('hbp_nrp_cle.robotsim.LocalGazebo.os')
    def test_stop(self, mocked_os):
        self.instance.stop()
        mocked_os.system.assert_any_call('/etc/init.d/gzserver stop')

    @patch('hbp_nrp_cle.robotsim.LocalGazebo.os')
    def test_restart(self, mocked_os):
        self.instance.restart('')
        mocked_os.system.assert_any_call('/etc/init.d/gzserver restart')

    @patch('hbp_nrp_cle.robotsim.LocalGazebo.os')
    def test_gazebo_master_uri(self, mocked_os):
        mocked_os.environ.get = MagicMock(return_value=None)
        self.assertIsInstance(self.instance.gazebo_master_uri, str)
        self.assertNotEqual(self.instance.gazebo_master_uri, "")
        mock_gazebo_master_uri = "http://localhost:12345"
        mocked_os.environ.get = MagicMock(return_value=mock_gazebo_master_uri)
        self.assertIsInstance(self.instance.gazebo_master_uri, str)
        self.assertEquals(self.instance.gazebo_master_uri, mock_gazebo_master_uri)

if __name__ == '__main__':
    unittest.main()
