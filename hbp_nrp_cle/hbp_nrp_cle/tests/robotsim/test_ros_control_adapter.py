from mock import patch, PropertyMock, Mock
from hbp_nrp_cle.robotsim.RosControlAdapter import RosControlAdapter

import unittest
from testfixtures import LogCapture

__author__ = 'Lorenzo Vannucci'


class TestRosControlAdapter(unittest.TestCase):

    def setUp(self):
        with LogCapture('hbp_nrp_cle.robotsim.RosControlAdapter') as l:
            with patch('hbp_nrp_cle.robotsim.RosCommunicationAdapter.rospy.wait_for_service') as _:
                with patch('hbp_nrp_cle.robotsim.GazeboLoadingHelper.rospy.ServiceProxy') as _:
                    self._rca = RosControlAdapter()
                    self._rca.initialize()

        l.check(('hbp_nrp_cle.robotsim.RosControlAdapter', 'INFO',
                 'Robot control adapter initialized'))

    def test_time_step(self):
        with LogCapture('hbp_nrp_cle.robotsim.RosControlAdapter') as logcapture:
            self._rca._RosControlAdapter__set_physics_properties = Mock(return_value=True)
            self.assertTrue(self._rca.set_time_step(0.01))
            self.assertEquals(self._rca.time_step, 0.01)

            self._rca._RosControlAdapter__set_physics_properties = Mock(return_value=False)
            self.assertFalse(self._rca.set_time_step(42))
            self.assertEquals(self._rca.time_step, 0.01)
            logcapture.check(('hbp_nrp_cle.robotsim.RosControlAdapter', 'INFO',
                              'new time step = 0.010000'),
                             ('hbp_nrp_cle.robotsim.RosControlAdapter', 'WARNING',
                              'impossible to set the new time step'))

            self.assertEquals(self._rca._RosControlAdapter__get_physics_properties.call_count, 4)

    def test_is_paused(self):
        type(self._rca._RosControlAdapter__get_physics_properties.return_value).pause = PropertyMock(return_value=True)
        self.assertTrue(self._rca.is_paused)

        type(self._rca._RosControlAdapter__get_physics_properties.return_value).pause = PropertyMock(return_value=False)
        self.assertFalse(self._rca.is_paused)

    def test_is_alive(self):
        type(self._rca._RosControlAdapter__get_world_properties.return_value).success = PropertyMock(return_value=True)
        self.assertTrue(self._rca.is_alive)

        type(self._rca._RosControlAdapter__get_world_properties.return_value).success = PropertyMock(return_value=False)
        self.assertFalse(self._rca.is_alive)

    def test_run_step(self):
        with LogCapture('hbp_nrp_cle.robotsim.RosControlAdapter') as logcapture:
            self._rca._RosControlAdapter__set_physics_properties = Mock(return_value=True)
            self._rca.set_time_step(0.01)
            with self.assertRaises(ValueError):
                self._rca.run_step(0.0001)
            logcapture.check(('hbp_nrp_cle.robotsim.RosControlAdapter', 'INFO',
                              'new time step = 0.010000'),
                             ('hbp_nrp_cle.robotsim.RosControlAdapter', 'ERROR',
                              'dt is not multiple of the physics time step'))

    def test_reset(self):
        with LogCapture('hbp_nrp_cle.robotsim.RosControlAdapter') as logcapture:
            self._rca._RosControlAdapter__set_physics_properties = Mock(return_value=True)
            self._rca.set_time_step(0.01)
            self._rca.run_step(0.05)
            self._rca.reset()
            logcapture.check(('hbp_nrp_cle.robotsim.RosControlAdapter', 'INFO',
                              'new time step = 0.010000'),
                             ('hbp_nrp_cle.robotsim.RosControlAdapter', 'DEBUG',
                              'Advancing simulation'),
                             ('hbp_nrp_cle.robotsim.RosControlAdapter', 'INFO',
                              'Resetting the world simulation'))

    """
    def test_pause(self):
        class Object(object):
            pass

        gpp = Object()
        with LogCapture('hbp_nrp_cle.robotsim.RosControlAdapter') as logcapture:
            self._rca._RosControlAdapter__set_physics_properties = Mock(return_value=True)
            self._rca.set_time_step(0.01)
            pastTime = self._rca.run_step(0.1)
            self._rca.unpause()
            gpp.pause = False
            self._rca._RosControlAdapter__get_physics_properties.return_value = gpp
            self.assertFalse(self._rca.is_paused)
            self._rca.pause()
            gpp.pause = True
            self._rca._RosControlAdapter__get_physics_properties.return_value = gpp
            self.assertTrue(self._rca.is_paused)
            self._rca.run_step(0.05)
            logcapture.check(('hbp_nrp_cle.robotsim.RosControlAdapter', 'DEBUG',
                              'Advancing simulation'),
                             ('hbp_nrp_cle.robotsim.RosControlAdapter', 'INFO',
                              'Unpausing the world simulation'),
                             ('hbp_nrp_cle.robotsim.RosControlAdapter', 'INFO',
                              'Pausing the world simulation'),
                             ('hbp_nrp_cle.robotsim.RosControlAdapter', 'DEBUG',
                              'Advancing simulation'))
    """


if __name__ == '__main__':
    unittest.main()
