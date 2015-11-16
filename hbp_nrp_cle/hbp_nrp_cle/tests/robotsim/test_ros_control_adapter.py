from mock import patch, PropertyMock, Mock, MagicMock
from hbp_nrp_cle.robotsim.RosControlAdapter import RosControlAdapter
import unittest
from testfixtures import LogCapture

__author__ = 'Lorenzo Vannucci, Sebastian Krach'


class TestRosControlAdapter(unittest.TestCase):

    def select_mock(self, *args, **kwargs):
        mock = {
            'gazebo/get_physics_properties': self.__get_physics,
            'gazebo/get_world_properties': self.__get_world,
            'gazebo/set_physics_properties': self.__set_physics,
            'gazebo/pause_physics': self.__pause_physics,
            'gazebo/unpause_physics': self.__unpause_physics,
            'gazebo/reset_sim': self.__reset_sim,
            'gazebo/end_world': self.__end_world,
        }[args[0]]

        for key, value in kwargs.items():
            setattr(mock, key, value)

        return mock

    @patch('hbp_nrp_cle.robotsim.RosControlAdapter.AsynchonousRospyServiceProxy')
    @patch('hbp_nrp_cle.robotsim.RosCommunicationAdapter.rospy.wait_for_service')
    @patch('hbp_nrp_cle.robotsim.RosCommunicationAdapter.rospy.ServiceProxy')
    def setUp(self, rospy_proxy, rospy_wfs, async_proxy_mock):
        self.__advance_simulation = MagicMock()
        self.__get_physics = MagicMock()
        self.__get_world = MagicMock()
        self.__set_physics = MagicMock()
        self.__pause_physics = MagicMock()
        self.__unpause_physics = MagicMock()
        self.__reset_sim = MagicMock()
        self.__end_world = MagicMock()

        async_proxy_mock.return_value = self.__advance_simulation

        rospy_proxy.side_effect = self.select_mock

        self._rca = RosControlAdapter()
        self._rca.initialize()

    @patch('hbp_nrp_cle.robotsim.RosControlAdapter.AsynchonousRospyServiceProxy')
    @patch('hbp_nrp_cle.robotsim.RosCommunicationAdapter.rospy.wait_for_service')
    @patch('hbp_nrp_cle.robotsim.RosCommunicationAdapter.rospy.ServiceProxy')
    def test_initialize(self, rospy_proxy, rospy_wfs, async_proxy_mock):

        self.__pause_physics = MagicMock()
        self.__get_physics = MagicMock()
        physics_mock = MagicMock(pause=False, time_step=1.0)
        physics_mock.pause = False
        physics_mock.time_step = 1.0
        self.__get_physics.return_value = physics_mock

        rospy_proxy.side_effect = self.select_mock

        with LogCapture('hbp_nrp_cle.robotsim.RosControlAdapter') as l:
            rca = RosControlAdapter()
            self.assertFalse(self.__pause_physics.called)
            rca.initialize()
            self.assertTrue(self.__pause_physics.called)
            self.assertEqual(rca.time_step, 1.0)

            l.check(('hbp_nrp_cle.robotsim.RosControlAdapter', 'INFO',
                     'Robot control adapter initialized'))

    def test_time_step(self):
        with LogCapture('hbp_nrp_cle.robotsim.RosControlAdapter') as logcapture:
            self.assertEquals(self.__get_physics.call_count, 1)
            self.__set_physics.return_value=True

            self.assertTrue(self._rca.set_time_step(0.01))
            self.assertEquals(self._rca.time_step, 0.01)

            self.__set_physics.return_value=False
            self.assertFalse(self._rca.set_time_step(42))
            self.assertEquals(self._rca.time_step, 0.01)
            logcapture.check(('hbp_nrp_cle.robotsim.RosControlAdapter', 'INFO',
                              'new time step = 0.010000'),
                             ('hbp_nrp_cle.robotsim.RosControlAdapter', 'WARNING',
                              'impossible to set the new time step'))

            self.assertEquals(self.__get_physics.call_count, 3)

    def test_is_paused(self):
        self.__get_physics.return_value.pause = True
        self.assertTrue(self._rca.is_paused)

        self.__get_physics.return_value.pause = False
        self.assertFalse(self._rca.is_paused)

    def test_is_alive(self):
        self.__get_world.return_value.success = True
        self.assertTrue(self._rca.is_alive)

        self.__get_world.return_value.success = False
        self.assertFalse(self._rca.is_alive)

    def test_run_step_logging(self):
        with LogCapture('hbp_nrp_cle.robotsim.RosControlAdapter') as logcapture:
            self.__set_physics.return_value = True
            self._rca.set_time_step(0.01)
            with self.assertRaises(ValueError):
                self._rca.run_step(0.0001)
            logcapture.check(('hbp_nrp_cle.robotsim.RosControlAdapter', 'INFO',
                              'new time step = 0.010000'),
                             ('hbp_nrp_cle.robotsim.RosControlAdapter', 'ERROR',
                              'dt is not multiple of the physics time step'))

    def test_run_step(self):
        future_mock = MagicMock()
        self.__advance_simulation.return_value = future_mock
        future_mock.result.return_value = 100

        self._rca.set_time_step(0.01)

        result = self._rca.run_step(0.01)
        self.__advance_simulation.assert_called_with(1.0)

        self.assertEqual(result, 100)

    def test_run_step_async(self):
        future_mock = MagicMock()
        self.__advance_simulation.return_value = future_mock
        self._rca.set_time_step(0.01)
        result = self._rca.run_step_async(0.01)
        self.assertEqual(result, future_mock)
        self.__advance_simulation.assert_called_with(1.0)

    def test_reset(self):
        with LogCapture('hbp_nrp_cle.robotsim.RosControlAdapter') as logcapture:
            self.__set_physics.return_value=True
            self._rca.set_time_step(0.01)
            self._rca.run_step(0.05)
            self._rca.reset()
            logcapture.check(('hbp_nrp_cle.robotsim.RosControlAdapter', 'INFO',
                              'new time step = 0.010000'),
                             ('hbp_nrp_cle.robotsim.RosControlAdapter', 'DEBUG',
                              'Advancing simulation'),
                             ('hbp_nrp_cle.robotsim.RosControlAdapter', 'INFO',
                              'Resetting the world simulation'))

    def test_pause(self):
        with LogCapture('hbp_nrp_cle.robotsim.RosControlAdapter') as logcapture:
            self.__set_physics.return_value=True
            self._rca.set_time_step(0.01)
            pastTime = self._rca.run_step(0.1)
            self._rca.unpause()

            gpp = MagicMock(pause=False)
            self.__get_physics.return_value = gpp
            self.assertFalse(self._rca.is_paused)

            self.assertFalse(self.__pause_physics.called)
            self._rca.pause()
            self.assertTrue(self.__pause_physics.called)

            gpp = MagicMock(pause=True)
            self.__get_physics.return_value = gpp
            self.assertTrue(self._rca.is_paused)

            self._rca.run_step(0.05)
            logcapture.check(
                ('hbp_nrp_cle.robotsim.RosControlAdapter', 'INFO',
                 'new time step = 0.010000'),
                ('hbp_nrp_cle.robotsim.RosControlAdapter', 'DEBUG',
                 'Advancing simulation'),
                ('hbp_nrp_cle.robotsim.RosControlAdapter', 'INFO',
                 'Unpausing the world simulation'),
                ('hbp_nrp_cle.robotsim.RosControlAdapter', 'INFO',
                 'Pausing the world simulation'),
                ('hbp_nrp_cle.robotsim.RosControlAdapter', 'DEBUG',
                 'Advancing simulation'))

    def test_shutdown(self):
        with LogCapture('hbp_nrp_cle.robotsim.RosControlAdapter') as logcapture:
            self._rca.shutdown()
            logcapture.check(('hbp_nrp_cle.robotsim.RosControlAdapter', 'INFO',
                              'Shutting down the world simulation'))


if __name__ == '__main__':
    unittest.main()
