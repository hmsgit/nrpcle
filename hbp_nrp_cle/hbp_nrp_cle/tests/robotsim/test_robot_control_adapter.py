from hbp_nrp_cle.robotsim.RosControlAdapter import RosControlAdapter

import unittest
from testfixtures import log_capture, LogCapture

__author__ = 'Lorenzo Vannucci'


class TestRosControlAdapter(unittest.TestCase):

    def setUp(self):
        with LogCapture('hbp_nrp_cle.robotsim.RosControlAdapter') as l:
            self._rca = RosControlAdapter()
            self._rca.initialize()
        l.check(('hbp_nrp_cle.robotsim.RosControlAdapter', 'INFO',
                 'Robot control adapter initialized'))

    @log_capture('hbp_nrp_cle.robotsim.RosControlAdapter')
    def test_time_step(self, logcapture):
        self.assertTrue(self._rca.set_time_step(0.01))
        self.assertEqual(self._rca.time_step, 0.01)   
        logcapture.check(('hbp_nrp_cle.robotsim.RosControlAdapter', 'INFO',
                          'new time step = 0.010000'))

    def test_is_paused(self):
        self.assertTrue(self._rca.is_paused)

    def test_is_alive(self):
        self.assertTrue(self._rca.is_alive)

    @log_capture('hbp_nrp_cle.robotsim.RosControlAdapter')
    def test_run_step(self, logcapture):
        self.assertEqual(self._rca.run_step(0.05), 0.05)
        with self.assertRaises(ValueError):
            self._rca.run_step(0.0001)
        logcapture.check(('hbp_nrp_cle.robotsim.RosControlAdapter', 'ERROR',
                          'dt is not multiple of the physics time step'))

    @log_capture('hbp_nrp_cle.robotsim.RosControlAdapter')
    def test_reset(self, logcapture):
        self._rca.run_step(0.05)
        self._rca.reset()
        self.assertEqual(self._rca.run_step(0.05), 0.05)
        logcapture.check(('hbp_nrp_cle.robotsim.RosControlAdapter', 'INFO',
                          'Resetting the world simulation'))

    @log_capture('hbp_nrp_cle.robotsim.RosControlAdapter')
    def test_pause(self, logcapture):
        pastTime = self._rca.run_step(0.1)
        self._rca.unpause()
        self.assertFalse(self._rca.is_paused)
        self.assertNotEqual(self._rca.run_step(0.1), pastTime + 0.1)
        self._rca.pause()
        self.assertTrue(self._rca.is_paused)
        pastTime = self._rca.run_step(0.05)
        self.assertEqual(round(self._rca.run_step(0.05), 3), round(pastTime + 0.05, 3))
        logcapture.check(('hbp_nrp_cle.robotsim.RosControlAdapter', 'INFO',
                          'Unpausing the world simulation'),
                         ('hbp_nrp_cle.robotsim.RosControlAdapter', 'INFO',
                          'Pausing the world simulation'))

#     def test_shutdown(self):
#         self._rca.shutdown()


if __name__ == '__main__':
    unittest.main()
