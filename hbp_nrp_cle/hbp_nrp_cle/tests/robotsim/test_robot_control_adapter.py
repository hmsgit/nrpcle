from hbp_nrp_cle.robotsim.RosControlAdapter import RosControlAdapter

import unittest

__author__ = 'Lorenzo Vannucci'


class TestRosControlAdapter(unittest.TestCase):

    def setUp(self):
        self._rca = RosControlAdapter()
        self._rca.initialize()

    def test_time_step(self):
        self.assertTrue(self._rca.set_time_step(0.01))
        self.assertEqual(self._rca.time_step, 0.01)

    def test_is_paused(self):
        self.assertTrue(self._rca.is_paused)

    def test_is_alive(self):
        self.assertTrue(self._rca.is_alive)

    def test_run_step(self):
        self.assertEqual(self._rca.run_step(0.05), 0.05)
        with self.assertRaises(ValueError):
            self._rca.run_step(0.0001)

    def test_reset(self):
        self._rca.run_step(0.05)
        self._rca.reset()
        self.assertEqual(self._rca.run_step(0.05), 0.05)

#     def test_shutdown(self):
#         self._rca.shutdown()


if __name__ == '__main__':
    unittest.main()
