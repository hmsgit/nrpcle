from python_cle.robotsim.RobotInterface import IRobotControlAdapter

import unittest

__author__ = 'LorenzoVannucci'

class TestRosControlAdapter(unittest.TestCase):

    def setUp(self):
        self._rca = RosControlAdapter()
        self._rca.initialize()
    
    def test_get_time_step(self):
        self.assertEqual(self._rca.time_step(), 0.001)
    
    def test_set_time_step(self):
        self.assertTrue(self._rca.time_step(0.01))
        self.assertEqual(self._rca.time_step(), 0.01)
    
    def test_is_paused(self):
        self.assertTrue(self._rca.is_paused())
    
    def test_is_alive(self):
        self.assertTrue(self._rca.is_alive())
        
    def run_step(self):
        self.assertEqual(self._rca.run_step(0.5), 0.5)
        self.assertRaises(self._rca.run_step(0.0001))


if __name__ == '__main__':
    suite = unittest.TestLoader().loadTestsFromTestCase(TestRosControlAdapter)
    unittest.TextTestRunner(verbosity=2).run(suite)