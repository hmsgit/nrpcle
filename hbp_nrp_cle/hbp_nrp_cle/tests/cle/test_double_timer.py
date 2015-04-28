"""
Unit tests for the double timer used for the state publishing and the timeout
"""

__author__ = 'Lorenzo Vannucci, Alessandro Ambrosano'

from hbp_nrp_cle.cle.ROSCLEServer import DoubleTimer
import time
import threading

import unittest
import mock


class TestDoubleTimer(unittest.TestCase):

    def setUp(self):
        self.timer_return = False

    def test_timer(self):
        f1 = mock.Mock()
        f2 = mock.Mock()

        # Wrong initializations
        self.assertRaises(ValueError, DoubleTimer, -1, None, 1, None)
        self.assertRaises(ValueError, DoubleTimer, 1, None, -1, None)

        def my_wait(_):
            time.sleep(0.01)
            return False

        # Testing ticks
        dt = DoubleTimer(1, f1, 3000, f2)
        dt.stopped.wait = my_wait
        dt.enable_second_callback()
        self.assertEqual(dt.get_remaining_time(), 3000)
        dt.start()
        time.sleep(1)
        self.assertTrue(dt.get_remaining_time() < 3000)
        dt.cancel_all()

        # Testing counting reset
        dt = DoubleTimer(1, f1, 3000, f2)
        dt.stopped.wait = my_wait
        dt.enable_second_callback()
        dt.start()
        time.sleep(1)
        self.assertTrue(dt.get_remaining_time() < 3000)
        dt.disable_second_callback()
        self.assertEqual(dt.get_remaining_time(), 3000)
        dt.cancel_all()

        # Testing everything is called the right number of times
        f1.call_count = 0
        f2.call_count = 0
        dt = DoubleTimer(1, f1, 3000, f2)
        dt.stopped.wait = mock.Mock(side_effect=time.sleep(0.000001), return_value=False)
        dt.enable_second_callback()
        dt.start()
        time.sleep(1)
        self.assertTrue(f1.call_count > 3000)
        self.assertEqual(f2.call_count, 1)
        dt.cancel_all()


if __name__ == '__main__':
    suite = unittest.TestLoader().loadTestsFromTestCase(TestDoubleTimer)
    unittest.TextTestRunner(verbosity=2).run(suite)
