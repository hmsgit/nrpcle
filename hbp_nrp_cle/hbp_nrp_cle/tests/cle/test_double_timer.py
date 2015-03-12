"""
Unit tests for the double timer used for the state publishing and the timeout
"""

__author__ = 'Lorenzo Vannucci'

from hbp_nrp_cle.cle.ROSCLEServer import DoubleTimer
import time

import unittest


class Mock(object):

    def __init__(self):
        self.expired = False

    def fun1(self):
        pass

    def fun2(self):
        self.expired = True


class TestDoubleTimer(unittest.TestCase):

    def test_timer(self):
        m = Mock()
        dt = DoubleTimer(1, m.fun1, 3, m.fun2)
        dt.enable_second_callback()
        self.assertEqual(dt.get_remaining_time(), 3)
        dt.start()
        time.sleep(1)
        self.assertTrue(dt.get_remaining_time() < 3)
        dt.disable_second_callback()
        self.assertEqual(dt.get_remaining_time(), 3)
        dt.enable_second_callback()
        time.sleep(4)
        self.assertTrue(m.expired)
        dt.cancel_all()

