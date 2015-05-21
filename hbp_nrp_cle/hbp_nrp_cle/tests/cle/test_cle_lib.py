"""
Unit tests for the CLELib (get_local_ip)
"""

__author__ = 'Bernd Eckstein'

from hbp_nrp_cle.cle.CLELib import get_local_ip

import unittest
import mock
import commands


class TestGetLocalIp(unittest.TestCase):

    def setUp(self):
        self.timer_return = False

    def test_get_local_ip(self):

        # A regex that recognizes ip-adresses
        ip_regex = "\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}"

        # Test if the result looks like an ip
        self.assertRegexpMatches(get_local_ip(), ip_regex)


if __name__ == '__main__':
    suite = unittest.TestLoader().loadTestsFromTestCase(TestGetLocalIp)
    unittest.TextTestRunner(verbosity=2).run(suite)
