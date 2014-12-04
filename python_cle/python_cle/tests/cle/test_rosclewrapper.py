"""
ROSCLEWrapper unit test
"""

from python_cle.mocks.cle.MockClosedLoopEngine import MockClosedLoopEngine
from python_cle.cle.ROSCLEWrapper import ROSCLEClient, ROSCLEServer

import unittest
import time
import threading

__author__ = 'Lorenzo Vannucci'


class Helper(threading.Thread):
    """
    Test helper class.
    """

    def __init__(self, cle):
        """
        Create the helper.
        """
        super(Helper, self).__init__()
        self.daemon = True

        self.server = ROSCLEServer(cle)

    def run(self):
        """
        Inherited from threading.Thread, override.
        """
        self.server.main()


# pylint: disable=R0904
# all the methods are inherited from unittest.TestCase
class TestClosedLoopEngine(unittest.TestCase):
    """
    Tests the ROSCLEWrapper.
    """

    def setUp(self):
        """
        Sets up the mocked cle for the wrappers.
        """
        self.mockcle = MockClosedLoopEngine(0.1)

    def test_wrappers(self):
        """
        Test both wrappers.
        """
        server = Helper(self.mockcle)
        print "here"

        server.start()
        print "here"

        client = ROSCLEClient()

        client.start()
        time.sleep(2)
        client.pause()
        time.sleep(2)
        client.reset()
        time.sleep(2)
        client.stop()


if __name__ == '__main__':
    unittest.main()
