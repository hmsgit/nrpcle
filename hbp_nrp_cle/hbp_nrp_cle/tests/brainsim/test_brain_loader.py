"""
brain_loader unit test
"""

from hbp_nrp_cle.brainsim import BrainLoader

import unittest
import os

__author__ = 'Lorenzo Vannucci'


# pylint: disable=R0904
# all the methods are inherited from unittest.TestCase
class TestClosedLoopEngine(unittest.TestCase):
    """
    Tests the brain_loader utility
    """

    # pylint: disable=R0201
    # method has to be a method in order to be run as part of the test suite
    def test_load_h5_network(self):
        """
        Test loading an .h5 file.
        """
        directory = os.path.split(__file__)[0]
        filename = os.path.join(directory, 'braitenberg.h5')
        BrainLoader.load_h5_network(filename, {'sensors': [0, 1, 2], 'actors': [3, 4, 5]})


if __name__ == '__main__':
    unittest.main()
