"""
cle.__init__ unit test
"""
import unittest
from hbp_nrp_cle.cle import ros_handler


class TestCleInit(unittest.TestCase):
    def test_ros_decorator(self):
        @ros_handler
        def void_function():
            pass

        @ros_handler
        def returning_function():
            return 0

        self.assertEquals(void_function(), [])
        self.assertEquals(returning_function(), 0)

if __name__ == '__main__':
    unittest.main()
