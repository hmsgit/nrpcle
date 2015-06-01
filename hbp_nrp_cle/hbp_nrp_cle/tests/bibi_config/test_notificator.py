"""
Notificator unit test
"""

import unittest
from mock import MagicMock
from hbp_nrp_cle.bibi_config.notificator import Notificator


class TestNotificator(unittest.TestCase):

    def test_notificator(self):
        notify = MagicMock()
        Notificator.register_notification_function(notify)
        Notificator.notify('test_message', 'test_update')
        self.assertEqual(notify.call_count, 1)


if __name__ == '__main__':
    unittest.main()