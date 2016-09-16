"""
This package contains the python code to run the closed loop engine (CLE)
"""

from hbp_nrp_cle.version import VERSION as __version__  # pylint: disable=W0611

import sys

__author__ = 'GeorgHinkel'

# This is a hack to allow utf-8 encoding in Python 2.7
# This is required by Nest 2.10 and should be removed as we go for Python 3
# pylint: disable=no-member
reload(sys)
sys.setdefaultencoding('utf-8')
