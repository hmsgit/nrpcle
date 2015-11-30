"""
Module only used to represent the PyNNCommunicationAdapter and bypass importing pyNN.nest
"""

__author__ = 'GeorgHinkel'

import sys

import mock

sys.modules['pyNN'] = mock.Mock()
sys.modules['pyNN.nest'] = mock.Mock()
import hbp_nrp_cle.brainsim.pynn_nest.PyNNCommunicationAdapter as PCA
sys.modules[__name__] = PCA