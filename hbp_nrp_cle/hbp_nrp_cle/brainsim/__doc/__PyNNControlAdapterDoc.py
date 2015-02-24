"""
Module only used to represent the PyNNControlAdapter and bypass importing pyNN.nest
"""

__author__ = 'GeorgHinkel'

import mock, sys
sys.modules['pyNN'] = mock.Mock()
sys.modules['pyNN.nest'] = mock.Mock()
import hbp_nrp_cle.brainsim.PyNNControlAdapter as PCA
sys.modules[__name__] = PCA