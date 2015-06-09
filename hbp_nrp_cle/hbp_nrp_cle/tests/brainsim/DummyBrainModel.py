"""
This is a stupid mini demo brain model. It is not actually loading anything useful
"""

import pyNN.nest as sim

__author__ = 'GeorgHinkel'

circuit = sim.Population(3, sim.EIF_cond_alpha_isfa_ista)