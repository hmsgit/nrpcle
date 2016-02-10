"""
This is just an example of a neural network
"""

__author__ = 'Georg Hinkel'

import pyNN.nest as sim

population = sim.Population(3, sim.IF_curr_alpha)
view = population[1:2]
assembly = population + view