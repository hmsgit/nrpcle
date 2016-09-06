"""
Global setting of neuronal simulator. PyNNN specific devices should not depend upon a concrete
package for the neuronal simulator.
"""

__author__ = 'Sebastian Krach'

import pyNN.nest as sim

simulator = sim
