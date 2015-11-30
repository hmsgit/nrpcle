'''
Implementation of PyNNFixedSpikeGenerator
moduleauthor: probst@fzi.de
'''

from hbp_nrp_cle.brainsim.pynn.devices import PyNNFixedSpikeGenerator
import nest

__author__ = 'DimitriProbst, Sebastian Krach'


class PyNNNestFixedSpikeGenerator(PyNNFixedSpikeGenerator):
    """
    Represents a spike generator which generated equidistant
    spike times at a given frequency
    """

    # PyLint does not correctly recognize the overriding of the property setter
    # pylint: disable=arguments-differ
    @PyNNFixedSpikeGenerator.rate.setter
    def rate(self, value):
        """
        Sets the frequency of the Fixed spike generator

        :param value: float
        """
        self._rate, current = self._calculate_rate_and_current(value)

        if current != self._current:
            self._current = current
            # The nest device is only available as protected property of the PyNN device
            # pylint: disable=protected-access
            nest.SetStatus(self._currentsource._device, {"amplitude": 1000.0 * current})
