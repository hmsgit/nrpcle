'''
Implementation of PyNNFixedSpikeGenerator
moduleauthor: probst@fzi.de
'''

from hbp_nrp_cle.brainsim.pynn.devices import PyNNFixedSpikeGenerator
import nest

__author__ = 'DimitriProbst'


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
        amplitude = self.set_current(value)
        self._currentsource.amplitude = amplitude
        # PyNN<0.8 does not support changing current source reconfiguration
        # pylint: disable=W0212
        nest.SetStatus(self._currentsource._device, {'amplitude': 1000.0 * amplitude})
