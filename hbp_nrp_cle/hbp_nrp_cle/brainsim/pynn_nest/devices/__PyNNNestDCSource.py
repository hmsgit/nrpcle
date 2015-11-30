'''
Implementation of PyNNDCSource
moduleauthor: probst@fzi.de
'''

from hbp_nrp_cle.brainsim.pynn.devices import PyNNDCSource
import nest

__author__ = 'DimitriProbst, Sebastian Krach'


class PyNNNestDCSource(PyNNDCSource):
    """
    Represents a direct current generator
    """

    # PyLint does not correctly recognize the overriding of the property setter
    # pylint: disable=arguments-differ
    @PyNNDCSource.amplitude.setter
    def amplitude(self, value):
        """
        Sets the amplitude of the current

        :param value: float
        """
        if self._generator.amplitude != value:
            self._generator.amplitude = value
            # The nest device is only available as protected property of the PyNN device
            # pylint: disable=protected-access
            nest.SetStatus(self._generator._device, {"amplitude": 1000.0 * value})
