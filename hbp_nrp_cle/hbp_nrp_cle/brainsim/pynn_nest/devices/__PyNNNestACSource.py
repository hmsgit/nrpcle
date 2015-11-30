'''
Implementation of PyNNACSource
moduleauthor: probst@fzi.de
'''

from hbp_nrp_cle.brainsim.pynn.devices import PyNNACSource
import nest

__author__ = 'DimitriProbst'


class PyNNNestACSource(PyNNACSource):
    """
    Represents an alternating current generator.
    """

    # PyLint does not correctly recognize the overriding of the property setter
    # pylint: disable=arguments-differ
    @PyNNACSource.amplitude.setter
    def amplitude(self, value):
        """
        Sets the amplitude of the current

        :param value: float
        """
        self._generator.amplitude = value
        # The nest device is only available as protected property of the PyNN device
        # pylint: disable=protected-access
        nest.SetStatus(self._generator._device, {'amplitude': 1000.0 * value})

    # PyLint does not correctly recognize the overriding of the property setter
    # pylint: disable=arguments-differ
    @PyNNACSource.offset.setter
    def offset(self, value):
        """
        Sets the offset of the current

        :param value: float
        """
        self._generator.offset = value
        # The nest device is only available as protected property of the PyNN device
        # pylint: disable=protected-access
        nest.SetStatus(self._generator._device, {'offset': 1000.0 * value})

    # PyLint does not correctly recognize the overriding of the property setter
    # pylint: disable=arguments-differ
    @PyNNACSource.frequency.setter
    def frequency(self, value):
        """
        Sets the frequency of the current

        :param value: float
        """
        self._generator.frequency = value
        # The nest device is only available as protected property of the PyNN device
        # pylint: disable=protected-access
        nest.SetStatus(self._generator._device, {'frequency': float(value)})

    # PyLint does not correctly recognize the overriding of the property setter
    # pylint: disable=arguments-differ
    @PyNNACSource.phase.setter
    def phase(self, value):
        """
        Sets the phase of the current

        :param value: float
        """
        self._generator.phase = value
        # The nest device is only available as protected property of the PyNN device
        # pylint: disable=protected-access
        nest.SetStatus(self._generator._device, {'phase': float(value)})
