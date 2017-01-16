'''
Implementation of PyNNACSource
moduleauthor: probst@fzi.de
'''

from hbp_nrp_cle.brainsim.pynn.devices import PyNNACSource
from hbp_nrp_cle.brainsim.pynn_nest.devices.__NestDeviceGroup import PyNNNestDevice, \
    create_transformation
import nest

__author__ = 'Georg Hinkel, Dimitri Probst'


class PyNNNestACSource(PyNNNestDevice, PyNNACSource):
    """
    Represents an alternating current generator.
    """

    transformations = {
        "amplitude": create_transformation("amplitude", 1000.0),
        "offset": create_transformation("offset", 1000.0),
        "frequency": create_transformation("frequency"),
        "phase": create_transformation("phase")
    }

    # PyLint does not correctly recognize the overriding of the property setter
    # pylint: disable=arguments-differ
    @PyNNACSource.amplitude.setter
    def amplitude(self, value):
        """
        Sets the amplitude of the current

        :param value: float
        """
        if value != self._generator.amplitude:
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
        if self._generator.offset != value:
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
        if self._generator.frequency != value:
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
        if self._generator.phase != value:
            self._generator.phase = value
            # The nest device is only available as protected property of the PyNN device
            # pylint: disable=protected-access
            nest.SetStatus(self._generator._device, {'phase': float(value)})
