'''
Implementation of PyNNDCSource
moduleauthor: probst@fzi.de
'''

from hbp_nrp_cle.brainsim.common.devices import AbstractBrainDevice
from hbp_nrp_cle.brainsim.BrainInterface import IDCSource
from hbp_nrp_cle.brainsim.pynn import simulator as sim

__author__ = 'DimitriProbst'


class PyNNDCSource(AbstractBrainDevice, IDCSource):
    """
    Represents a direct current generator
    """

    # pylint: disable=W0221
    def __init__(self, **params):
        """
        Initializes a direct current generator.

        :param amplitude: Amplitude of direct current, default: 1.0 nA
        :param start: Start time of current injection, default: 0.0 ms
        :param stop: Stop time of current injection, default: infinity
        """
        self._generator = None
        self.create_device(params)

    @property
    def amplitude(self):
        """
        Returns the amplitude of the current
        """
        return self._generator.amplitude

    # pylint: disable=unused-argument
    # pylint: disable=no-self-use
    @amplitude.setter
    def amplitude(self, value):
        """
        Sets the amplitude of the current

        :param value: float
        """
        raise RuntimeError("Resetting this property is currently not supported by PyNN")

    def create_device(self, params):
        """
        Create a direct current source

        :param params: Dictionary of neuron configuration parameters
        :param amplitude: Amplitude of direct current, default: 1.0 nA
        :param start: Start time of current injection, default: 0.0 ms
        :param stop: Stop time of current injection, default: infinity
        """
        self._generator = sim.DCSource(amplitude=params.get('amplitude', 1.0),
                                        start=params.get('start', 0.0),
                                        stop=params.get('stop', None))

    # No connection parameters necessary for this device
    # pylint: disable=W0613
    def connect(self, neurons, **params):
        """
        Connects the neurons specified by "neurons" to the device.

        :param neurons: must be a Population, PopulationView or
            Assembly object
        """
        self._generator.inject_into(neurons)
