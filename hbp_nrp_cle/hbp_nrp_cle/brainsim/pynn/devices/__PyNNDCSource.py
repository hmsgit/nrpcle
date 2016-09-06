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

    default_parameters = {
        "amplitude": 1.0,
        "start": 0.0,
        "stop": float("inf")
    }

    # pylint: disable=W0221
    def __init__(self, **params):
        """
        Initializes a direct current generator.

        :param amplitude: Amplitude of direct current, default: 1.0 nA
        :param start: Start time of current injection, default: 0.0 ms
        :param stop: Stop time of current injection, default: infinity
        """

        super(PyNNDCSource, self).__init__(**params)
        self._generator = None
        self.create_device()

    @property
    def amplitude(self):
        """
        Returns the amplitude of the current
        """
        return self._generator.amplitude

    @amplitude.setter
    def amplitude(self, value):  # pragma: no cover
        """
        Sets the amplitude of the current

        :param value: float
        """

        self._generator.set(amplitude=value)

    def create_device(self):
        """
        Create a direct current source
        """

        self._generator = sim.DCSource(**self.get_parameters("amplitude",
                                                             "start",
                                                             "stop"))

    def connect(self, neurons):
        """
        Connects the neurons specified by "neurons" to the device.

        :param neurons: must be a Population, PopulationView or
            Assembly object
        """
        self._generator.inject_into(neurons)
