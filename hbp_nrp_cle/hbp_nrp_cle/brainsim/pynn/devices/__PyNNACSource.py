'''
Implementation of PyNNACSource
moduleauthor: probst@fzi.de
'''

from hbp_nrp_cle.brainsim.common.devices import AbstractBrainDevice
from hbp_nrp_cle.brainsim.BrainInterface import IACSource
from hbp_nrp_cle.brainsim.pynn import simulator as sim

__author__ = 'DimitriProbst, Sebastian Krach'


class PyNNACSource(AbstractBrainDevice, IACSource):
    """
    Represents an alternating current generator.
    """

    default_parameters = {
        "amplitude": 1.0,
        "offset": 0.0,
        "frequency": 10.0,
        "phase": 0.0,
        "start": 0.0,
        "stop": float("inf")
    }

    # pylint: disable=W0221
    def __init__(self, **params):
        """
        Initializes an alternating current generator.

        :param amplitude: Amplitude of alternating current, default: 1.0 nA
        :param offset: Offset of alternating current, default: 0.0 nA
        :param frequency: Frequency of alternating current, default: 10 Hz
        :param phase: Phase of alternating current, default: 0.0
        :param start: Start time of current injection, default: 0.0 ms
        :param stop: Stop time of current injection, default: infinity
        """

        super(PyNNACSource, self).__init__(**params)

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

    @property
    def offset(self):
        """
        Returns the offset of the current
        """
        return self._generator.offset

    @offset.setter
    def offset(self, value):  # pragma: no cover
        """
        Sets the offset of the current

        :param value: float
        """
        self._generator.set(offset=value)

    @property
    def frequency(self):
        """
        Returns the frequency of the current
        """
        return self._generator.frequency

    @frequency.setter
    def frequency(self, value):  # pragma: no cover
        """
        Sets the frequency of the current

        :param value: float
        """
        self._generator.set(frequency=value)

    @property
    def phase(self):
        """
        Returns the phase of the current
        """
        return self._generator.phase

    @phase.setter
    def phase(self, value):  # pragma: no cover
        """
        Sets the phase of the current

        :param value: float
        """
        self._generator.set(phase=value)

    def create_device(self):
        """
        Creates an alternating current source
        """

        self._generator = sim.ACSource(**self.get_parameters("amplitude",
                                                             "offset",
                                                             "frequency",
                                                             "phase",
                                                             "start",
                                                             "stop"))

    def connect(self, neurons):
        """
        Connects the neurons specified by "neurons" to the device.

        :param neurons: must be a Population, PopulationView or
            Assembly object
        """
        self._generator.inject_into(neurons)

    def _disconnect(self):
        """
        Disconnects the device by setting all output to 0 since we cannot delete the device or
        connection directly via PyNN.
        """
        if self._generator:
            self.amplitude = 0.0
            self.offset = 0.0
            self.frequency = 0.0
            self.phase = 0.0
            self._generator = None
