'''
Implementation of PyNNACSource
moduleauthor: probst@fzi.de
'''

from ..BrainInterface import IACSource
import pyNN.nest as sim

__author__ = 'DimitriProbst'


class PyNNACSource(IACSource):
    """
    Represents an alternating current generator.
    """

    # pylint: disable=W0221
    def __init__(self, params):
        """
        Initializes an alternating current generator.

        :param params: Dictionary of neuron configuration parameters
        :param amplitude: Amplitude of alternating current, default: 1.0 nA
        :param offset: Offset of alternating current, default: 0.0 nA
        :param frequency: Frequency of alternating current, default: 10 Hz
        :param phase: Phase of alternating current, default: 0.0
        :param start: Start time of current injection, default: 0.0 ms
        :param stop: Stop time of current injection, dafault: infinity
        """
        self.__generator = None

        self.create_device(params)

    @property
    def amplitude(self):
        """
        Returns the amplitude of the current
        """
        return self.__generator.amplitude

    @amplitude.setter
    def amplitude(self, value):
        """
        Sets the amplitude of the current

        :param value: float
        """
        self.__generator.amplitude = value

    @property
    def offset(self):
        """
        Returns the offset of the current
        """
        return self.__generator.offset

    @offset.setter
    def offset(self, value):
        """
        Sets the offset of the current

        :param value: float
        """
        self.__generator.offset = value

    @property
    def frequency(self):
        """
        Returns the frequency of the current
        """
        return self.__generator.frequency

    @frequency.setter
    def frequency(self, value):
        """
        Sets the frequency of the current

        :param value: float
        """
        self.__generator.frequency = value

    @property
    def phase(self):
        """
        Returns the phase of the current
        """
        return self.__generator.phase

    @phase.setter
    def phase(self, value):
        """
        Sets the phase of the current

        :param value: float
        """
        self.__generator.phase = value

    def create_device(self, params):
        """
        Creates an alternating current source

        :param params: Dictionary of neuron configuration parameters
        :param amplitude: Amplitude of alternating current, default: 1.0 nA
        :param offset: Offset of alternating current, default: 0.0 nA
        :param frequency: Frequency of alternating current, default: 10 Hz
        :param phase: Phase of alternating current, default: 0.0
        :param start: Start time of current injection, default: 0.0 ms
        :param stop: Stop time of current injection, dafault: infinity
        """
        self.__generator = sim.ACSource(amplitude=params.get('amplitude', 1.0),
                                        offset=params.get('offset', 0.0),
                                        frequency=params.get('frequency',
                                                             10.0),
                                        phase=params.get('phase', 0.0),
                                        start=params.get('start', 0.0),
                                        stop=params.get('stop', None))

    # No connection parameters necessary for this device
    # pylint: disable=W0613
    def connect(self, neurons, params):
        """
        Connects the neurons specified by "neurons" to the device.

        :param neurons: must be a Population, PopulationView or
            Assembly object
        """
        self.__generator.inject_into(neurons)
