'''
Implementation of MockSpikeDetector
moduleauthor: Michael.Weber@fzi.de
'''

from hbp_nrp_cle.brainsim.BrainInterface import ISpikeRecorder
import warnings

__author__ = 'MichaelWeber'


class MockSpikeRecorder(ISpikeRecorder):
    """
    Represents a device which returns a "1" whenever one of the recorded
    neurons has spiked, otherwise a "0"
    """

    def __init__(self, **params):
        """
        Initializes the neuron whose membrane potential is to be read out.
        The obligatory threshold voltage 'v_thresh' is set to "infinity"
        is set to infinity by default in order to forbid the neuron to elicit
        spikes.
        :param params: Dictionary of neuron configuration parameters
        """
        self.__spiked = False
        self.__update = params.get('updates', [])

    @property
    def spiked(self):
        """
        Returns the membrane voltage of the cell
        """
        return self.__spiked

    def refresh(self, time):
        """
        Refreshes the voltage value
        :param time: The current simulation time
        """
        if hasattr(self.__update, '__getitem__'):
            while len(self.__update) > 0 and isinstance(self.__update[0], tuple)\
                    and time >= self.__update[0][0]:
                self.__spiked = self.__update[0][1]
                self.__update = self.__update[1:]
        else:
            warnings.warn("Updates schedules must be sorted lists of tuples")

    @property
    def updates(self):
        """
        Gets the scheduled updates for this device
        :return:
        """
        return self.__update

    @updates.setter
    def updates(self, updates):
        """
        Sets the scheduled updates for this device
        :param updates:
        """
        self.__update = updates
        if updates is None:
            self.__update = []
