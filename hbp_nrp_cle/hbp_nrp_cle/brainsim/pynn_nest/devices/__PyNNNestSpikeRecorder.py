'''
Implementation of PyNNSpikeDetector
'''

from hbp_nrp_cle.brainsim.common.devices import AbstractBrainDevice
from hbp_nrp_cle.brainsim.BrainInterface import ISpikeRecorder
import numpy as np
import nest
import logging

__author__ = 'GeorgHinkel, Igor Peric, Alina Roitberg'
logger = logging.getLogger(__name__)


class PyNNNestSpikeRecorder(AbstractBrainDevice, ISpikeRecorder):
    """
    Represents a device which returns a "1" whenever one of the recorded
    neurons has spiked, otherwise a "0"
    """

    # No connection parameters necessary for this device
    # pylint: disable=W0613
    # pylint: disable=W0221
    def __init__(self, **params):
        """
        Represents a device which returns a "1" whenever one of the recorded
        neurons has spiked, otherwise a "0"
        """
        self.__spikes = np.array([[], []])
        self.__neurons = None
        self.__refresh_count = 0

    @property
    def spiked(self):
        """
        Returns the recorded spikes
        "1": neuron spiked within the last time step
        "0": neuron was silent within the last time step
        """
        return self.__spikes.shape[0] > 0

    @property
    def times(self):
        """
        Returns the times and neuron IDs of the recorded spikes within the last time step.
        :return:
        """
        return self.__spikes

    # pylint: disable=protected-access
    def start_record_spikes(self):
        """
        Records the spikes of "neurons"
        """
        print "Record spike recorder"
        # Even though to_file is set to False, an issue in PyNN prevent it to be applied.
        # PyNN is built in such a way that for the spikes in NEST, only the file storage
        # is available. This should be investigated.
        # Meanwhile, we are interacting with NEST directly.

        # Fortunately for us, record implementation in PyNN will call "_reset" that will
        # remove the NEST recording device from the simulation (see src/common/populations.py
        # and src/nest/recording.py in PyNN source code)
        self.__neurons.record(to_file=False)
        nest.SetStatus(self.__neurons.recorders['spikes']._device.device, "to_memory", True)
        nest.SetStatus(self.__neurons.recorders['spikes']._device.device, "to_file", False)

    def connect(self, neurons, **params):
        """
        Connects the neurons specified by "neurons" to the
        spike recorder

        :param neurons: must be a Population, PopulationView or
            Assembly object
        """
        self.__neurons = neurons
        self.start_record_spikes()

    # simulation time not necessary for this device
    # pylint: disable=W0613
    # pylint: disable=protected-access
    def refresh(self, time):
        """
        Refreshes the voltage value

        :param time: The current simulation time
        """
        # Get the spikes directly from NEST (It let use use memory instead of files)
        nest_device = self.__neurons.recorders['spikes']._device.device
        nest_info = nest.GetStatus(nest_device, 'events')[0]
        times_nest = nest_info['times']
        spikes_nest = nest_info['senders']
        nest.SetStatus(nest_device, 'n_events', 0)
        self.__spikes = np.array([spikes_nest, times_nest]).T
