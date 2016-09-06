'''
Implementation of PyNNSpikeDetector
'''

from hbp_nrp_cle.brainsim.pynn.devices import PyNNSpikeRecorder
import nest
import numpy as np
import logging

__author__ = 'GeorgHinkel, Igor Peric, Alina Roitberg, Sebastian Krach'
logger = logging.getLogger(__name__)


class PyNNNestSpikeRecorder(PyNNSpikeRecorder):
    """
    Represents a device which returns a "1" whenever one of the recorded
    neurons has spiked, otherwise a "0"
    """

    # pylint: disable=protected-access
    def _start_record_spikes(self):
        """
        Records the spikes of "neurons"
        """
        # Even though to_file is set to False, an issue in PyNN prevent it to be applied.
        # PyNN is built in such a way that for the spikes in NEST, only the file storage
        # is available. This should be investigated.
        # Meanwhile, we are interacting with NEST directly.

        # Fortunately for us, record implementation in PyNN will call "_reset" that will
        # remove the NEST recording device from the simulation (see src/common/populations.py
        # and src/nest/recording.py in PyNN source code)
        self._neurons.record("spikes", to_file=False)
        recorder_device = self._neurons.recorder._spike_detector.device
        nest.SetStatus(recorder_device, "to_memory", True)
        nest.SetStatus(recorder_device, "to_file", False)

    # simulation time not necessary for this device
    # pylint: disable=unused-argument
    def refresh(self, time):
        """
        Refreshes the recorded spikes

        :param time: The current simulation time
        """
        # Get the spikes directly from NEST (It let use use memory instead of files)
        # pylint: disable=protected-access
        nest_device = self._neurons.recorder._spike_detector.device

        nest_info = nest.GetStatus(nest_device, 'events')[0]
        times_nest = nest_info['times']
        spikes_nest = nest_info['senders']
        self._spikes = np.array([spikes_nest, times_nest]).T

    # simulation time not necessary for this device
    # pylint: disable=W0613
    # pylint: disable=protected-access
    def finalize_refresh(self, time):
        """
        Resets the number of spikes for the connected spike recorder

        :param time: The current simulation time
        """
        nest_device = self._neurons.recorder._spike_detector.device
        nest.SetStatus(nest_device, 'n_events', 0)
