# ---LICENSE-BEGIN - DO NOT CHANGE OR MOVE THIS HEADER
# This file is part of the Neurorobotics Platform software
# Copyright (C) 2014,2015,2016,2017 Human Brain Project
# https://www.humanbrainproject.eu
#
# The Human Brain Project is a European Commission funded project
# in the frame of the Horizon2020 FET Flagship plan.
# http://ec.europa.eu/programmes/horizon2020/en/h2020-section/fet-flagships
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
# ---LICENSE-END
'''
Implementation of PyNNSpikeDetector
'''

from hbp_nrp_cle.brainsim.pynn.devices import PyNNSpikeRecorder
from hbp_nrp_cle.brainsim.pynn_nest.devices.__NestDeviceGroup import PyNNNestDevice

from collections import defaultdict
from itertools import chain
import nest
from pyNN.common import Assembly, Population
import numpy as np
import logging
from mpi4py import MPI

__author__ = 'GeorgHinkel, Igor Peric, Alina Roitberg, Sebastian Krach'
logger = logging.getLogger(__name__)


class PyNNNestSpikeRecorder(PyNNSpikeRecorder, PyNNNestDevice):
    """
    Represents a device which returns a "1" whenever one of the recorded
    neurons has spiked, otherwise a "0"
    """

    # neurons to be recorded per population base, indexed by PyNNNestSpikeRecorder instance
    _recording_neurons = defaultdict(dict)

    def __init__(self, **params):
        """
        Represents a device which returns a "1" whenever one of the recorded
        neurons has spiked, otherwise a "0"
        """
        super(PyNNNestSpikeRecorder, self).__init__(**params)
        self.__use_ids = self.get_parameters().get("use_ids")
        self.__recorders = None

    # pylint: disable=protected-access
    def _start_record_spikes(self):
        """
        Records the spikes of "neurons"
        """
        self.__recorders = []
        self._add_all_recorders(self._neurons, self.__recorders)
        population = (self._neurons
                      if isinstance(self._neurons, Population)
                      else self._neurons.grandparent)
        neuron_positions = [population.id_to_index(n_id) for n_id in self._neurons]
        # adds neurons positions to the list of population neurons to be recorded
        self._recording_neurons[population.label][id(self)] = neuron_positions
        self.__update_recording_neurons()

    def __update_recording_neurons(self):
        """
        Updates the underlying recorder status with the aggregated neurons
        to be recorded for the root population
        """
        population = (self._neurons
                      if isinstance(self._neurons, Population)
                      else self._neurons.grandparent)
        pop_neurons = list(set(chain(*self._recording_neurons[population.label].itervalues())))
        if len(pop_neurons) > 0:
            # Population recorders need to be reset before being reused
            for rec in self.__recorders:
                rec.reset()

            population[pop_neurons].record("spikes", to_file=False)

            # Even though to_file is set to False, an issue in PyNN prevent it to be applied.
            # PyNN is built in such a way that for the spikes in NEST, only the file storage
            # is available. This should be investigated.
            # Meanwhile, we are interacting with NEST directly.
            for rec in self.__recorders:
                recorder_device = rec._spike_detector.device
                self.SetStatus(recorder_device, {"to_memory": True})
                self.SetStatus(recorder_device, {"to_file": False})
        else:
            population.record(None)

    def _stop_record_spikes(self):
        """
        Stops recording the spikes of "neurons"
        """
        population = (self._neurons
                      if isinstance(self._neurons, Population)
                      else self._neurons.grandparent)
        del self._recording_neurons[population.label][id(self)]
        self.__update_recording_neurons()

    def _add_all_recorders(self, population, recorder_list):
        """
        Adds all recorders contained in the given population to the list of recorders
        :param population: The population to start with
        :param recorder_list: The list of recorders
        """
        if isinstance(population, Assembly):
            for p in population.populations:
                self._add_all_recorders(p, recorder_list)
        else:
            recorder_list.append(population.recorder)

    # simulation time not necessary for this device
    # pylint: disable=unused-argument
    def refresh(self, time):
        """
        Refreshes the recorded spikes

        :param time: The current simulation time
        """
        recorders = self.__recorders
        if len(recorders) == 1:
            spikes_nest, times_nest = self.__read_recorder_data(self._neurons.recorder)
        else:
            spikes_nest = []
            times_nest = []
            for rec in recorders:
                sp, ti = self.__read_recorder_data(rec)
                spikes_nest.append(sp)
                times_nest.append(ti)
            spikes_nest = np.concatenate(spikes_nest)
            times_nest = np.concatenate(times_nest)
        self._spikes = np.array([spikes_nest, times_nest]).T

    def __read_recorder_data(self, recorder):
        """
        Reads the recorded data of the given recorder
        :param nest_device:
        :return:
        """
        # Get the spikes directly from NEST (It let use use memory instead of files)
        # pylint: disable=protected-access
        nest_info = nest.GetStatus(recorder._spike_detector.device, 'events')[0]

        # for distrbuted Nest experiments, this direct access requires us to gather data
        # from all processes for assemble, CLE is guaranteed to be MPI process 0
        if self.mpi_aware:
            updated_info = MPI.COMM_WORLD.gather(nest_info, root=0)

            # only let the CLE continue processing
            if MPI.COMM_WORLD.Get_rank() > 0:
                return [], []

            # concatenate all of the dictionaries
            nest_info = {'times': [], 'senders': []}
            for other in updated_info:
                nest_info['times'].extend(other['times'])
                nest_info['senders'].extend(other['senders'])

        times_nest = nest_info['times']
        spikes_nest = nest_info['senders']
        if not self.__use_ids:
            for i in range(len(spikes_nest) - 1, -1, -1):
                try:
                    spikes_nest[i] = self._neurons.id_to_index(int(spikes_nest[i]))
                except IndexError:
                    del spikes_nest[i]
                    del times_nest[i]
        return spikes_nest, times_nest

    # simulation time not necessary for this device
    # pylint: disable=W0613
    # pylint: disable=protected-access
    def finalize_refresh(self, time):
        """
        Resets the number of spikes for the connected spike recorder, this is a PyNN-Nest specific
        command that clears all memory used by a recorder.

        :param time: The current simulation time
        """
        for rec in self.__recorders:
            rec._clear_simulator()
