'''
Implementation of PyNNPopulationRate
moduleauthor: probst@fzi.de
'''

from hbp_nrp_cle.brainsim.pynn.devices import PyNNPopulationRate
import nest

__author__ = 'DimitriProbst'


class PyNNNestPopulationRate(PyNNPopulationRate):
    """
    Represents the rate of a population of LIF neurons by
    measuring and normalizing the membrane potential of a
    leaky integrator with decaying-exponential post-synaptic currents
    """

    def start_record_rate(self):
        # Since we get the data directly from Nest and Nest supports reading of just the latest
        # value we don't need to record the entire voltage trace.
        pass

    # simulation time not necessary for this device
    # pylint: disable=W0613
    def refresh(self, time):
        """
        Refreshes the rate value

        :param time: The current simulation time
        """

        self._rate = nest.GetStatus([self._cell[0]])[0]['V_m']
