'''
Implementation of PyNNLeakyIntegratorAlpha
moduleauthor: probst@fzi.de
'''

from hbp_nrp_cle.brainsim.pynn.devices import PyNNLeakyIntegrator
from hbp_nrp_cle.brainsim.pynn.devices import PyNNLeakyIntegratorAlpha, PyNNLeakyIntegratorExp
import nest

__author__ = 'DimitriProbst'


# PyLint does not recognize the following class as abstract as it does not define new methods
# raising NotImplementedException
# pylint: disable=abstract-method
class PyNNNestLeakyIntegrator(PyNNLeakyIntegrator):
    """
    Represents the membrane potential of a current-based LIF neuron
    with alpha-shaped post synaptic currents
    """

    def start_record_voltage(self):
        # Since we get the data directly from Nest and Nest supports reading of just the latest
        # value we don't need to record the entire voltage trace.
        pass

    # simulation time not necessary for this device
    # pylint: disable=W0613
    def refresh(self, time):
        """
        Refreshes the voltage value

        :param time: The current simulation time
        """

        self._voltage = nest.GetStatus([self._cell[0]])[0]['V_m']


class PyNNNestLeakyIntegratorAlpha(PyNNNestLeakyIntegrator, PyNNLeakyIntegratorAlpha):
    """
    The Nest-specific adaption of the PyNN leaky Integrator using a current-based LIF neuron with
    alpha-shaped post synaptic currents
    """


class PyNNNestLeakyIntegratorExp(PyNNNestLeakyIntegrator, PyNNLeakyIntegratorExp):
    """
    The Nest-specific adaption of the PyNN leaky Integrator using a current-based LIF neuron with
    decaying-exponential post-synaptic currents
    """
