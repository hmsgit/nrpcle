'''
PyNNCommunicationAdapter.py
moduleauthor: probst@fzi.de
'''

import logging
from hbp_nrp_cle.brainsim.BrainInterface import ILeakyIntegratorAlpha, \
    IDCSource, IACSource, INCSource, ILeakyIntegratorExp, IPopulationRate, \
    IFixedSpikeGenerator, ISpikeRecorder

from hbp_nrp_cle.brainsim.pynn.PyNNCommunicationAdapter import PyNNCommunicationAdapter
from .devices import PyNNNestACSource, PyNNNestDCSource, PyNNNestNCSource, \
    PyNNNestLeakyIntegratorAlpha, PyNNNestLeakyIntegratorExp, PyNNNestFixedSpikeGenerator, \
    PyNNNestPopulationRate, PyNNNestSpikeRecorder

logger = logging.getLogger(__name__)

__author__ = 'Dimitri Probst, Sebastian Krach'


class PyNNNestCommunicationAdapter(PyNNCommunicationAdapter):
    """
    Represents the communication adapter to the neuronal simulator
    """
    # In this dictionary, the association of spike generator types to classes
    # implementing their functionality is established
    __device_dict = {IFixedSpikeGenerator: PyNNNestFixedSpikeGenerator,
                     IDCSource: PyNNNestDCSource,
                     IACSource: PyNNNestACSource,
                     INCSource: PyNNNestNCSource,
                     ILeakyIntegratorAlpha: PyNNNestLeakyIntegratorAlpha,
                     ILeakyIntegratorExp: PyNNNestLeakyIntegratorExp,
                     IPopulationRate: PyNNNestPopulationRate,
                     ISpikeRecorder: PyNNNestSpikeRecorder}

    def initialize(self):
        """
        Marks the PyNN adapter as initialized
        """
        super(PyNNNestCommunicationAdapter, self).initialize()

    def _get_device_type(self, device_type):
        """
        Returns the pynn specific implementation for specified device type
        :param device_type: the device type for which the implementation type is requested
        :return: the concrete type
        """

        if device_type in self.__device_dict:
            return self.__device_dict[device_type]
        return super(PyNNNestCommunicationAdapter, self)._get_device_type(device_type)
