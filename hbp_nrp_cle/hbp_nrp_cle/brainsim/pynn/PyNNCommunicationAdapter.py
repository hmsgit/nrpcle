'''
PyNNCommunicationAdapter.py
moduleauthor: probst@fzi.de
'''

import logging
from hbp_nrp_cle.brainsim.BrainInterface import ILeakyIntegratorAlpha, IPoissonSpikeGenerator, \
    IDCSource, IACSource, INCSource, ILeakyIntegratorExp, IPopulationRate, \
    IFixedSpikeGenerator, ISpikeRecorder

from hbp_nrp_cle.brainsim.common import AbstractCommunicationAdapter
from hbp_nrp_cle.brainsim.pynn.__devices import PyNNSpikeRecorder, \
    PyNNPopulationRate, PyNNACSource, PyNNDCSource, PyNNFixedSpikeGenerator, \
    PyNNLeakyIntegratorAlpha, PyNNLeakyIntegratorExp, PyNNNCSource, PyNNPoissonSpikeGenerator

logger = logging.getLogger(__name__)

__author__ = 'Dimitri Probst, Sebastian Krach'


class PyNNCommunicationAdapter(AbstractCommunicationAdapter):
    """
    Represents the communication adapter to the neuronal simulator
    """
    # In this dictionary, the association of spike generator types to classes
    # implementing their functionality is established
    __device_dict = {IFixedSpikeGenerator: PyNNFixedSpikeGenerator,
                     IPoissonSpikeGenerator: PyNNPoissonSpikeGenerator,
                     IDCSource: PyNNDCSource,
                     IACSource: PyNNACSource,
                     INCSource: PyNNNCSource,
                     ILeakyIntegratorAlpha: PyNNLeakyIntegratorAlpha,
                     ILeakyIntegratorExp: PyNNLeakyIntegratorExp,
                     IPopulationRate: PyNNPopulationRate,
                     ISpikeRecorder: PyNNSpikeRecorder}

    def initialize(self):
        """
        Marks the PyNN adapter as initialized
        """
        logger.info("PyNN communication adapter initialized")
        super(PyNNCommunicationAdapter, self).initialize()

    def _get_device_type(self, device_type):
        """
        Returns the pynn specific implementation for specified device type
        :param device_type: the device type for which the implementation type is requested
        :return: the concrete type
        """
        if device_type in self.__device_dict:
            return self.__device_dict[device_type]
        super(PyNNCommunicationAdapter, self)._get_device_type(device_type)
