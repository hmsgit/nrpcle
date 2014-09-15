'''
PyNNCommunicationAdapter.py
moduleauthor: probst@fzi.de
'''

from brainsim.BrainInterface import IBrainCommunicationAdapter, \
    IIFCurrAlpha, IPoissonSpikeGenerator, IDCSource, IACSource, INCSource
from brainsim.devices.PyNNPoissonSpikeGenerator import \
    PyNNPoissonSpikeGenerator
from brainsim.devices.PyNNDCSource import PyNNDCSource
from brainsim.devices.PyNNACSource import PyNNACSource
from brainsim.devices.PyNNNCSource import PyNNNCSource
from brainsim.devices.PyNNIFCurrAlpha import PyNNIFCurrAlpha

__author__ = 'DimitriProbst'


class PyNNCommunicationAdapter(IBrainCommunicationAdapter):
    """
    Represents the communication adapter to the neuronal simulator
    """

    def __init__(self):
        """
        Initializes the communication adapter
        """
        self.__generator_types = {IPoissonSpikeGenerator:
                                  PyNNPoissonSpikeGenerator,
                                  IDCSource: PyNNDCSource,
                                  IACSource: PyNNACSource,
                                  INCSource: PyNNNCSource}
        self.__detector_types = {IIFCurrAlpha: PyNNIFCurrAlpha}
        self.detectors = []

    def register_spike_source(self, neurons, spike_generator_type, params,
                              **connparams):
        """
        Requests a communication object with the given spike generator type
        for the given set of neurons
        :param neurons: A reference to the neurons to which the spike generator
        should be connected
        :param spike_generator_type: A spike generator type (see documentation
        or a list of allowed values)
        :param kwargs: A dictionary of configuration parameters
        :return: A communication object
        """
        generator = self.__generator_types[spike_generator_type](params)
        generator.connect(neurons, **connparams)
        return generator

    def register_spike_sink(self, neurons, spike_detector_type, params,
                            **connparams):
        '''
        Requests a communication object with the given spike detector type
        for the given set of neurons
        :param neurons: A reference to the neurons which should be connected
        to the spike detector
        :param spike_detector_type: A spike detector type (see documentation
        for a list of allowed values)
        :param kwargs: A dictionary of configuration parameters
        :return: A Communication object
        '''
        detector = self.__detector_types[spike_detector_type](params)
        self.detectors.append(detector)
        detector.connect(neurons, **connparams)
        return detector

    def refresh_buffers(self):
        """
        Refreshes all detector buffers
        """
        for det in self.detectors:
            det.refresh()
