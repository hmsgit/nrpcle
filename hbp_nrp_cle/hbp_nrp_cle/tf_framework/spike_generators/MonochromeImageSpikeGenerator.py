"""
This module contains a class generating spikes for a monochrome image
"""

from hbp_nrp_cle.brainsim.BrainInterface import ICustomDevice, IBrainCommunicationAdapter, \
    IPoissonSpikeGenerator

__author__ = 'GeorgHinkel'


class MonochromeImageSpikeGenerator(ICustomDevice):
    """
    A spike generator device that transforms camera images into spikes
    """

    def __init__(self, width, height):
        """
        Initializes a new camera spike generator
        :param width: The image width
        :param height: The image height
        """
        self.__width = width
        self.__height = height
        self.__devices = None

    def apply(self, neurons, brain_adapter):
        """
        Binds the current image spike generator
        :param neurons: The target image neurons
        :param brain_adapter: The brain communication adapter
        """
        assert isinstance(brain_adapter, IBrainCommunicationAdapter)
        if self.__width * self.__height != len(neurons):
            raise Exception(
                "The amount of assigned spikes is incorrect. A monochrome image spike generator " +
                "must be assigned to as many neurons as there are pixels in the image")

        self.__devices = brain_adapter.register_spike_source(neurons, IPoissonSpikeGenerator)

    def update_image(self, image):
        """
        Updates the image for this device
        :param image: The image to be processed
        """
        # TODO: Implement
        # self.__devices[7].rate = 4.5
        pass

    def reset(self, transfer_function_manager):
        """
        Resets the device
        :param transfer_function_manager: The transfer function manager the device belongs to
        :return: The reset adapter
        """
        self.__devices = self.__devices.reset(transfer_function_manager)
