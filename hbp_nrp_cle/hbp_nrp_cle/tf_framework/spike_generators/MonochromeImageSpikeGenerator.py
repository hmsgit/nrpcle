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
"""
This module contains a class generating spikes for a monochrome image
"""

from hbp_nrp_cle.brainsim.BrainInterface import ICustomDevice, IBrainCommunicationAdapter, \
    IPoissonSpikeGenerator

__author__ = 'GeorgHinkel'


# The MonochromeImageSpikeGenerator was just a prototype and is therefore not tested
class MonochromeImageSpikeGenerator(ICustomDevice):  # pragma: no cover
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

    def apply(self, neurons, brain_adapter, **config):
        """
        Binds the current image spike generator

        :param neurons: The target image neurons
        :param brain_adapter: The brain communication adapter
        :param config: Additional configuration
        """
        assert isinstance(brain_adapter, IBrainCommunicationAdapter)
        if self.__width * self.__height != len(neurons):
            raise Exception(
                "The amount of assigned spikes is incorrect. A monochrome image spike generator " +
                "must be assigned to as many neurons as there are pixels in the image")

        self.__devices = brain_adapter.register_spike_source(neurons, IPoissonSpikeGenerator,
                                                             **config)

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
