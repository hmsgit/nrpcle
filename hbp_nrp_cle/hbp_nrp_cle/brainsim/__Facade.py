"""
This module provides interface functionality abstracting from different implementations of the
brain adapter. Concrete implementations should not be instantiated directly but through the
provided functionality in this module.

The purpose of the abstraction is to facilitate later exchanges of the brain adapter implementation.
"""

__author__ = "Sebastian Krach"

from . import config

from hbp_nrp_cle.brainsim.pynn.PyNNCommunicationAdapter import PyNNCommunicationAdapter
from hbp_nrp_cle.brainsim.pynn.PyNNControlAdapter import PyNNControlAdapter

DEFAULT_COMMUNICATION_ADAPTER = PyNNCommunicationAdapter
DEFAULT_CONTROL_ADAPTER = PyNNControlAdapter


def initialize_defaults():
    """
    Sets the brain simulator config to the default PyNN implementation
    """

    config.communication_adapter_type = DEFAULT_COMMUNICATION_ADAPTER
    config.control_adapter_type = DEFAULT_CONTROL_ADAPTER


def instantiate_communication_adapter():
    """
    Instantiates a new IBrainCommunicationAdapter according to the currently configured brain
    simulator adapter implementation.

    :return: an IBrainCommunicationAdapter instance
    """
    if not config.communication_adapter_type:
        initialize_defaults()
    return config.communication_adapter_type()


def instantiate_control_adapter():
    """
    Instantiates a new IBrainControlAdapter according to the currently configured brain
    simulator adapter implementation.

    :return: an IBrainControlAdapter instance
    """
    if not config.control_adapter_type:
        initialize_defaults()
    return config.control_adapter_type()
