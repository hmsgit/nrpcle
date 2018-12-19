"""
This package contains the simulator independent implementations for the brain adapter
"""

__author__ = "Sebastian Krach"

from .__AbstractCommunicationAdapter import AbstractCommunicationAdapter
# pylint: disable=import-error
import enum


class InternalBrainException(Exception):
    """
    Represents that an unhandled exception occurred within the brain simulation
    """

    def __init__(self, *args, **kwargs):
        super(InternalBrainException, self).__init__(*args, **kwargs)


class DeviceCommunicationDirection(enum.Enum):
    """
    Enumeration for the different types of communication direction
    """
    IN = 1
    OUT = 2
    INOUT = 3
