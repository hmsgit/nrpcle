"""
This package contains the interface and implementation for the communication and control to the
neuronal (brain) simulator. As brain simulator, we are using PyNN.
"""

__author__ = 'GeorgHinkel'

from .BrainInterface import IBrainCommunicationAdapter, IBrainControlAdapter
