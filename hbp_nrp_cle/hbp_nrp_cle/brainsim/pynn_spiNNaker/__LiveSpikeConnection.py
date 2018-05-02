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
This package contains the brain adapter implementation relying on the PyNN neuronal simulator
abstraction layer but contains implementation specifics for the SpiNNaker simulator.
"""

__author__ = "Georg Hinkel"

from hbp_nrp_cle.brainsim.pynn_spiNNaker import spynnaker
import logging

logger = logging.getLogger(__name__)

default_receiver = None
default_sender = None
default_poisson = None

send_callbacks = {}
receive_callbacks = {}
poisson_callbacks = {}

RECEIVE_PORT = 19996
SEND_PORT = 19999
POISSON_PORT = 19997

next_port = 12345


def create_and_start_connections():
    """
    Creates the live spikes connections, if necessary
    """
    # pylint: disable=global-statement
    global default_receiver
    global default_sender
    global default_poisson
    if default_sender is None and len(send_callbacks) > 0:
        default_sender = spynnaker.external_devices.SpynnakerLiveSpikesConnection(
            receive_labels=None, local_port=SEND_PORT, send_labels=list(send_callbacks)
        )
        for label in send_callbacks:
            for callback in send_callbacks[label]:
                default_sender.add_start_resume_callback(label, callback)
    if default_receiver is None and len(receive_callbacks) > 0:
        default_receiver = spynnaker.external_devices.SpynnakerLiveSpikesConnection(
            receive_labels=list(receive_callbacks), local_port=RECEIVE_PORT, send_labels=None
        )
        for label in receive_callbacks:
            for callback in receive_callbacks[label]:
                default_receiver.add_receive_callback(label, callback)
    if default_poisson is None and len(poisson_callbacks) > 0:
        default_poisson = spynnaker.external_devices.SpynnakerPoissonControlConnection(
            poisson_labels=list(poisson_callbacks), local_port=POISSON_PORT
        )
        for label in poisson_callbacks:
            for callback in poisson_callbacks[label]:
                default_poisson.add_start_callback(label, callback)


def get_port(desired_port=None):
    """
    Gets a free port, perhaps the desired one
    :param desired_port: The desired port
    :return: A port not yet taken by other requests
    """
    # pylint: disable=global-statement
    global next_port
    if desired_port is None:
        port = next_port
    elif desired_port < next_port:
        port = next_port
        logger.warning("Port {} is potentially used".format(desired_port))
    else:
        port = desired_port
    next_port = port + 1
    return port


def __register_callback(label, repo, callback):
    """
    Registers the callback

    :param label: Label under which the callback is stored
    :param repo: The repo where to register the callback
    :param callback: The callback function
    """
    try:
        callbacks = repo[label]
    except KeyError:
        callbacks = []
        repo[label] = callbacks
    callbacks.append(callback)


def register_receiver(label, receive_callback):
    """
    Registers a receive callback for the provided label

    :param label: The label that should be used for the receiver
    :param receive_callback: A callback that should be called whenever a new spike is received
    """
    __register_callback(label, receive_callbacks, receive_callback)


def register_sender(label, init_callback):
    """
    Registers a send callback for the provided label

    :param label: The label that should be used for the sender
    :param init_callback: A callback that should be called when the connection is established
    """
    __register_callback(label, send_callbacks, init_callback)


def register_poisson(label, init_callback):
    """
    Registers a poisson rate callback for the given label

    :param label: The label
    :param init_callback: A callback that should be called when the connection is established
    """
    __register_callback(label, poisson_callbacks, init_callback)


def shutdown():
    """
    Clears the registered callbacks
    """
    # pylint: disable=global-statement
    global default_sender
    global default_receiver
    global default_poisson
    receive_callbacks.clear()
    send_callbacks.clear()
    poisson_callbacks.clear()
    if default_sender is not None:
        default_sender.close()
        default_sender = None
    if default_receiver is not None:
        default_receiver.close()
        default_receiver = None
    if default_poisson is not None:
        default_poisson.close()
        default_poisson = None
    # pylint: disable=global-statement
    global next_port
    next_port = 12345
