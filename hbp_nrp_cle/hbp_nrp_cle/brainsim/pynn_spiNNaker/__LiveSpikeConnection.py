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

send_receive_conn = None
poisson_conn = None


def __register_callback(label, repo, callback):  # pragma no cover
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


def register_receiver(neurons, receive_callback):  # pragma no cover
    """
    Sets up live reception for the given neurons with the given callback called
    whenever a spike is received.

    :param neurons: The population of neurons to receive spikes from
    :param receive_callback: A callback that should be called whenever a new \
        spike is received
    """
    # pylint: disable=global-statement
    global send_receive_conn
    if send_receive_conn is None:
        send_receive_conn = \
            spynnaker.external_devices.SpynnakerLiveSpikesConnection(local_port=None)
    spynnaker.external_devices.activate_live_output_for(
        neurons,
        database_notify_host="localhost",
        database_notify_port_num=send_receive_conn.local_port
    )
    send_receive_conn.add_receive_label(neurons.label)
    send_receive_conn.add_receive_callback(neurons.label, receive_callback)


def register_sender(neurons, init_callback, source, receptor_type, connector,
                    synapse_type):  # pragma no cover
    """
    Sets up live sending to the given neurons, with the given callback called
    when the simulation is ready to be sent to

    :param neurons: The population of neurons to send spikes to
    :param init_callback: A callback that should be called when the connection\
         is established
    :param source: The source of the projection
    :param receptor_type: The receptor type of neurons to project to
    :param connector: The connector for the projection
    :param synapse_type: The synapse type for the projection
    """
    # pylint: disable=global-statement
    global send_receive_conn
    if send_receive_conn is None:
        send_receive_conn = \
            spynnaker.external_devices.SpynnakerLiveSpikesConnection(local_port=None)
    label = "{}_sender".format(neurons.label)
    injector_pop = spynnaker.Population(
        neurons.size, spynnaker.external_devices.SpikeInjector(
            database_notify_host="localhost",
            database_notify_port_num=send_receive_conn.local_port),
        label=label)
    spynnaker.Projection(injector_pop, neurons, connector, synapse_type,
                         source=source, receptor_type=receptor_type)
    send_receive_conn.add_send_label(label)
    send_receive_conn.add_start_resume_callback(label, init_callback)


def register_poisson(generator, init_callback):  # pragma no cover
    """
    Sets up live control of a Poisson source, with the given callback called
    when the simulation is ready to be sent to

    :param generator: The Poisson source
    :param init_callback: A callback that should be called when the connection\
        is established
    """
    # pylint: disable=global-statement
    global poisson_conn
    if poisson_conn is None:
        poisson_conn = \
            spynnaker.external_devices.SpynnakerPoissonControlConnection(local_port=None)
    spynnaker.external_devices.add_poisson_live_rate_control(
        generator,
        database_notify_host="localhost",
        database_notify_port_num=poisson_conn.local_port)
    poisson_conn.add_poisson_label(generator.label)
    poisson_conn.add_start_resume_callback(generator.label, init_callback)


def shutdown(): # pragma no cover
    """
    Clears the registered callbacks
    """
    # pylint: disable=global-statement
    global send_receive_conn
    global poisson_conn
    # pylint: disable=protected-access
    if send_receive_conn is not None:
        send_receive_conn.close()
        send_receive_conn = None
    if poisson_conn is not None:
        poisson_conn.close()
        poisson_conn = None
