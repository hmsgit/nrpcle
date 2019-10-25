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
This module contains shared helper functions to parse synapse types

This module does not import the simulator module itself, but rather takes the module implementation
as a parameter. The reason for this is that this allows to make it transparent for unit tests
whether some of the parsing functionality has been extracted here.
"""

__author__ = 'LorenzoVannucci'


def set_synapse_type(device_parameters):
    """
    Corrects the synapse type for the given device parameters

    :param device_parameters: The device parameters of the given device
    """
    if device_parameters["synapse_type"] is None:
        device_parameters["synapse_type"] = {'model': 'static_synapse'}
    elif isinstance(device_parameters["synapse_type"], dict):
        dyn = device_parameters["synapse_type"]
        try:
            if dyn["type"] == "TsodyksMarkram":
                device_parameters["synapse_type"] = {'model': 'tsodyks_synapse',
                                                     'U': dyn['U'],
                                                     'tau_rec': dyn['tau_rec'],
                                                     'tau_fac': dyn['tau_facil']
                                                     }
            else:
                raise Exception("Unknown synapse type {0}".format(dyn["type"]))
        except KeyError as e:
            raise Exception("The synapse definition {0} is missing the required field {1}"
                            .format(device_parameters["synapse_type"], str(e)))


def set_connector(device_parameters, updated):
    """
    Corrects the connector type for the given device parameters

    :param device_parameters: The device parameters of the given device
    :param updated: A dictionary of value updates
    """
    if "connector" not in updated or device_parameters["connector"] is None:
        device_parameters["connector"] = 'all_to_all'
    else:
        conn = device_parameters["connector"]
        if isinstance(conn, dict):
            if "weight" not in updated and "weight" in conn:
                device_parameters["weight"] = conn["weight"]
            if "delay" not in updated and "delay" in conn:
                device_parameters["delay"] = conn["delay"]
            if conn.get("mode") == "OneToOne":
                device_parameters["connector"] = 'one_to_one'
            elif conn.get("mode") == "AllToAll":
                device_parameters["connector"] = 'all_to_all'
            elif conn.get("mode") == "Fixed":
                device_parameters["connector"] = {'rule': 'fixed_indegree',
                                                  'indegree': conn.get("n", 1)}
            else:
                raise Exception("Invalid connector mode")
    if device_parameters["receptor_type"] == 'excitatory':
        device_parameters["weight"] = abs(device_parameters["weight"])
    else:
        device_parameters["weight"] = -abs(device_parameters["weight"])
