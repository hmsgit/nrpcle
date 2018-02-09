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
This module contains dedicated support for device groups in Nest
Since PyNN does not allow to be bypassed, we completely bypass PyNN in order to get an
improved performance when updating device parameters of multiple devices
"""

from hbp_nrp_cle.brainsim.common.devices import DeviceGroup
from mpi4py import MPI
import nest
import numpy

__author__ = "Georg Hinkel"


def create_transformation(nest_name, transform=None):
    """
    Specifies a value transform to the given nest name

    :param nest_name: The name of an attribute in nest
    :param transform: A linear scale factor or None
    :return: a pair of getter and setter functions that will perform the corresponding nest
     operations
    """
    if transform is None:
        def getter(device_ids):
            """
            Gets the attribute values for the given device ids

            :param device_ids: A sequence of device ids
            :return: A sequence of values for the individual devices
            """
            return nest.GetStatus(device_ids, nest_name)

        def setter(device_ids, value):
            """
            Sets the attribute values for the given device ids

            :param device_ids: A sequence of device ids
            :param value: A sequence of values for the individual devices
            """
            if hasattr(value, '__getitem__'):
                vals = []
                for i in range(0, len(device_ids)):
                    vals.append({nest_name: value[i]})
                nest.SetStatus(device_ids, vals)
            else:
                nest.SetStatus(device_ids, {nest_name: value})
        return (getter, setter)
    else:
        def getter(device_ids):
            """
            Gets the attribute values for the given device ids

            :param device_ids: A sequence of device ids
            :return: A sequence of values for the individual devices
            """
            return numpy.asarray(nest.GetStatus(device_ids, nest_name)) / transform

        def setter(device_ids, value):
            """
            Sets the attribute values for the given device ids

            :param device_ids: A sequence of device ids
            :param value: A sequence of values for the individual devices
            """
            if hasattr(value, '__getitem__'):
                vals = []
                for i in range(0, len(device_ids)):
                    vals.append({nest_name: value[i] * transform})
                nest.SetStatus(device_ids, vals)
            else:
                nest.SetStatus(device_ids, {nest_name: value * transform})
        return (getter, setter)


class PyNNNestDeviceGroup(DeviceGroup):
    """
    This class provides an optimized device group behavior
    """

    def create_subgroup(self, selection):
        """
        Creates a sub-devicegroup for the given indices

        :param selection: A selection of devices as slice or list
        :return: A new device group representing a subset of the devices represented
        by this device group
        """
        devices = self.devices[selection]
        group = type(self)(self.device_type, devices)
        device_ids = []
        for d in devices:
            device_ids.append(d.device_id)
        group.__dict__['_device_ids'] = device_ids
        return group

    def connect(self, neurons, **params):
        """
        Connects the contained devices to the neurons specified as parameter.

        :param neurons: the neuron population
        :param params: additional parameters for the connection
        """
        super(PyNNNestDeviceGroup, self).connect(neurons, **params)
        device_ids = []
        for d in self.__dict__['devices']:
            device_ids.append(d.device_id)
        self.__dict__['_device_ids'] = device_ids

    def get(self, attrname):
        """
        Gets the specified attribute of all devices in the device group

        :param attrname: The attribute to get
        :return: A numpy array with all the values for the given attribute for all the devices
        in this device group
        """
        return self.device_type.transformations[attrname][0](self.__dict__['_device_ids'])

    def set(self, attrname, value):
        """
        Sets the specified attribute of all devices to the given value

        :param attrname: The name of the attribute
        :param value: The value that should be assigned to the attribute.
        If this value is indexable, each device is assigned the respective index of the value
        """
        self.device_type.transformations[attrname][1](self.__dict__['_device_ids'], value)


class PyNNNestDevice(object):
    """
    This class provides the base functionality of a device with optimized device group behavior and
    also provides the base functionality of an MPI aware Nest device.
    """

    # default attribute for MPI awareness set to false
    mpi_aware = False

    @classmethod
    def create_new_device_group(cls, populations, params):
        """
        Returns a new device group instance for the concrete implementation of this brain device.

        :param length: the size of the group (the amount of nested devices)
        :param params: additional parameters which are passed to the constructor of the nested
            devices. For each parameter either the value can be supplied, or a list of values,
            one for each nested device.
        :param populations: The populations for which the device should be created
        :return: a new device group containing the created nested devices
        """
        return PyNNNestDeviceGroup.create_new_device_group(populations, cls, params)

    @property
    def device_id(self):
        """
        Returns the internal device id
        """
        # pylint: disable=protected-access, no-member
        return self._generator._device[0]

    def SetStatus(self, neuron_ids, params):
        """
        MPI-aware instance of Nest SetStatus, notifies the remote brain processes to call
        SetStatus with the same parameters if the simulation is being run in a supported
        configuration. Otherwise, only runs the command in this process.

        This emulates PyNN-like behavior as SetStatus must be invoked in all processes to
        have an impact on the simulation as we cannot guarantee the location of each neuron.
        """

        # default single-process or non MPI-aware (e.g. MUSIC) instance
        if self.mpi_aware:
            for rank in xrange(MPI.COMM_WORLD.Get_size()):
                if rank == MPI.COMM_WORLD.Get_rank():
                    continue

                # send the data required to call SetStatus on each process
                MPI.COMM_WORLD.send({'command': 'SetStatus', 'ids': neuron_ids, 'params': params},
                                    dest=rank, tag=100)

        # perform the nest command in this process regardless of configuration
        # The nest device is only available as protected property of the PyNN device
        # pylint: disable=protected-access, no-member
        nest.SetStatus(neuron_ids, params)
