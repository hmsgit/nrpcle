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
'''
Implementation of DeviceGroup
moduleauthor: probst@fzi.de
'''

from hbp_nrp_cle.brainsim.BrainInterface import IDeviceGroup
import numpy

__author__ = 'DimitriProbst, Sebastian Krach'


class DeviceGroup(IDeviceGroup):
    """
    Gathers multiple devices of the same type to a group.
    The class receives the property values as slice, tuple, list, numpy array
    and returns the property values as numpy array.
    """

    def __init__(self, cls, devices):
        """
        Initializes a device group
        """
        # use this form since __setattr__ is overridden
        self.__dict__['_spec'] = None
        self.__dict__['device_type'] = cls
        self.__dict__['devices'] = devices

    @classmethod
    def create_new_device_group(cls, populations, nested_device_type, params):
        """
        Creates a new device group of the specified size consisting of the brainsim devices of
        the specified type.

        :param nested_device_type: The concrete type of brain device to instantiate
        :param params: additional parameters which are passed to the constructor of the nested
            devices. For each parameter either the value can be supplied, or a list of values,
            one for each nested device.
        :param populations: The populations for which the device should be created
        :return: a new device group
        """
        devices = list()
        i = 0
        for pop in populations:
            device = nested_device_type.create_new_device(
                pop, **cls._create_device_config(params, i))
            devices.append(device)
            i += 1
        return cls(nested_device_type, devices)

    def __getitem__(self, index):
        if isinstance(index, (slice, numpy.ndarray)):
            return self.create_subgroup(index)
        elif isinstance(index, int):
            return self.devices[index]
        else:
            raise TypeError

    def __len__(self):
        return len(self.devices)

    def __getattr__(self, attrname):
        # This is required to enable a device group reset.
        if attrname == 'spec':
            return self.__dict__['_spec']
        if hasattr(DeviceGroup, attrname):
            return super(DeviceGroup, self).__getattr__(attrname)
        return self.get(attrname)

    def create_subgroup(self, selection):
        """
        Creates a sub-devicegroup for the given indices

        :param selection: A selection of devices as slice or list
        :return: A new device group representing a subset of the devices represented
        by this device group
        """
        return type(self)(self.device_type, self.devices[selection])

    def get(self, attrname):
        """
        Gets the specified attribute of all devices in the device group

        :param attrname: The attribute to get
        :return: A numpy array with all the values for the given attribute for all the devices
        in this device group
        """
        array = numpy.zeros(len(self.devices))
        i = 0
        for device in self.devices:
            array[i] = getattr(device, attrname)
            i += 1
        return array

    def __setattr__(self, attrname, value):
        # This is needed to enable a device group reset.
        if attrname == 'spec':
            self.__dict__['_spec'] = value
        elif attrname == 'active':
            super(DeviceGroup, self).__setattr__(attrname, value)
        elif attrname in self.__dict__:
            self.__dict__[attrname] = value
        else:
            self.set(attrname, value)

    def set(self, attrname, value):
        """
        Sets the specified attribute of all devices to the given value

        :param attrname: The name of the attribute
        :param value: The value that should be assigned to the attribute.
        If this value is indexable, each device is assigned the respective index of the value
        """
        if hasattr(value, '__getitem__'):
            i = 0
            for device in self.devices:
                setattr(device, attrname, value[i])
                i += 1
        else:
            for device in self.devices:
                setattr(device, attrname, value)

    def refresh(self, t):
        """
        Refreshes all inner devices at time t

        :param t: The current simulation time
        """
        for device in self.devices:
            if hasattr(device, 'refresh'):
                device.refresh(t)

    @property
    def active(self):
        """
        Returns the activation status of all devices
        """
        active_list = [d.active for d in self.devices]

        return reduce(lambda d1, d2: d1 and d2, active_list, True)

    # pylint: disable=arguments-differ
    @active.setter
    def active(self, bool_value):
        """
        Sets the activation status of all devices
        """
        if bool_value is True:
            self._activate()
        else:
            self._deactivate()

    def _activate(self):
        """
        Activate all devices
        """
        if not self.active:
            for d in self.devices:
                d.active = True

    def _deactivate(self):
        """
        Deactivate all devices
        """
        if self.active:
            for d in self.devices:
                d.active = False

    def connect(self, neurons, **params):
        """
        Connects the contained devices to the neurons specified as parameter.

        :param neurons: the neuron population
        :param params: additional parameters for the connection
        """
        for pop_index, (pop, dev) in enumerate(zip(neurons, self.devices)):
            dev.connect(pop, **self._create_device_config(params, pop_index))

    def _disconnect(self):
        """
        Disconnects all of the underlying devices and clears the device list, this
        device group will be unusable after this call.
        """
        for device in self.devices:
            device._disconnect() # pylint: disable=protected-access
        self.devices[:] = []

    def reset(self, transfer_function_manager):
        """
        Resets the nested devices

        :param transfer_function_manager: The transfer function manager the device belongs to
        :return: The reset device group
        """
        reset_devices = list()
        changes = False
        for device in self.devices:
            reset_device = device.reset(transfer_function_manager)
            reset_devices.append(reset_device)
            changes = changes or (reset_device is not device)
        if not changes:
            return self
        return type(self)(self.device_type, reset_devices)

    @classmethod
    def _create_device_config(cls, params, index):
        """
        Creates the configuration for the device with the given index

        :param params: The original parameters
        :param index: The index of the device
        :return: A parameter array
        """
        new_params = {}
        for key in params:
            value = params[key]
            if isinstance(value, list) or isinstance(value, numpy.ndarray):
                value = value[index]
            new_params[key] = value
        return new_params
