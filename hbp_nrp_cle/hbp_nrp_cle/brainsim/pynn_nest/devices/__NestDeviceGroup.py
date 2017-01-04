"""
This module contains dedicated support for device groups in Nest
Since PyNN does not allow to be bypassed, we completely bypass PyNN in order to get an
improved performance when updating device parameters of multiple devices
"""

from hbp_nrp_cle.brainsim.common.devices import DeviceGroup
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

    def __init__(self, device_type, devices):
        """
        Initializes a device group
        """
        super(PyNNNestDeviceGroup, self).__init__(device_type, devices)

        device_ids = []
        for d in devices:
            device_ids.append(d.device_id)
        self.__dict__['_device_ids'] = device_ids

    def get(self, attrname):
        """
        Gets the specified attribute of all devices in the device group

        :param attrname: The attribute to get
        :return: A numpy array with all the values for the given attribute for all the devices
        in this device group
        """
        self.device_type.transformations[attrname][0](self.__dict__['_device_ids'])

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
    This class provides the base functionality of a device with optimized device group behavior
    """

    @classmethod
    def create_new_device_group(cls, length, params):
        """
        Returns a new device group instance for the concrete implementation of this brain device.

        :param length: the size of the group (the amount of nested devices)
        :param params: additional parameters which are passed to the constructor of the nested
            devices. For each parameter either the value can be supplied, or a list of values,
            one for each nested device.
        :return: a new device group containing the created nested devices
        """
        return PyNNNestDeviceGroup.create_new_device_group(cls, length, params)

    @property
    def device_id(self):
        """
        Returns the internal device id
        """
        # pylint: disable=protected-access, no-member
        return self._generator._device[0]
