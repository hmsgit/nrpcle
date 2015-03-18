'''
Implementation of PyNNDeviceGroup
moduleauthor: probst@fzi.de
'''

from ..BrainInterface import IDeviceGroup
import numpy

__author__ = 'DimitriProbst'


class PyNNDeviceGroup(IDeviceGroup):
    """
    Gathers multiple devices of the same type to a group.
    The class receives the property values as slice, tuple, list, numpy array
    and returns the property values as numpy array.
    """

    # pylint: disable=R0903,R0924
    def __init__(self, devices):
        """
        Initializes a device group
        """
        # use this form since __setattr__ is overridden
        self.__dict__['devices'] = devices

    def __getitem__(self, index):
        if isinstance(index, (slice, numpy.ndarray)):
            return PyNNDeviceGroup(self.devices[index])
        elif isinstance(index, int):
            return self.devices[index]
        else:
            raise TypeError

    def __getattr__(self, attrname):
        array = numpy.zeros(len(self.devices))
        i = 0
        for device in self.devices:
            array[i] = getattr(device, attrname)
            i += 1
        return array

    def __setattr__(self, attrname, value):
        if attrname in self.__dict__:
            self.__dict__[attrname] = value
        else:
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
