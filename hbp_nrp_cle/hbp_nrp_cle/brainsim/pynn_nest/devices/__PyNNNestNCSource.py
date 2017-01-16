'''
Implementation of PyNNNCSource
moduleauthor: probst@fzi.de
'''

from hbp_nrp_cle.brainsim.pynn.devices import PyNNNCSource
from hbp_nrp_cle.brainsim.pynn_nest.devices.__NestDeviceGroup import PyNNNestDevice, \
    create_transformation
import nest

__author__ = 'Georg Hinkel, Dimitri Probst'


class PyNNNestNCSource(PyNNNestDevice, PyNNNCSource):
    """
    Represents a noisy current generator
    :param kwargs: Optional configuration parameters
    """

    transformations = {
        "mean": create_transformation("mean", 1000.0),
        "stdev": create_transformation("std", 1000.0)
    }

    # PyLint does not correctly recognize the overriding of the property setter
    # pylint: disable=arguments-differ
    @PyNNNCSource.mean.setter
    def mean(self, value):
        """
        Sets the mean of the current

        :param value: float
        """
        self._generator.mean = value
        # The nest device is only available as protected property of the PyNN device
        # pylint: disable=protected-access
        nest.SetStatus(self._generator._device, {'mean': 1000.0 * value})

    # PyLint does not correctly recognize the overriding of the property setter
    # pylint: disable=arguments-differ
    @PyNNNCSource.stdev.setter
    def stdev(self, value):
        """
        Sets the stdev of the current

        :param value: float
        """
        self._generator.stdev = value
        # The nest device is only available as protected property of the PyNN device
        # pylint: disable=protected-access
        nest.SetStatus(self._generator._device, {'std': 1000.0 * value})
