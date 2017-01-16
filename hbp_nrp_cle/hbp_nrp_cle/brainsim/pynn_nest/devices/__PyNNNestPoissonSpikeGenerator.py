'''
Implementation of PyNNACSource
moduleauthor: probst@fzi.de
'''

from hbp_nrp_cle.brainsim.pynn.devices import PyNNPoissonSpikeGenerator
from hbp_nrp_cle.brainsim.pynn_nest.devices.__NestDeviceGroup import PyNNNestDevice, \
    create_transformation
import nest

__author__ = 'Georg Hinkel, Dimitri Probst'


class PyNNNestPoissonSpikeGenerator(PyNNNestDevice, PyNNPoissonSpikeGenerator):
    """
    Represents an alternating current generator.
    """

    transformations = {
        "rate": create_transformation("rate")
    }

    # PyLint does not correctly recognize the overriding of the property setter
    # pylint: disable=arguments-differ
    @PyNNPoissonSpikeGenerator.rate.setter
    def rate(self, value):
        """
        Sets the amplitude of the current

        :param value: float
        """
        # The nest device is only available as protected property of the PyNN device
        # pylint: disable=protected-access, no-member
        nest.SetStatus(self._generator._device, {'rate': value})

    @property
    def device_id(self):
        """
        Returns the internal device id
        """
        # pylint: disable=protected-access, no-member
        return self._generator[0]
