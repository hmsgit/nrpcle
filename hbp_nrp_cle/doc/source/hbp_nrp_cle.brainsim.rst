brainsim - adapters for neuronal simulation using PyNN
======================================================

.. automodule:: hbp_nrp_cle.brainsim
    :members:
    :undoc-members:
    :show-inheritance:

:mod:`BrainInterface` Module
----------------------------

.. automodule:: hbp_nrp_cle.brainsim.BrainInterface
    :members:
    :undoc-members:
    :show-inheritance:

:mod:`Common` Module
--------------------

.. automodule:: hbp_nrp_cle.brainsim.common.__AbstractCommunicationAdapter
    :members:
    :undoc-members:
    :show-inheritance:

Device Groups of any kind
^^^^^^^^^^^^^^^^^^^^^^^^^

.. automodule:: hbp_nrp_cle.brainsim.common.devices.__DeviceGroup
    :members:
    :show-inheritance:

AbstractBrainDevice
^^^^^^^^^^^^^^^^^^^

.. automodule:: hbp_nrp_cle.brainsim.common.devices.__AbstractBrainDevice
    :members:
    :show-inheritance:


:mod:`BrainLoader` Module
-------------------------


.. class:: hbp_nrp_cle.brainsim.pynn.PyNNBrainLoader

    .. note:: For technical reasons, the documentation of this class is moved to the class below.

.. automodule:: hbp_nrp_cle.brainsim.pynn.__doc.__PyNNBrainLoaderDoc
    :members: load_h5_network, load_pointneuron_circuit
    :undoc-members:
    :show-inheritance:

:mod:`PyNNCommunicationAdapter` Module
--------------------------------------

.. class:: hbp_nrp_cle.brainsim.pynn.PyNNCommunicationAdapter.PyNNCommunicationAdapter

    .. note:: For technical reasons, the documentation of this class is moved to the class below.

.. automodule:: hbp_nrp_cle.brainsim.pynn.__doc.__PyNNCommAdapterDoc
    :members: PyNNCommunicationAdapter
    :undoc-members:
    :show-inheritance:

Devices used by the PyNNCommunicationAdapter
--------------------------------------------

The *PyNNCommunicationAdapter* uses the following modules to implement its mapping:

IACSource
^^^^^^^^^

.. automodule:: hbp_nrp_cle.brainsim.pynn.devices.__PyNNACSource
    :members:
    :show-inheritance:

INCSource
^^^^^^^^^

.. automodule:: hbp_nrp_cle.brainsim.pynn.devices.__PyNNDCSource
    :members:
    :show-inheritance:
    
IFixedSpikeGenerator
^^^^^^^^^^^^^^^^^^^^

.. automodule:: hbp_nrp_cle.brainsim.pynn.devices.__PyNNFixedSpikeGenerator
    :members:
    :show-inheritance:
    
ILeakyIntegratorAlpha and ILeakyIntegratorExp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. automodule:: hbp_nrp_cle.brainsim.pynn.devices.__PyNNLeakyIntegrator
    :members:
    :show-inheritance:
    
.. automodule:: hbp_nrp_cle.brainsim.pynn.devices.__PyNNLeakyIntegratorTypes
    :members:
    :show-inheritance:
    
INCSource
^^^^^^^^^

.. automodule:: hbp_nrp_cle.brainsim.pynn.devices.__PyNNNCSource
    :members:
    :show-inheritance:


IPoissonSpikeGenerator
^^^^^^^^^^^^^^^^^^^^^^

.. automodule:: hbp_nrp_cle.brainsim.pynn.devices.__PyNNPoissonSpikeGenerator
    :members:
    :show-inheritance:
    
IPopulationRate
^^^^^^^^^^^^^^^

.. automodule:: hbp_nrp_cle.brainsim.pynn.devices.__PyNNPopulationRate
    :members:
    :show-inheritance:


Devices used by the PyNNNestCommunicationAdapter
------------------------------------------------

The *PyNNNestCommunicationAdapter* uses the following modules to implement its mapping:

IACSource
^^^^^^^^^

.. automodule:: hbp_nrp_cle.brainsim.pynn_nest.devices.__PyNNNestACSource
    :members:
    :show-inheritance:

INCSource
^^^^^^^^^

.. automodule:: hbp_nrp_cle.brainsim.pynn_nest.devices.__PyNNNestDCSource
    :members:
    :show-inheritance:

IFixedSpikeGenerator
^^^^^^^^^^^^^^^^^^^^

.. automodule:: hbp_nrp_cle.brainsim.pynn_nest.devices.__PyNNNestFixedSpikeGenerator
    :members:
    :show-inheritance:

ILeakyIntegratorAlpha and ILeakyIntegratorExp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. automodule:: hbp_nrp_cle.brainsim.pynn_nest.devices.__PyNNNestLeakyIntegrator
    :members:
    :show-inheritance:


INCSource
^^^^^^^^^

.. automodule:: hbp_nrp_cle.brainsim.pynn_nest.devices.__PyNNNestNCSource
    :members:
    :show-inheritance:

IPoissonSpikeGenerator
^^^^^^^^^^^^^^^^^^^^^^

Thy *PyNNNestCommunicationAdapter* does not provide an own poisson spike generator implementation
 but uses the one provided by the *PyNNCommunicationAdapter* (see
 :class:`hbp_nrp_cle.brainsim.pynn.devices.__PyNNPoissonSpikeGenerator`).

IPopulationRate
^^^^^^^^^^^^^^^

.. automodule:: hbp_nrp_cle.brainsim.pynn_nest.devices.__PyNNNestPopulationRate
    :members:
    :show-inheritance:

ISpikeRecorder
^^^^^^^^^^^^^^

.. automodule:: hbp_nrp_cle.brainsim.pynn_nest.devices.__PyNNNestSpikeRecorder
    :members:
    :show-inheritance:


:mod:`PyNNControlAdapter` Module
--------------------------------


.. class:: hbp_nrp_cle.brainsim.pynn.PyNNControlAdapter.PyNNControlAdapter

    .. note:: For technical reasons, the documentation of this class is moved to the class below.

.. automodule:: hbp_nrp_cle.brainsim.pynn.__doc.__PyNNControlAdapterDoc
    :members: PyNNControlAdapter
    :undoc-members:
    :show-inheritance:

