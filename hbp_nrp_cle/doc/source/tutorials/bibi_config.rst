Tutorial: Writing a BIBI Configuration
======================================

A BIBI Configuration contains all the information necessary to couple a brain model with a robot model using Transfer Functions and run these simulations
in the Closed Loop Engine (CLE). Thus, besides references to brain model and robot model, it contains the specifications of the TFs.

As an XML file, such a specification may be created by tools. We have an XML Schema document to validate BIBI Configuration files.

As the complete metamodel of the BIBI Configuration may be a bit complicated at the beginning, we build a BIBI Configuration stepwise in XML. However, at any point in time, you may
find it useful to lookup :doc:`../BIBI-configuration` for a complete reference.

To begin, we start with a new BIBI Configuration.

.. code-block:: xml

    <bibi xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
          xmlns="http://schemas.humanbrainproject.eu/SP10/2014/BIBI" >
    </bibi>

That is, we simply created a *bibi* element in the BIBI Configuration namespace.

.. note:: The namespace is subject to change. In particular, the XML Schema definition currently cannot be obtained via the given URL.

Specification of the neuronal network
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

However, our BIBI Configuration is not valid. One of the reasons for this is that it lacks a description for the used neuronal network. We need to tell the CLE
what brain models we are going to use and what neuron groups exist.

.. code-block:: xml

    <brainModel>
      <file>brain_model/braitenberg.h5</file>
      <neuronGroup population="sensors" xsi:type="Range" from="0" to="5"/>
      <neuronGroup population="actors" xsi:type="Range" from="6" to="8"/>
    </brainModel>

This code block must be inserted as a child element of the root *bibi* element.

In this example, we have specified that the brain model from *brain_model/braitenberg.h5* should be used. This path is either absolute or
relative to the **NRP_MODELS_DIRECTORY** environment variable.

.. note::
    When using the CLE through the NRP platform, the **NRP_MODELS_DIRECTORY** will be your user directory.
    When using the CLE separately as e.g. for development machines, this environment variable should be set to a Models repository clone.

We further specify two neuron groups, the sensors ranging from 0 to 5 (exclusive) and the actors from 6 to and excluding 8. Note that these neuron groups exactly match the
neuron groups (colors) from :doc:`setup page<setup>`. We will use these neuron groups in the TFs for reference.

.. note:: The XML Schema enforces that the neuronal network file has the correct extension **.h5**.

Specification of the robot model
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Our BIBI Configuration also needs a robot model. We specify it simply by the file name of the robot model in the SDF format. This file then contains both the robot meshes as well as
information on the plugins used by this robot.

.. code-block:: xml

    <bodyModel>husky_model/model.sdf</bodyModel>

.. note:: The XML Schema enforces that the brain model has the correct file extension **.sdf**.

Up to this point, the BIBI Configuration should look as follows:

.. code-block:: xml

    <?xml version="1.0" encoding="UTF-8"?>
    <bibi xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
          xmlns="http://schemas.humanbrainproject.eu/SP10/2014/BIBI">
      <brainModel>
        <file>brain_model/braitenberg.h5</file>
        <neuronGroup population="sensors" xsi:type="Range" from="0" to="5"/>
        <neuronGroup population="actors" xsi:type="Range" from="6" to="8"/>
      </brainModel>
      <bodyModel>husky_model/model.sdf</bodyModel>
    </bibi>

While we now have created a valid BIBI Configuration, it does not yet contain any TF, so the simulations will run in parallel with no connection to each other.
To learn how to specify TFs, see :doc:`neuron2robot`.