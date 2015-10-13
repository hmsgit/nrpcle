Tutorial: Using TFs specified directly in Python
================================================

.. warning:: This integration is subject to change due to security issues. In the future, users may
    be limited to certain packages and may not be allowed to use double underscores (__)

Transfer functions can be specified as Python code directly additional to the model-based
specification. They can be loaded from a python script residing in the 'NRP_MODELS_DIRECTORY' or
embedded directly into the BIBI xml. Each python function has to ensure that the required modules
are imported. Furthermore, as python is sensitive to a correct indentation particularly for
embedding the transfer functions you are required to either use four spaces (as recommended in the
python documentation) or tabs. The syntax is as depicted below.

.. note:: A combination of embedding and referencing is not possible for one transfer function.
    Whenever python code files are referenced (``src`` attribute provided) embedded text is
    completely ignored.


The following imports the 'MyTransferFunctions.py' script from the models directory:

.. code-block:: xml

    <transferFunction xsi:type="PythonTransferFunction" src="MyTransferFunctions.py"/>

The following imports the embedded transfer function:

.. code-block:: xml

    <transferFunction xsi:type="PythonTransferFunction">
    import ...

    def transfer_function_foo(...
        return ...
    </transferFunction>


The following works equally (initial indent does not matter, as long as indent is relatively
correct):

.. code-block:: xml

    <transferFunction xsi:type="PythonTransferFunction">
        import ...

        def transfer_function_foo(...
            return ...
    </transferFunction>

**The following does not work, as the relative indent between the first line of content and the
remainder is off.**

.. code-block:: xml

    <transferFunction xsi:type="PythonTransferFunction">
        import ...

            def transfer_function_foo(...
                return ...
    </transferFunction>

