BIBI Configuration Generator
============================

The BIBI Configuration Generator or BIBI Configuration Script is a Python module that generates
Python transfer functions based on a BIBI Configuration in XML format according to :doc:`BIBI-configuration`.

Why do we generate code for the BIBI Configuration?
---------------------------------------------------

In principal, there are two options of using the BIBI Configuration models, namely

1. Interpretation, i.e. we traverse the transfer functions each time we call them.
2. Compilation, i.e. we generate code in a lower abstraction from the transfer function model.

The advantages of interpretation are an easier implementation, better security and a higher interopability, i.e. it is easier to change the transfer function during a running simulation.
Conversely, the advantage of a compilation is usually its better performance and lower memory consumption since it is not necessary to traverse a graph at runtime.

Since the performance of transfer functions is critical given that they are executed at every timestep and we do not want to support to change the transfer functions during a running simulation,
we have decided for the compilation. The implementation effort is in a reasonable scale and the security is addressed in :ref:`bibi-security`.

For the same reason, we use CPython that generates (compiles) Python files into C code before running it. This is also done in order to improve the performance.

How do we compile the transfer functions?
-----------------------------------------

At first, the XML file is loaded into memory using model representation classes as generated from **generateDS** (http://pythonhosted.org/generateDS/ ), based on the XML Schema Document for the
BIBI Configuration File Format (checked in into the Models repository). Then, this configuration model is transformed to Python code using **Jinja2** templates. These generated files are then loaded into memory
by using the **imp** module.

The **imp** module represents the internals of the *import* statement of python. Therefore, the same machinery is used as for usual Python code. That means for a typical environment running CPython
that C code will be generated for the user defined transfer functions which is then run at every timestep. We use this machinery to minimize the cost of transfer functions for the overall simulations.

An example of the BIBI generator script can be seen below.

.. literalinclude:: ../../hbp_nrp_cle/bibi_config/cle_template.pyt
    :language: python
    :lines: 65-84

The **{% for local in tf.local %}** directives specify that the template parts until the corresponding **{% endfor %}** should be generated for each element of the **tf.local** collection. Within this template,
the contents of *local* can be printed through double braces as for example **{{print_expression(local.body)}}** that would call the *print_expression* function with the *body* property of *local*.
Jinja2 knows the *print_expression* function as all globally defined functions of the *bibi_configuration_script* are automatically forwarded to the Jinja2 templating engine.

.. _bibi-security:

Security
--------

Currently, we have not invested a lot of resources into security. Currently, this is not too important as we do not yet allow users to upload their own BIBI Configuration Files.
We plan to secure the BIBI Configuration files by escaping the builtin functions and removing all double underscores from called methods. Furthermore, we want to restrict the possible names for calls by a whitelist.

.. note:: The security of the BIBI Configuration Script is subject to change.