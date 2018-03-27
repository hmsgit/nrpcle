BIBI Configuration Generator
============================

The BIBI Configuration Generator or BIBI Configuration Script is a Python module that generates
Python transfer functions based on a BIBI Configuration in XML format according to
:doc:`BIBI-configuration`.

Why do we generate code for the BIBI Configuration?
---------------------------------------------------

In principal, there are two options of using the BIBI Configuration models, namely

1. Interpretation, i.e. we traverse the transfer functions each time we call them.
2. Compilation, i.e. we generate code in a lower abstraction from the transfer function model.

The advantages of interpretation are an easier implementation, better security and a higher
interopability, i.e. it is easier to change the transfer function during a running simulation.
Conversely, the advantage of a compilation is usually its better performance and lower memory 
consumption since it is not necessary to traverse a graph at runtime.

Since the performance of transfer functions is critical given that they are executed at every
timestep and we do not want to support to change the transfer functions during a running simulation,
we have decided for the compilation. The implementation effort is in a reasonable scale and the
security is addressed in :ref:`bibi-security`.

How do we compile the transfer functions?
-----------------------------------------

At first, the XML file is loaded into memory using model representation classes as generated from
**pyxb** (http://pyxb.sourceforge.net/ ), based on the XML Schema Document for the
BIBI Configuration File Format (checked in into the Experiments repository). Then, this configuration
model is transformed to Python code using **Jinja2** templates. These generated files are then
loaded into memory by using the **RestrictedPython** module.

The **RestrictedPython** essentially transforms the AST of the Transfer Function and allows users to
plug in checks whether certain attributes may be accessed, whether certain modules may be loaded and
so on. At the time of writing, we are not using any of these checks, so we basically allow whatever
code the user has specified with one notable exception: RestrictedPython disallows any usage of double
underscores ``__`` as these can be a security hazard.