Tutorial: Variables in Transfer Functions
===============================================

The evaluation of certain experiments requires observing the world and or the brain simulation and
judge based on the observations whether an experiment is executed successfully or not. These
decisions can be based on metrics which can be observed directly or have to be derived from
observable values.

Transfer functions interconnect the two different simulations and are executed iteratively after
each simulation step. Therefore, they are also suitable to aggregate measurements in order to
determine relevant metrics. This tutorial demonstrates the use of global and local variables to
use transfer functions as means of aggregating metrics.

.. note:: Transfer function variables are currently only available in the python transfer function
    framework. An extension of the BIBI model is planned for the near future.

Transfer function variables are mapped to parameters of the transfer function similar to the
mappings of robot devices (e.g. `@nrp.MapRobotSubscriber`) or brain devices (e.g. `@nrp.MapSpikeSink`).
The following code snipped demonstrates declaring a transfer function local variable `last_t` to
which the current value of `t` (the simulation time) is stored to:

.. code-block:: python

    @nrp.MapVariable("last_t", initial_value=0)
    @nrp.Robot2Neuron()
    def calculate_diff_t(t, last_t):
        diff_t = t - last_t.value
        # do something important with the difference
        last_t.value = t

The `initial_value` keyword allows to initialize the variable before the first invocation of the
transfer function. After that the variable stores the last value assigned to its `value` property
for the lifetime of the transfer function. The scope of a transfer function variable is local to a
transfer function by default.

Additionally, transfer function variables can also be declared globally, to be accessible from
different transfer functions. For this it is necessary to explicitly specify the scope of the
variable to be global.

.. code-block:: python

    @nrp.MapVariable("shared_among_all_tfs", initial_value=0, scope=nrp.GLOBAL)
    @nrp.Robot2Neuron()
    def calculate_diff_t(t, shared_among_all_tfs):
        # do some calculation


In order to prevent from multiple global variables to create naming conflicts it is possible to
specify a global key that is used to identify the variable. In the following example the parameter
`shared_var` of the first transfer function maps to the same variable as the parameter `shared_v` of
the second one. If no global key is specified the parameter name is used as identifier.

.. code-block:: python

    @nrp.MapVariable("shared_var", global_key="shared_var_no_1", initial_value=0, scope=nrp.GLOBAL)
    @nrp.Robot2Neuron()
    def calculate_sth_1(t, shared_among_all_tfs):
        # do some calculation

    @nrp.MapVariable("shared_v", global_key="shared_var_no_1", initial_value=0, scope=nrp.GLOBAL)
    @nrp.Neuron2Robot(Topic("/metrics/some_metric", std_msgs.msg.Float32))
    def calculate_sth_2(t, shared_v):
        # do some calculation

Although the examples showed only single variable mappings it is nevertheless possible to map
multiple variable to a transfer function or to combine variable mappings with other parameters (
e.g. `@nrp.MapRobotSubscriber`)
