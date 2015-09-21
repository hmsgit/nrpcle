"""
Script to run CLE from BIBI Configuration File
"""
from hbp_nrp_cle.bibi_config.generated import bibi_api_gen

__author__ = 'GeorgHinkel'

import jinja2
import os
import logging

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

__device_types = {'ACSource': 'ac_source', 'DCSource': 'dc_source',
                  'FixedFrequency': 'fixed_frequency',
                  'LeakyIntegratorAlpha': 'leaky_integrator_alpha',
                  'LeakyIntegratorExp': 'leaky_integrator_exp',
                  'NCSource': 'nc_source',
                  'Poisson': 'poisson',
                  'PopulationRate': 'population_rate',
                  'SpikeRecorder': 'spike_recorder'
                  }
__device_properties = {'ACSource': 'amplitude', 'DCSource': 'amplitude',
                       'FixedFrequency': 'rate',
                       'LeakyIntegratorAlpha': 'voltage',
                       'LeakyIntegratorExp': 'voltage',
                       'NCSource': 'mean',
                       'Poisson': 'rate',
                       'PopulationRate': 'rate',
                       'SpikeRecorder': 'times'}
__operator_symbols = {bibi_api_gen.Subtract: '({0} - {1})',
                      bibi_api_gen.Add: '({0} + {1})',
                      bibi_api_gen.Multiply: '{0} * {1}',
                      bibi_api_gen.Divide: '{0} / {1}',
                      bibi_api_gen.Min: 'min({0}, {1})',
                      bibi_api_gen.Max: 'max({0}, {1})'}

__monitoring_types = {'PopulationRate': 'cle_ros_msgs.msg.SpikeRate',
                      'LeakyIntegratorAlpha': 'cle_ros_msgs.msg.SpikeRate',
                      'LeakyIntegratorExp': 'cle_ros_msgs.msg.SpikeRate',
                      'SpikeRecorder': 'cle_ros_msgs.msg.SpikeEvent'}
# 1 = simulation time,  2 = spikes, 3 = port name, 4 = number of monitored neurons
__monitoring_factory = {'PopulationRate': '{0}({1}, {2}, "{3}")',
                        'LeakyIntegratorAlpha': '{0}({1}, {2}, "{3}")',
                        'LeakyIntegratorExp': '{0}({1}, {2}, "{3}")',
                        'SpikeRecorder': 'monitoring.create_spike_recorder_message'
                                         '({1}, {4}, {2}, "{3}")'}
__monitoring_topics = {'PopulationRate': '/monitor/population_rate',
                       'LeakyIntegratorAlpha': '/monitor/leaky_integrator_alpha',
                       'LeakyIntegratorExp': '/monitor/leaky_integrator_exp',
                       'SpikeRecorder': '/monitor/spike_recorder'}


def __load_template(path):
    """
    Loads the template given by the specified relative path

    :param path: Template path relative to bibi configuration script (this file)
    :return: The template as Jinja2 template
    """
    template_path = os.path.join(os.path.split(__file__)[0], path)
    with open(template_path, 'r') as template_file:
        return jinja2.Template(template_file.read())


cle_template = __load_template('cle_template.pyt')
tf_template = __load_template('tf_template.pyt')


def remove_extension(fname):
    """
    Removes the extension from the given file name.

    :param fname: The file name
    """
    return os.path.splitext(fname)[0]


def get_device_name(device_type):
    """
    Gets the CLE name of the given device type

    :param device_type: The device type
    """
    return __device_types[device_type]


def print_expression(expression):
    """
    Prints the given flow expression to a string

    :param expression: The expression to be printed
    """

    # Pylint demands less *if* statements but each *if* is very simple so that should be ok
    # pylint: disable=R0911
    if isinstance(expression, bibi_api_gen.Scale):
        return str(expression.factor) + ' * ' + print_expression(expression.inner)
    if isinstance(expression, bibi_api_gen.Call):
        temp = expression.type + '('
        i = 1
        for argument in expression.argument:
            temp += argument.name + '=' + print_expression(argument.value_)
            if i < len(expression.argument):
                temp += ', '
            i += 1
        return temp + ')'
    if isinstance(expression, bibi_api_gen.Operator):
        return print_operator(expression)
    if isinstance(expression, bibi_api_gen.ArgumentReference):
        if expression.property_ is None:
            return expression.name
        else:
            return expression.name + '.' + expression.property_
    if isinstance(expression, bibi_api_gen.Constant):
        return str(expression.value_)

    if isinstance(expression, bibi_api_gen.SimulationStep):
        return "t"

    raise Exception('No idea how to print expression of type ' + repr(type(expression)))


def get_monitoring_topic(monitor):
    """
    Gets the monitoring topic for the given neuron monitor

    :param monitor: The neuron monitor
    :return: The topic address as string
    """
    devtype = monitor.device[0].type
    return __monitoring_topics.get(devtype)


def get_monitoring_type(monitor):
    """
    Gets the topic type for the given neuron monitor

    :param monitor: The neuron monitor
    :return: The topic of the monitoring
    """
    devtype = monitor.device[0].type
    return __monitoring_types.get(devtype)


def get_monitoring_impl(monitor):
    """
    Gets the monitoring implementation for the given monitor

    :param monitor: The given monitor
    :return: The implementation, i.e. the value to send to the monitoring topic as code
    """
    dev = monitor.device[0]
    function = __monitoring_factory.get(monitor.device[0].type)
    return function.format(get_monitoring_type(monitor), "t",
                           dev.name + "." + get_default_property(dev.type),
                           monitor.name, get_neuron_count(dev.neurons))


def print_operator(expression):
    """
    Prints the given operator expression to a string

    :param expression: The operator expression to be printed
    """
    text = print_expression(expression.operand[0])
    operator = __operator_symbols[type(expression)]
    for i in range(1, len(expression.operand)):
        text = operator.format(text, print_expression(expression.operand[i]))
    return text


def get_default_property(device_type):
    """
    Gets the default property for the given device type

    :param device_type: The device type
    """
    return __device_properties[device_type]


def print_neurons(neuron_selector):
    """
    Gets a string representing the accessed neuron population

    :param neuron_selector: The neuron selector
    """
    index = get_neurons_index(neuron_selector)
    if index is not None:
        return "nrp.brain." + neuron_selector.population + "[" + index + "]"
    return "nrp.brain." + neuron_selector.population


def get_neuron_count(neurons):
    """
    Gets the amount of neurons connected

    :param neurons: The neuron selector
    :return: The amount of neurons as int
    """
    if isinstance(neurons, bibi_api_gen.Index):
        return 1
    elif isinstance(neurons, bibi_api_gen.Range):
        if neurons.step is None:
            return neurons.to - neurons.from_
        return (neurons.to - neurons.from_) / neurons.step
    elif isinstance(neurons, bibi_api_gen.List):
        return len(neurons.element)
    elif isinstance(neurons, bibi_api_gen.Population):
        return neurons.count
    raise Exception("Neuron Count: Don't know how to process neuron selector "
                    + str(type(neurons)))


def get_neurons_index(neurons):
    """
    Gets the indexing operator for the given neuron selector

    :param neurons: The neuron selector
    :return: A string with the appropriate index or None
    """

    if isinstance(neurons, bibi_api_gen.Index):
        return str(neurons.index)
    elif isinstance(neurons, bibi_api_gen.Range):
        step_string = ""
        if neurons.step is not None:
            step_string = ', ' + str(neurons.step)
        return 'slice(' + str(neurons.from_) + ', ' + str(neurons.to) \
               + step_string + ')'
    elif isinstance(neurons, bibi_api_gen.List):
        if len(neurons.element) == 0:
            return '[]'
        neuron_list = '[' + str(neurons.element[0])
        for i in range(1, len(neurons.element)):
            neuron_list = neuron_list + ', ' + str(neurons.element[i])
        return neuron_list + ']'
    elif isinstance(neurons, bibi_api_gen.Population):
        return None
    raise Exception("Neuron Print: Don't know how to process neuron selector "
                    + str(neurons))


def get_neuron_template(neurons):
    """
    Prints the given neuron selector template

    :param neurons: The neurons selector template
    :return: The neurons selection
    """
    if isinstance(neurons, bibi_api_gen.IndexTemplate):
        return "[" + str(neurons.index) + "]"
    if isinstance(neurons, bibi_api_gen.RangeTemplate):
        step_string = ""
        if neurons.step is not None:
            step_string = ', ' + str(neurons.step)
        return '[slice(' + str(neurons.from_) + ', ' + str(neurons.to) \
               + step_string + ')]'
    if isinstance(neurons, bibi_api_gen.ListTemplate):
        if len(neurons.element) == 0:
            return '[[]]'
        neuron_list = '[[' + neurons.element[0]
        for i in range(1, len(neurons.element)):
            neuron_list = neuron_list + ', ' + neurons.element[i]
        return neuron_list + ']]'
    raise Exception("Neuron Print: Don't know how to process neuron selector "
                    + str(type(neurons)))


def get_collection_source(selector):
    """
    Gets the collection specifier for the given neuron source

    :param selector: The neuron source
    :return: A collection specifier as string
    """
    if isinstance(selector, bibi_api_gen.Range):
        step_string = ""
        if selector.step is not None:
            step_string = ', ' + str(selector.step)
        return 'range(' + str(selector.from_) + ', ' + str(selector.to) \
               + step_string + ')'
    elif isinstance(selector, bibi_api_gen.List):
        if len(selector.element) == 0:
            return '[]'
        neuron_list = '[' + str(selector.element[0])
        for i in range(1, len(selector.element)):
            neuron_list = neuron_list + ', ' + str(selector.element[i])
        return neuron_list + ']'
    elif isinstance(selector, bibi_api_gen.Population):
        return "range(0, " + str(selector.count) + ")"
    raise Exception("Neuron Print: Don't know how to process neuron selector "
                    + str(type(selector)))


def print_neuron_group(neuron_group):
    """
    Prints the given neuron group

    :param neuron_group: The given neuron group
    :return: The neuron group printed to text
    """
    if isinstance(neuron_group, bibi_api_gen.MapSelector):
        result = "nrp.map_neurons(" + get_collection_source(neuron_group.source) + ", " + \
                 "lambda i: nrp.brain." + neuron_group.source.population + \
                 get_neuron_template(neuron_group.pattern) + ")"
        return result
    elif isinstance(neuron_group, bibi_api_gen.ChainSelector):
        result = "nrp.chain_neurons("
        anything = False
        for neuron_selector in neuron_group.neurons:
            if anything:
                result += ", "
            else:
                anything = True
            result += print_neurons(neuron_selector)
        for group in neuron_group.connectors:
            if anything:
                result += ", "
            else:
                anything = True
            result += print_neuron_group(group)
        result += ")"
        return result
    raise Exception("Don't know how to print group of type " + str(type(neuron_group)))


def print_device_config(dev):
    """
    Prints the configuration of the given device or device group

    :param dev: The device or device group
    :return: The device configuration as string
    """
    res = ""
    if dev.synapseDynamics is not None:
        res += ", synapse_dynamics=" + print_synapse_dynamics(dev.synapseDynamics)
    elif dev.synapseDynamicsRef is not None:
        res += ", synapse_dynamics=" + dev.synapseDynamicsRef.ref
    if dev.connector is not None:
        res += ", connector=" + print_connector(dev.connector)
    elif dev.connectorRef is not None:
        res += ", connector=" + dev.connectorRef.ref
    if dev.target is not None:
        res += ", target='" + dev.target.lower() + "'"
    return res


def print_synapse_dynamics(synapse_dynamics):
    """
    Creates a synapse dynamics initialization

    :param synapse_dynamics: The synanapse dynamics element
    :return: Code that creates the synapse dynamics
    """
    if isinstance(synapse_dynamics, bibi_api_gen.TsodyksMarkramMechanism):
        return "sim.SynapseDynamics(fast=" \
               "sim.TsodyksMarkramMechanism(U={0}, tau_rec={1}, tau_facil={2}))" \
            .format(synapse_dynamics.u, synapse_dynamics.tau_rec, synapse_dynamics.tau_facil)
    raise Exception(
        "Don't know how to print synapse dynamics of type " + str(type(synapse_dynamics)))


def print_connector(connector):
    """
    Creates a neuron connector

    :param connector: The neuron connector model
    :return: Code that creates the neuron connector
    """
    if isinstance(connector, bibi_api_gen.OneToOneConnector):
        return "sim.OneToOneConnector(weights={0}, delays={1})".format(connector.weights,
                                                                       connector.delays)
    if isinstance(connector, bibi_api_gen.AllToAllConnector):
        return "sim.AllToAllConnector(weights={0}, delays={1})".format(connector.weights,
                                                                       connector.delays)
    if isinstance(connector, bibi_api_gen.FixedNumberPreConnector):
        return "sim.FixedNumberPreConnector({2}, weights={0}, delays={1})".format(
            connector.weights, connector.delays, connector.count)
    raise Exception("Don't know how to print connector of type " + str(type(connector)))


def compute_dependencies(config):
    """
    Computed the dependencies of the given configuration

    :param config: The BIBI configuration
    """
    dependencies = set()
    assert isinstance(config, bibi_api_gen.BIBIConfiguration)
    for tf in config.transferFunction:
        if hasattr(tf, "local"):
            for local in tf.local:
                __add_dependencies_for_expression(local.body, dependencies)
        if isinstance(tf, bibi_api_gen.Neuron2Robot):
            if tf.returnValue is not None:
                dependencies.add(tf.returnValue.type)
        if hasattr(tf, "topic"):
            for topic in tf.topic:
                dependencies.add(topic.type)
    return dependencies


def __add_dependencies_for_expression(expression, dependencies):
    """
    Adds the dependencies for the given expression to the given set of dependencies

    :param expression: The expression that may cause dependencies
    :param dependencies: The dependencies detected so far
    """
    if isinstance(expression, bibi_api_gen.Scale):
        __add_dependencies_for_expression(expression.inner, dependencies)
    if isinstance(expression, bibi_api_gen.Call):
        dependencies.add(expression.type)
        for argument in expression.argument:
            __add_dependencies_for_expression(argument.value_, dependencies)
    if isinstance(expression, bibi_api_gen.Operator):
        for operand in expression.operand:
            __add_dependencies_for_expression(operand, dependencies)


def is_not_none(item):
    """
    Gets whether the given item is None (required since Jinja2 does not understand None tests)

    :param item: The item that should be tested
    :return: True if the item is not None, otherwise False
    """
    return item is not None


FOUR_SPACES = " " * 4


def correct_indentation(text, first_line_indent_level, indent_string=FOUR_SPACES):
    """
    Adapts the indentation of the given code in order to paste it into the generated script. The
    indentation of the given text is determined based on the first content line (first line which
    consists not only of white spaces). The indentation must either use four spaces or a tab
    character.

    :param text: the original input to adapt
    :param first_line_indent_level: the target indentation level of the entire block
    :param indent_string: (Optional): the pattern for one level of indentation
    :return: the adapted text
    """

    logger.debug(text)
    text.replace("\t", indent_string)
    lines = text.split("\n")
    result = []

    first_line_indent = None
    for line in lines:
        if line.lstrip():
            if first_line_indent is None and line:
                first_line_stripped = line.lstrip()
                first_line_indent = line[:len(line) - len(first_line_stripped)]

            result.append(indent_string * first_line_indent_level + line[len(first_line_indent):])
        else:
            result.append("\n")

    return "\n".join(result)


def import_referenced_python_tfs(bibi_conf_inst, models_path):
    """
    Parses the BIBI configuration, resolves references to external python transfer functions and
    embeds them into the BIBI configuration instance

    :param bibi_conf_inst: A completely parsed 'bibi_api_gen.BIBIconfiguration' instance
    """

    for tf in bibi_conf_inst.transferFunction:
        if hasattr(tf, "src") and tf.src:
            # If the "src" tag is specified, no embedded code is allowed. To make sure
            # clean content beforehand
            assert isinstance(tf, bibi_api_gen.PythonTransferFunction)
            del tf.orderedContent()[:]
            with open(os.path.join(models_path, tf.src)) as f:
                tf.append(f.read())


def generate_tf(tf):
    """
    Generates the code for the given transfer function

    :param tf: The transfer function to generate code for
    :return: A unicode string with the generated code
    """
    names = dict(globals())
    names["tf"] = tf
    return tf_template.render(names)


def generate_cle(bibi_conf, script_file_name, timeout, gzserver_host, sim_id, tf_path):
    """
    Generates Code to run the CLE based on the given configuration file

    :param bibi_conf: The BIBI configuration
    :param script_file_name: The file name of the script to be generated. \
        If None, the generated file will be returned as string
    :param timeout: The timeout found in the ExDConfig
    :param gzserver_host: The host where the gzserver will run, local for local machine \
        lugano for remote Lugano viz cluster.
    :param sim_id: The simulation id
    :param tf_path: A folder where the system may look for transfer function source code
    :return string: Content of the generated file (only if script_file_name is None) else, \
        nothing is returned
    """
    logger.info("Generating CLE launch script")
    logger.debug("Loading BIBI Configuration")
    with open(bibi_conf) as bibi_xml:
        config = bibi_api_gen.CreateFromDocument(bibi_xml.read())

    logger.debug("Resolving python code references")
    import_referenced_python_tfs(config, tf_path)

    names = dict(globals())
    names['config'] = config
    names['dependencies'] = compute_dependencies(config)
    # system functions are somehow not included in globals
    names['len'] = len
    names['timeout'] = timeout
    names['gzserver_host'] = gzserver_host
    names['sim_id'] = sim_id
    logger.debug("Instantiate CLE Template")

    if script_file_name is None:
        # Generate into string and return it
        return cle_template.render(names)

    else:
        # Generate into file and save to disk
        with open(script_file_name, 'w') as output_file:
            output_file.write(cle_template.render(names))
