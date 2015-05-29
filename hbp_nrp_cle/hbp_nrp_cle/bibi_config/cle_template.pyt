# -*- coding: utf-8 -*-
"""
This file is generated from cle_template.pyt. The original demo script this file is based on has been created by Lorenzo Vannucci
meanwhile the Template has been created by Georg Hinkel.
"""
# pragma: no cover

__author__ = 'BIBI Configuration Script'

import rospy
import cle_ros_msgs.msg
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Point, Pose, Quaternion
from std_msgs.msg import Float32, Int32, String
from os.path import expanduser
import os
import netifaces
import subprocess
import logging
from hbp_nrp_cle.bibi_config.notificator import Notificator

logger = logging.getLogger(__name__)

def cle_function_init(world_file):

    from hbp_nrp_cle.cle.ROSCLEServer import ROSCLEServer

    # Create ROS server
    cle_server = ROSCLEServer()
    Notificator.register_notification_function(
        lambda subtask, update_progress: cle_server.notify_current_task(subtask, update_progress, True)
    )

    cle_server.notify_start_task("Initializing the Neurorobotic Closed Loop Engine",
                                 "Importing needed packages",
                                 {% if is_not_none(config.extRobotController) %}7{% else %}5{% endif %}, # number of subtasks
                                 True)  # block_ui

    from hbp_nrp_cle.cle.SerialClosedLoopEngine import SerialClosedLoopEngine

    from hbp_nrp_cle.robotsim.GazeboLoadingHelper import load_gazebo_model_file, empty_gazebo_world, load_gazebo_world_file
    from hbp_nrp_cle.robotsim.RobotInterface import Topic
    from hbp_nrp_cle.robotsim.RosControlAdapter import RosControlAdapter
    from hbp_nrp_cle.robotsim.RosCommunicationAdapter import RosCommunicationAdapter

    from hbp_nrp_cle.brainsim.PyNNControlAdapter import PyNNControlAdapter
    from hbp_nrp_cle.brainsim.PyNNCommunicationAdapter import PyNNCommunicationAdapter

    import hbp_nrp_cle.tf_framework as nrp
    import hbp_nrp_cle.tf_framework.monitoring as monitoring

    # Needed in order to cleanup global static variables
    nrp.start_new_tf_manager()

    # import dependencies from BIBI configuration
{% for dep in dependencies %}
    import {{dep[:dep.rfind('.')]}} #import {{dep.split('.')|last()}}{% endfor %}

    # import transfer functions specified in Python
{% if len(config.transferFunctionImport) > 0 %}    # pylint: disable=W0401
{% for imp in config.transferFunctionImport %}
    import {{remove_extension(imp)}}{% endfor %}{% endif %}

{% for tf in config.transferFunction %}{% if tf.extensiontype_ == 'Neuron2Robot' %}
{% for topic in tf.topic %}{% if is_not_none(topic.body) %}
    @nrp.MapRobotPublisher("{{topic.name}}", Topic('{{topic.topic}}', {{topic.type_}})){% else %}
    @nrp.MapRobotSubscriber("{{topic.name}}", Topic('{{topic.topic}}', {{topic.type_}})){% endif %}{% endfor %}{% for dev in tf.device %}{% if is_not_none(dev.body) %}
    @nrp.MapSpikeSource("{{dev.name}}", nrp.brain.{{get_neurons(dev)}}, nrp.{{get_device_name(dev.type_)}}){% else %}
    @nrp.MapSpikeSink("{{dev.name}}", nrp.brain.{{get_neurons(dev)}}, nrp.{{get_device_name(dev.type_)}}){% endif %}{% endfor %}
    @nrp.Neuron2Robot({% if is_not_none(tf.returnValue) %}Topic('{{tf.returnValue.topic}}', {{tf.returnValue.type_}}){% endif %})
    def {{tf.name}}(t{% for t in tf.topic %}, {{t.name}}{%endfor%}{% for dev in tf.device %}, {{dev.name}}{%endfor%}):
{% for local in tf.local %}
        {{local.name}} = {{print_expression(local.body)}}{% endfor %}
{% for dev in tf.device %}{% if is_not_none(dev.body) %}
        {{dev.name}}.{{get_default_property(dev.type_)}} = {{print_expression(dev.body)}}{% endif %}{% endfor %}
{% for top in tf.topic %}{% if is_not_none(top.body) %}
        {{top.name}}.send_message({{print_expression(top.body)}}){% endif %}{% endfor %}
{% if is_not_none(tf.returnValue) %}
        return {{print_expression(tf.returnValue.body)}}{% endif %}

{% elif tf.extensiontype_ == 'Neuron2Monitor' %}
    @nrp.MapSpikeSink("{{tf.device.name}}", nrp.brain.{{get_neurons(tf.device)}}, nrp.{{get_device_name(tf.device.type_)}})
    @nrp.Neuron2Robot(Topic('{{get_monitoring_topic(tf)}}', {{get_monitoring_type(tf)}}))
    def {{tf.name}}(t, {{tf.device.name}}):
        return {{get_monitoring_impl(tf)}}

{% else %}{% for topic in tf.topic %}{% if is_not_none(topic.body) %}
    @nrp.MapRobotPublisher("{{topic.name}}", Topic('{{topic.topic}}', {{topic.type_}})){% else %}
    @nrp.MapRobotSubscriber("{{topic.name}}", Topic('{{topic.topic}}', {{topic.type_}})){% endif %}{% endfor %}{% for dev in tf.device %}{% if is_not_none(dev.body) %}
    @nrp.MapSpikeSource("{{dev.name}}", nrp.brain.{{get_neurons(dev)}}, nrp.{{get_device_name(dev.type_)}}){% else %}
    @nrp.MapSpikeSink("{{dev.name}}", nrp.brain.{{get_neurons(dev)}}, nrp.{{get_device_name(dev.type_)}}){% endif %}{% endfor %}
    @nrp.Robot2Neuron()
    def {{tf.name}}(t{% for topic in tf.topic %}, {{topic.name}}{%endfor%}{% for dev in tf.device %}, {{dev.name}}{%endfor%}):
{% for local in tf.local %}
        {{local.name}} = {{print_expression(local.body)}}{% endfor %}
{% for dev in tf.device %}{% if is_not_none(dev.body) %}
        {{dev.name}}.{{get_default_property(dev.type_)}} = {{print_expression(dev.body)}}{% endif %}{% endfor %}
{% for top in tf.topic %}{% if is_not_none(top.body) %}
        {{top.name}}.send_message({{print_expression(top.body)}}){% endif %}{% endfor %}{% endif %}{% endfor %}


    # consts
    TIMESTEP = 0.02

    # set models path variable
    models_path = os.environ.get('NRP_MODELS_DIRECTORY')

    Notificator.notify("Resetting Gazebo robotic simulator", True)

    local_ip = netifaces.ifaddresses('eth0')[netifaces.AF_INET][0]['addr']
    ros_master_uri = os.environ.get("ROS_MASTER_URI")
    ros_master_uri = ros_master_uri.replace('localhost', local_ip)

    from hbp_nrp_cle.robotsim.LocalGazebo import LocalGazeboBridgeInstance
    gzweb = LocalGazeboBridgeInstance()

{% if gzserver_host == 'local' %}
    from hbp_nrp_cle.robotsim.LocalGazebo import LocalGazeboServerInstance
    gzserver = LocalGazeboServerInstance()
    gzserver.start(ros_master_uri)
{% elif gzserver_host == 'lugano' %}
    from hbp_nrp_cle.robotsim.LuganoVizClusterGazebo import LuganoVizClusterGazebo
    gzserver = LuganoVizClusterGazebo()
    gzserver.start(ros_master_uri)
{% endif %}

    os.environ['GAZEBO_MASTER_URI'] = gzserver.gazebo_master_uri
    # We do not know here in which state the previous user did let us gzweb.
    gzweb.restart()

    empty_gazebo_world()

    cle_server.notify_current_task("Loading experiment environment",
                                True,  # update_progress
                                True)  # block_ui
    load_gazebo_world_file(world_file)

    # Create interfaces to Gazebo
    cle_server.notify_current_task("Loading neuRobot",
                                True,  # update_progress
                                True)  # block_ui
    # spawn robot model
    load_gazebo_model_file('robot', '{{config.bodyModel}}')

    # control adapter
    roscontrol = RosControlAdapter()
    # communication adapter
    roscomm = RosCommunicationAdapter()
{% if is_not_none(config.extRobotController) %}    # optionally load external robot controllers
    robot_controller_filepath = os.path.join(models_path, '{{config.extRobotController}}')
    if os.path.isfile(robot_controller_filepath):
        cle_server.notify_current_task("Loading external robot controllers",
                                    True,  # update_progress
                                    True)  # block_ui
        res = subprocess.call([robot_controller_filepath, 'start'])
        if res > 0:
            logger.error("The external robot controller could not be loaded")
            __shutdown(cle_server)
            return
{% endif %}
    # Create interfaces to brain
    cle_server.notify_current_task("Loading neural Simulator NEST",
                                True,  # update_progress
                                True)  # block_ui
    # control adapter
    brainfilepath = '{{config.brainModel.file}}'
    if models_path is not None:
        brainfilepath = os.path.join(models_path, brainfilepath)
    braincontrol = PyNNControlAdapter(brainfilepath{% for p in config.brainModel.neuronGroup %},
                                      {{p.population}}={{print_neurons(p)}}{% endfor %})
    # communication adapter
    braincomm = PyNNCommunicationAdapter()


    # Create transfer functions manager
    cle_server.notify_current_task("Connecting neural simulator to neurobot",
                                True,  # update_progress
                                True)  # block_ui
    # tf manager
    tfmanager = nrp.config.active_node
    # set adapters
    tfmanager.robot_adapter = roscomm
    tfmanager.brain_adapter = braincomm


    # Create CLE
    cle = SerialClosedLoopEngine(roscontrol, roscomm, braincontrol, braincomm, tfmanager, TIMESTEP)
    # initialize everything
    cle.initialize()

    # Now that we have everything ready, we could prepare the simulation
    cle_server.prepare_simulation(cle, {{timeout}})
    # Loading is completed.
    cle_server.notify_finish_task()
    
    return [cle_server, models_path, gzweb, gzserver]

def shutdown(cle_server, models_path, gzweb, gzserver):
    from hbp_nrp_cle.robotsim.GazeboLoadingHelper import empty_gazebo_world

    # Once we do reach this point, the simulation is stopped and we could clean after ourselves.
    # Clean up gazebo after ourselves
    cle_server.notify_start_task("Stopping simulation",
                              "Emptying 3D world",
                              2, # number of subtasks
                              False)  # block_ui
    empty_gazebo_world()

    gzweb.stop()
    gzserver.stop()

{% if is_not_none(config.extRobotController) %}    # optionally stop all external robot controllers
    robot_controller_filepath = os.path.join(models_path, '{{config.extRobotController}}')
    if os.path.isfile(robot_controller_filepath):
        cle_server.notify_current_task("Stopping external robot controllers",
                                    True,  # update_progress
                                    False)  # block_ui
        subprocess.check_call([robot_controller_filepath, 'stop'])
{% endif %}
    # Shutdown CLE
    Notificator.register_notification_function(
        lambda subtask, update_progress: cle_server.notify_current_task(subtask, update_progress,\
        False)
    )
    cle_server.notify_current_task("Shutting down Closed Loop Engine",
                                True,  # update_progress
                                False)  # block_ui
    # we could close the notify task here but it will be closed in any case by shutdown()
    cle_server.shutdown()
    # shutdown is complete
