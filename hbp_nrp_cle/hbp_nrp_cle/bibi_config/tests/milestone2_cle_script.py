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


def cle_function(world_file):

    from hbp_nrp_cle.cle.ROSCLEServer import ROSCLEServer

    # Create ROS server
    cle_server = ROSCLEServer()
    Notificator.register_notification_function(
        lambda subtask, update_progress: cle_server.notify_current_task(subtask, update_progress, True)
    )

    cle_server.notify_start_task("Initializing the Neurorobotic Closed Loop Engine",
                                 "Importing needed packages",
                                 5, # number of subtasks
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

    import geometry_msgs.msg #import Twist
    import hbp_nrp_cle.tf_framework.tf_lib #import detect_red
    import sensor_msgs.msg #import Image

    # import transfer functions specified in Python



    @nrp.MapSpikeSink("left_wheel_neuron", nrp.brain.actors[1], nrp.population_rate)
    @nrp.Neuron2Robot(Topic('/monitor/population_rate', cle_ros_msgs.msg.SpikeRate))
    def left_wheel_neuron_monitor(t, left_wheel_neuron):
        return cle_ros_msgs.msg.SpikeRate(t, left_wheel_neuron.rate, "left_wheel_neuron_monitor")



    @nrp.MapSpikeSink("left_wheel_neuron", nrp.brain.actors[1], nrp.leaky_integrator_alpha)
    @nrp.MapSpikeSink("right_wheel_neuron", nrp.brain.actors[2], nrp.leaky_integrator_alpha)
    @nrp.Neuron2Robot(Topic('/husky/cmd_vel', geometry_msgs.msg.Twist))
    def linear_twist(t, left_wheel_neuron, right_wheel_neuron):




        return geometry_msgs.msg.Twist(linear=geometry_msgs.msg.Vector3(x=20.0 * min(left_wheel_neuron.voltage, right_wheel_neuron.voltage), y=0.0, z=0.0), angular=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=100.0 * (right_wheel_neuron.voltage - left_wheel_neuron.voltage)))


    @nrp.MapRobotSubscriber("camera", Topic('/husky/camera', sensor_msgs.msg.Image))
    @nrp.MapSpikeSource("red_left_eye", nrp.brain.sensors[slice(0, 3, 2)], nrp.poisson)
    @nrp.MapSpikeSource("red_right_eye", nrp.brain.sensors[slice(1, 4, 2)], nrp.poisson)
    @nrp.MapSpikeSource("green_blue_eye", nrp.brain.sensors[4], nrp.poisson)
    @nrp.Robot2Neuron()
    def eye_sensor_transmit(t, camera, red_left_eye, red_right_eye, green_blue_eye):

        image_results = hbp_nrp_cle.tf_framework.tf_lib.detect_red(image=camera.value)

        red_left_eye.rate = 1000.0 * image_results.left
        red_right_eye.rate = 1000.0 * image_results.right
        green_blue_eye.rate = 1000.0 * image_results.go_on



    # consts
    TIMESTEP = 0.01
    MAX_SIM_TIME = 5

    # set models path variable
    models_path = os.environ.get('NRP_MODELS_DIRECTORY')

    Notificator.notify("Resetting Gazebo robotic simulator", True)

    local_ip = netifaces.ifaddresses('eth0')[netifaces.AF_INET][0]['addr']
    ros_master_uri = os.environ.get("ROS_MASTER_URI")
    ros_master_uri = ros_master_uri.replace('localhost', local_ip)

    from hbp_nrp_cle.robotsim.LocalGazebo import LocalGazeboBridgeInstance
    gzweb = LocalGazeboBridgeInstance()


    from hbp_nrp_cle.robotsim.LocalGazebo import LocalGazeboServerInstance
    gzserver = LocalGazeboServerInstance()
    gzserver.start(ros_master_uri)


    os.environ['GAZEBO_MASTER_URI'] = gzserver.gazebo_master_uri
    gzweb.start()

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
    load_gazebo_model_file('robot', 'husky_model/model.sdf')

    # control adapter
    roscontrol = RosControlAdapter()
    # communication adapter
    roscomm = RosCommunicationAdapter()

    # Create interfaces to brain
    cle_server.notify_current_task("Loading neural Simulator NEST",
                                True,  # update_progress
                                True)  # block_ui
    # control adapter
    brainfilepath = 'brain_model/braitenberg.h5'
    if models_path is not None:
        brainfilepath = os.path.join(models_path, brainfilepath)
    braincontrol = PyNNControlAdapter(brainfilepath,
                                      sensors=slice(0, 5),
                                      actors=slice(5, 8))
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
    cle_server.prepare_simulation(cle, 300.0)
    # Loading is completed.
    cle_server.notify_finish_task()
    
    # Main infinite loop (until the ROS stop service is called)
    cle_server.main()
    __shutdown(cle_server, models_path, gzweb, gzserver)


def __shutdown(cle_server, models_path, gzweb, gzserver):
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