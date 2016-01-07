"""
Helper class for gazebo loading operations
"""
from hbp_nrp_cle.robotsim import ROS_S_SPAWN_SDF_LIGHT, ROS_S_SPAWN_SDF_MODEL

__author__ = "Stefan Deser, Georg Hinkel, Luc Guyot"

import rospy
import os
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SpawnModel, GetWorldProperties, DeleteModel, SetModelState
from std_srvs.srv import Empty
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from lxml import etree
import logging

# Info messages sent to this logger will be forwarded as notifications
logger = logging.getLogger('hbp_nrp_cle.user_notifications')

TIMEOUT = 180  # duplicated ROSCLEClient.ROSCLEClient.ROS_SERVICE_TIMEOUT


def load_gazebo_world_file(world_file):
    """
    Load a SDF world file into the ROS connected gazebo running instance.

    :param world_file: The absolute path of the SDF world file.
    """
    world_file_sdf = etree.parse(world_file)

    sdf_wrapper = "<?xml version=\"1.0\" ?>\n<sdf version='1.5'>%s</sdf>"

    # Load lights
    for light in world_file_sdf.xpath("/sdf/world/light"):
        # This call will produce errors on the console in this form:
        # [ WARN] [1422519240.507654550, 234.622000000]: Could not find <model>
        # or <world> element in sdf, so name and initial position cannot be applied
        # This is because ROS is based on an old and deprecated version of SDF.
        # Anyway, regardless of the warning, the lights are loaded with their correct
        # positions.
        light_name = light.xpath("@name")[0]
        logger.info("Loading light \"%s\".", light_name)
        load_light_sdf(light_name, sdf_wrapper % (etree.tostring(light), ))

    models_state = world_file_sdf.xpath("/sdf/world/state/model")

    # Load models
    for model in world_file_sdf.xpath("/sdf/world/model"):
        model_name = model.xpath("@name")[0]
        logger.info("Loading model \"%s\".", model_name)
        # Checking whether some extra state is defined in the SDF
        state = [x for x in models_state if x.xpath("@name")[0] == model_name]
        if len(state) != 0:
            model.remove(model.find("pose"))
            model.append(state[0].find("pose"))
        load_gazebo_sdf(model_name, sdf_wrapper % (etree.tostring(model), ))

    logger.debug("%s successfully loaded in Gazebo", world_file)


def load_gazebo_model_file(model_name, model_file, initial_pose=None):
    """
    Load a sdf model file into the ROS connected running gazebo instance.

    :param model_name: Name of the model (can be anything)
    :param model_file: The name of the model sdf file inside the \
        NRP_MODELS_DIRECTORY folder. If the NRP_MODELS_DIRECTORY \
        environment variable is not set, this script will search \
        the model in its own folder.\
    :param initial_pose: Initial pose of the model. Uses the Gazebo \
        "Pose" type.
    """
    model_file_sdf = open(os.path.join(os.environ.get('NRP_MODELS_DIRECTORY'), model_file), 'r')
    model_sdf = model_file_sdf.read()
    model_file_sdf.close()
    # spawn model
    load_gazebo_sdf(model_name, model_sdf, initial_pose)
    logger.debug("%s successfully loaded in Gazebo", model_file)


def load_light_sdf(light_name, light_sdf, initial_pose=None):
    """
    Load a gazebo light (sdf) into the ROS connected running gazebo instance.

    :param light_name: Name of the light (can be anything).
    :param light_sdf: The SDF xml code describing the light.
    :param initial_pose: Initial pose of the light. Uses the Gazebo \
        "Pose" type.
    """
    # We are checking here that light_sdf is indeed an XML string, fromstring() raises
    # exception if the parameter is not a valid XML.
    etree.fromstring(light_sdf)
    # set initial pose
    if initial_pose is None:
        initial_pose = Pose()
        initial_pose.position = Point(0, 0, 0)
        initial_pose.orientation = Quaternion(0, 0, 0, 1)
    # spawn light
    rospy.wait_for_service(ROS_S_SPAWN_SDF_LIGHT, TIMEOUT)
    spawn_light_proxy = rospy.ServiceProxy(ROS_S_SPAWN_SDF_LIGHT, SpawnModel)
    # What if the service doesn't like the parameters?
    spawn_light_proxy(light_name, light_sdf, "", initial_pose, "world")
    spawn_light_proxy.close()


def load_gazebo_sdf(model_name, model_sdf, initial_pose=None):
    """
    Load a gazebo model (sdf) into the ROS connected running gazebo instance.

    :param model_name: Name of the model (can be anything).
    :param model_sdf: The SDF xml code describing the model.
    :param initial_pose: Initial pose of the model. Uses the Gazebo \
        "Pose" type.
    """
    # We are checking here that light_sdf is indeed an XML string, fromstring() raises
    # exception if the parameter is not a valid XML.
    etree.fromstring(model_sdf)
    # set initial pose
    if initial_pose is None:
        initial_pose = Pose()
        initial_pose.position = Point(0, 0, 0)
        initial_pose.orientation = Quaternion(0, 0, 0, 1)
    # spawn model
    rospy.wait_for_service(ROS_S_SPAWN_SDF_MODEL, TIMEOUT)
    spawn_model_proxy = rospy.ServiceProxy(ROS_S_SPAWN_SDF_MODEL, SpawnModel)
    spawn_model_proxy(model_name, model_sdf, "", initial_pose, "world")
    spawn_model_proxy.close()


def set_model_pose(model_name, pose):
    """
    Sets the pose of a given model to a given pose.
    :param model_name: The model to reposition.
    :param pose: The new pose of the model, if None, it will be set to origin w/o rotation.
    """

    if pose is None:
        pose = Pose()
        pose.position = Point(0, 0, 0)
        pose.orientation = Quaternion(0, 0, 0, 1)

    rospy.wait_for_service('gazebo/set_model_state', TIMEOUT)
    set_model_state_proxy = rospy.ServiceProxy('gazebo/set_model_state', SetModelState)
    msg = ModelState()
    msg.model_name = model_name
    msg.pose = pose
    msg.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
    msg.reference_frame = "world"
    set_model_state_proxy(msg)
    set_model_state_proxy.close()


def empty_gazebo_world():
    """
    Clean up the ROS connected running instance.
    Remove all models and all lights.
    """

    rospy.wait_for_service('gazebo/get_world_properties', TIMEOUT)
    get_world_properties_proxy = rospy.ServiceProxy('gazebo/get_world_properties',
                                                    GetWorldProperties)
    world_properties = get_world_properties_proxy()
    get_world_properties_proxy.close()

    rospy.wait_for_service('gazebo/delete_model', TIMEOUT)
    delete_model_proxy = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
    for model in world_properties.model_names:
        logger.info("Cleaning model " + model)
        delete_model_proxy(model)
    delete_model_proxy.close()

    logger.info("Cleaning lights")
    rospy.wait_for_service('gazebo/delete_lights', TIMEOUT)
    delete_lights_proxy = rospy.ServiceProxy('gazebo/delete_lights', Empty)
    delete_lights_proxy()
    delete_lights_proxy.close()
