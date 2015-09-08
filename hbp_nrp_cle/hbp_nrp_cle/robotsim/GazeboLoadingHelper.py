"""
Helper class for gazebo loading operations
"""
from hbp_nrp_cle.robotsim import ROS_S_SPAWN_SDF_LIGHT, ROS_S_SPAWN_SDF_MODEL

__author__ = "Stefan Deser, Georg Hinkel, Luc Guyot"

import rospy
import os
from gazebo_msgs.srv import SpawnModel, GetWorldProperties, DeleteModel
from std_srvs.srv import Empty
from geometry_msgs.msg import Point, Pose, Quaternion
from lxml import etree
import logging
from hbp_nrp_cle.bibi_config.notificator import Notificator

logger = logging.getLogger(__name__)

TIMEOUT = 180  # duplicated ROSCLEClient.ROSCLEClient.ROS_SERVICE_TIMEOUT


def load_gazebo_world_file(world_file):
    """
    Load a SDF world file into the ROS connected gazebo running instance.

    :param world_file: The absolute path of the SDF world file.
    """
    world_file_sdf = etree.parse(world_file)

    # Load lights
    for light in world_file_sdf.xpath("/sdf/world/light"):
        # This call will produce errors on the console in this form:
        # [ WARN] [1422519240.507654550, 234.622000000]: Could not find <model>
        # or <world> element in sdf, so name and initial position cannot be applied
        # This is because ROS is based on an old and deprecated version of SDF.
        # Anyway, regardless of the warning, the lights are loaded with their correct
        # positions.
        logger.info("Loading light \"%s\" in Gazebo", light.xpath("@name")[0])
        Notificator.notify("Loading light " + light.xpath("@name")[0], False)
        load_light_sdf(light.xpath("@name")[0],
                       "<?xml version=\"1.0\" ?>\n<sdf version='1.5'>" +
                       etree.tostring(light) +
                       "</sdf>")
    # Load models
    for model in world_file_sdf.xpath("/sdf/world/model"):
        logger.info("Loading model \"%s\" in Gazebo", model.xpath("@name")[0])
        Notificator.notify("Loading model " + model.xpath("@name")[0], False)
        load_gazebo_sdf(model.xpath("@name")[0],
                        "<?xml version=\"1.0\" ?>\n<sdf version='1.5'>" +
                        etree.tostring(model) +
                        "</sdf>")
    logger.info("%s successfully loaded in Gazebo", world_file)


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
    logger.info("%s successfully loaded in Gazebo", model_file)


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
    spawn_light_proxy(light_name, light_sdf, "", initial_pose, "")
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
    spawn_model_proxy(model_name, model_sdf, "", initial_pose, "")
    spawn_model_proxy.close()


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
        Notificator.notify("Cleaning model " + model, False)
        delete_model_proxy(model)
    delete_model_proxy.close()

    Notificator.notify("Cleaning lights", False)
    rospy.wait_for_service('gazebo/delete_lights', TIMEOUT)
    delete_lights_proxy = rospy.ServiceProxy('gazebo/delete_lights', Empty)
    delete_lights_proxy()
    delete_lights_proxy.close()
