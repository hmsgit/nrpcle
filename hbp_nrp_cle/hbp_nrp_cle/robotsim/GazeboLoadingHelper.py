"""
Helper class for gazebo loading operations
"""

import rospy
import os
from gazebo_msgs.srv import SpawnModel, GetWorldProperties, DeleteModel
from std_srvs.srv import Empty
from geometry_msgs.msg import Point, Pose, Quaternion
from lxml import etree


def load_gazebo_world_file(world_file):
    """
    Load a sdf world file into the ROS connected gazebo running instance.
    :param world_file: The name of the world inside the NRP_MODELS_DIRECTORY \
        folder. If the NRP_MODELS_DIRECTORY environment variable is not set, \
        this script will search the model in its own folder.
    """
    world_file_sdf = etree.parse(os.path.join(_get_basepath(__file__), world_file))

    # Load lights
    for light in world_file_sdf.xpath("/sdf/world/light"):
        # This call will produce errors on the console in this form:
        # [ WARN] [1422519240.507654550, 234.622000000]: Could not find <model>
        # or <world> element in sdf, so name and initial position cannot be applied
        # This is because ROS is based on an old and deprecated version of SDF.
        # Anyway, regardless of the warning, the lights are loaded with their correct
        # positions.
        load_gazebo_sdf(light.xpath("@name")[0],
                         "<?xml version=\"1.0\" ?>\n<sdf version='1.5'>" +
                         etree.tostring(light) +
                         "</sdf>")
    # Load models
    for model in world_file_sdf.xpath("/sdf/world/model"):
        load_gazebo_sdf(model.xpath("@name")[0],
                         "<?xml version=\"1.0\" ?>\n<sdf version='1.5'>" +
                         etree.tostring(model) +
                         "</sdf>")


def load_gazebo_model_file(model_name, model_file, initial_pose=None):
    """
    Load a sdf model file into the ROS connected running gazebo instance.
    :param model_name: Name of the model (can be anything)
    :param model_file: The name of the model sdf file inside the \
        NRP_MODELS_DIRECTORY folder. If the NRP_MODELS_DIRECTORY \
        environment variable is not set, this script will search \
        the model in its own folder.
    :param initial_pose: Initial pose of the model. Uses the Gazebo \
        "Pose" type.
    """

    model_file_sdf = open(os.path.join(_get_basepath(__file__), model_file), 'r')
    model_sdf = model_file_sdf.read()
    model_file_sdf.close()

    # spawn model
    load_gazebo_sdf(model_name, model_sdf, initial_pose)


def load_gazebo_sdf(model_name, model_sdf, initial_pose=None):
    """
    Load a gazebo model (sdf) into the ROS connected running gazebo instance.
    :param model_name: Name of the model (can be anything).
    :param model_sdf: The SDF xml code describing the model.
    :param initial_pose: Initial pose of the model. Uses the Gazebo \
        "Pose" type.
    """

    # set initial pose
    if initial_pose is None:
        initial_pose = Pose()
        initial_pose.position = Point(0, 0, 0)
        initial_pose.orientation = Quaternion(0, 0, 0, 1)

    # spawn model
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_model_prox = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    spawn_model_prox(model_name,
                     model_sdf,
                     "",
                     initial_pose,
                     "")
    spawn_model_prox.close()


def empty_gazebo_world():
    """
    Clean up the ROS connected running instance.
    Remove all models and all lights.
    """

    rospy.wait_for_service('gazebo/get_world_properties')
    get_world_properties_proxy = rospy.ServiceProxy('gazebo/get_world_properties',
                                                    GetWorldProperties)
    world_properties = get_world_properties_proxy()
    get_world_properties_proxy.close()

    rospy.wait_for_service('gazebo/delete_model')
    delete_model_proxy = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
    for model in world_properties.model_names:
        delete_model_proxy(model)
    delete_model_proxy.close()

    rospy.wait_for_service('gazebo/delete_lights')
    delete_lights_proxy = rospy.ServiceProxy('gazebo/delete_lights', Empty)
    delete_lights_proxy()
    delete_lights_proxy.close()


def _get_basepath(adjacent_file=None):
    """
    :return the basepath for retrieving Models / \
    There are three possible cases. They
    are evaluated in the following order: \
    1. The NRP_MODELS_DIRECTORY variable is set, then return the \
       content of this variable.
    2. The adjacent_file argument is set, then return the \
       basepath of this file.
    3. Return None.
    """
    path = os.environ.get('NRP_MODELS_DIRECTORY')
    if (path is None) and (adjacent_file is not None):
        path = os.path.dirname(adjacent_file)
    return path
