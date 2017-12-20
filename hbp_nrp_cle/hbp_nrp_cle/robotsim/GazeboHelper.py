# ---LICENSE-BEGIN - DO NOT CHANGE OR MOVE THIS HEADER
# This file is part of the Neurorobotics Platform software
# Copyright (C) 2014,2015,2016,2017 Human Brain Project
# https://www.humanbrainproject.eu
#
# The Human Brain Project is a European Commission funded project
# in the frame of the Horizon2020 FET Flagship plan.
# http://ec.europa.eu/programmes/horizon2020/en/h2020-section/fet-flagships
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
# ---LICENSE-END
"""
Helper class for gazebo loading operations
"""
from hbp_nrp_cle.robotsim import GZROS_S_SPAWN_SDF_ENTITY, GZROS_S_GET_WORLD_PROPERTIES, \
    GZROS_S_SET_MODEL_STATE, GZROS_S_DELETE_MODEL, GZROS_S_DELETE_LIGHT, GZROS_S_DELETE_LIGHTS, \
    GZROS_S_GET_LIGHTS_NAME, GZROS_S_WAIT_FOR_RENDERING

__author__ = "Stefan Deser, Georg Hinkel, Luc Guyot"

import rospy
import os
import tf.transformations
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SpawnEntity, GetWorldProperties, DeleteModel, SetModelState, \
    GetLightsName, DeleteLight
from std_srvs.srv import Empty
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from lxml import etree
import logging

# Info messages sent to this logger will be forwarded as notifications
logger = logging.getLogger('hbp_nrp_cle.user_notifications')

TIMEOUT = 180  # duplicated ROSCLEClient.ROSCLEClient.ROS_SERVICE_TIMEOUT


class GazeboHelper(object):
    """
    This class provides some useful Gazebo macros by wrapping ROS services.
    """

    def __init__(self):
        rospy.wait_for_service(GZROS_S_SPAWN_SDF_ENTITY, TIMEOUT)
        rospy.wait_for_service(GZROS_S_GET_WORLD_PROPERTIES, TIMEOUT)
        rospy.wait_for_service(GZROS_S_SET_MODEL_STATE, TIMEOUT)
        rospy.wait_for_service(GZROS_S_DELETE_MODEL, TIMEOUT)
        rospy.wait_for_service(GZROS_S_DELETE_LIGHT, TIMEOUT)
        rospy.wait_for_service(GZROS_S_DELETE_LIGHTS, TIMEOUT)
        rospy.wait_for_service(GZROS_S_GET_LIGHTS_NAME, TIMEOUT)
        rospy.wait_for_service(GZROS_S_WAIT_FOR_RENDERING, TIMEOUT)

        self.spawn_entity_proxy = rospy.ServiceProxy(GZROS_S_SPAWN_SDF_ENTITY, SpawnEntity)
        self.get_world_properties_proxy = rospy.ServiceProxy(GZROS_S_GET_WORLD_PROPERTIES,
                                                             GetWorldProperties)
        self.set_model_state_proxy = rospy.ServiceProxy(GZROS_S_SET_MODEL_STATE, SetModelState)
        self.delete_model_proxy = rospy.ServiceProxy(GZROS_S_DELETE_MODEL, DeleteModel)
        self.delete_light_proxy = rospy.ServiceProxy(GZROS_S_DELETE_LIGHT, DeleteLight)
        self.delete_lights_proxy = rospy.ServiceProxy(GZROS_S_DELETE_LIGHTS, Empty)
        self.get_lights_name_proxy = rospy.ServiceProxy(GZROS_S_GET_LIGHTS_NAME, GetLightsName)
        self.wait_for_rendering_proxy = rospy.ServiceProxy(GZROS_S_WAIT_FOR_RENDERING, Empty)

    def load_gazebo_world_file(self, world_file):
        """
        Load an SDF world file into the ROS connected gazebo running instance.

        :param world_file: The absolute path of the SDF world file.
        :return A pair of dictionaries:
            1st containing pairs (model_name: {'model_sdf': sdf, 'model_state_sdf': sdf}),
            2nd second pairs (light_name: light_sdf)
        """

        with open(world_file, 'r') as world_file_sdf:
            world_sdf_string = world_file_sdf.read()

        world_models_sdf, world_lights_sdf = GazeboHelper.parse_world_string(world_sdf_string)

        self.load_gazebo_world(world_models_sdf, world_lights_sdf)

        return world_models_sdf, world_lights_sdf

    @staticmethod
    def parse_gazebo_world_file(world_file):
        """
        Parse an SDF world file.

        :param world_file: The absolute path of the SDF world file.
        :return A pair of dictionaries:
            1st containing pairs (model_name: {'model_sdf': sdf, 'model_state_sdf': sdf}),
            2nd second pairs (light_name: light_sdf)
        """

        with open(world_file, 'r') as world_file_sdf:
            world_sdf_string = world_file_sdf.read()

        return GazeboHelper.parse_world_string(world_sdf_string)

    @staticmethod
    def try_remove_pose(model):
        """
        Remove pose from the model object if it exists

        :param: A model object
        """

        pose = model.find("pose")
        if pose:
            model.remove(pose)  # remove model pose

    @staticmethod
    def parse_world_string(world_string):
        """
        Parse an SDF world string producing a pair of dictionaries:
        - (model_name: (model_sdf, model_state_sdf))
        - (light_name: light_sdf)

        :param world_string: A string containing the SDF world.
        :return A pair of dictionaries,
            the 1st containing pairs (model_name: {'model_sdf': sdf, 'model_staet_sdf': sdf},
            the 2nd pairs (light_name: light_sdf)
        """
        world_file_sdf = etree.fromstring(world_string)

        world_lights_sdf = {}
        world_models_sdf = {}

        sdf_wrapper = "<?xml version=\"1.0\" ?>\n<sdf version='1.5'>%s</sdf>"

        # Load lights
        for light in world_file_sdf.xpath("/sdf/world/light"):
            light_name = light.xpath("@name")[0]
            light_sdf = sdf_wrapper % (etree.tostring(light), )
            world_lights_sdf[light_name] = light_sdf

        models_state = world_file_sdf.xpath("/sdf/world/state/model")

        # Load models
        for model in world_file_sdf.xpath("/sdf/world/model"):
            model_name = model.xpath("@name")[0]
            # Checking whether some extra state is defined in the SDF
            state = [x for x in models_state if x.xpath("@name")[0] == model_name]
            if len(state) != 0:
                model_state_sdf = etree.tostring(state[0])
                GazeboHelper.try_remove_pose(model)
                model.append(state[0].find("pose"))  # apply state pose
            else:
                model_state_sdf = None

            model_sdf = sdf_wrapper % (etree.tostring(model), )
            world_models_sdf[model_name] = \
                {'model_sdf': model_sdf, 'model_state_sdf': model_state_sdf}

        return world_models_sdf, world_lights_sdf

    @staticmethod
    def parse_model_state_sdf(model_state_sdf):
        """
        Create a ModelState message using a model state sdf element

        :param model_state_sdf: A string serialization of a model state sdf element
        :return: A ModelState message
        """

        if model_state_sdf is None or model_state_sdf == '':
            return None

        model_state_root = etree.fromstring(model_state_sdf)
        #no twist informations in sdf files

        model_name = model_state_root.get('name')
        scale_elem = model_state_root.find('scale')
        scale_str = scale_elem.text if scale_elem is not None else None

        pose_elem = model_state_root.find('pose')
        pose_str = pose_elem.text if pose_elem is not None else None

        reference_frame = pose_elem.get('frame') if pose_elem is not None else None

        #parse pose and scale
        if (scale_str is not None) and (pose_str is not None):
            scale_values = [float(n) for n in scale_str.split(' ')]  # [x, y ,z]
            pose_values = [float(n) for n in pose_str.split(' ')]  # [x, y ,z, roll, pitch, yaw]

            pose = Pose()
            pose.position = Point(*pose_values[0:3])
            pose.orientation = \
                Quaternion(*tf.transformations.quaternion_from_euler(*pose_values[3:6]))
            scale = Vector3(*scale_values[0:3])
        else:
            pose = None
            scale = None

        return GazeboHelper.make_model_state_msg(model_name, pose, scale,
                                                 reference_frame=reference_frame)

    def load_gazebo_world(self, models_sdf_dict, lights_sdf_dict):
        """
        Load a SDF world into the ROS connected gazebo running instance.
        The world is described by the models_sdf_dict and lights_sdf_dict dictionaries.

        :param models_sdf_dict: A dictionary containing pairs
            (model_name: {'model_sdf': sdf, 'model_state_sdf': sdf})
        :param lights_sdf_dict: A dictionary containing pairs (light_name: light_sdf)
        """

        # Load lights
        for light_name, light_sdf in lights_sdf_dict.items():
            # This call will produce errors on the console in this form:
            # [ WARN] [1422519240.507654550, 234.622000000]: Could not find <model>
            # or <world> element in sdf, so name and initial position cannot be applied
            # This is because ROS is based on an old and deprecated version of SDF.
            # Anyway, regardless of the warning, the lights are loaded with their correct
            # positions.
            logger.info("Loading light \"%s\".", light_name)
            self.load_sdf_entity(light_name, light_sdf)

        # Load models
        for curr_model_name, curr_model_sdf in models_sdf_dict.items():
            logger.info("Loading model \"%s\".", curr_model_name)

            model_state_sdf = curr_model_sdf['model_state_sdf']
            model_sdf = curr_model_sdf['model_sdf']

            self.load_sdf_entity(curr_model_name, model_sdf)
            self.set_model_state(model_state_sdf)

        logger.debug("World successfully loaded in Gazebo.")

    def load_gazebo_model_file(self, model_name, model_file, initial_pose=None,
                               retina_config_path=None):
        """
        Load a sdf model file into the ROS connected running gazebo instance.

        :param model_name: Name of the model (can be anything)
        :param model_file: The name of the model sdf file inside the \
            NRP_MODELS_DIRECTORY folder. If the NRP_MODELS_DIRECTORY \
            environment variable is not set, this script will search \
            the model in its own folder.\
        :param initial_pose: Initial pose of the model. Uses the Gazebo \
            "Pose" type.
        :param retina_config_path: Configuration script for the Retina Camera Plugin
        """
        model_file_path = os.path.join(os.environ.get('NRP_MODELS_DIRECTORY'), model_file)
        with open(model_file_path, 'r') as model_file_sdf:

            model_sdf_str = model_file_sdf.read()

            # append retina_script_path to retina camera plugin element
            if retina_config_path is not None:

                model_root_elem = etree.fromstring(model_sdf_str)

                plugin_elem = \
                    model_root_elem.xpath("//sensor/plugin[@name='RetinaCameraPlugin']")[0]
                if plugin_elem is not None:
                    etree.SubElement(plugin_elem, "retinaScriptPath").text = retina_config_path
                    # get updated string serialization
                    model_sdf_str = etree.tostring(model_root_elem)
                else:
                    logger.error("Retina plugin element not found in model sdf file!")

            # spawn model
            self.load_sdf_entity(model_name, model_sdf_str, initial_pose)
            logger.debug("%s successfully loaded in Gazebo", model_file)

    def load_sdf_entity(self, model_name, model_sdf, initial_pose=None):
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
        self.spawn_entity_proxy(model_name, model_sdf, "", initial_pose, "world")

    def set_model_pose(self, model_name, pose):
        """
        Sets the pose of a given model to a given pose.
        :param model_name: The model to reposition.
        :param pose: The new pose of the model, if None, it will be set to origin w/o rotation.
        """
        msg = self.make_model_state_msg(model_name, pose)
        self.set_model_state_proxy(msg)

    def set_model_state(self, model_state_sdf):
        """
        Set a model state described by the model state sdf element string
        :param model_state_sdf A string serialization of the model state sdf element
        describing the model state to be set
        """

        msg = GazeboHelper.parse_model_state_sdf(model_state_sdf)

        if msg is not None:
            self.set_model_state_proxy(msg)

    @staticmethod
    def make_model_state_msg(model_name=None, pose=None, scale=None, twist=None,
                             reference_frame=None):
        """
        ModelState messages factory
        """

        msg = ModelState()

        msg.model_name = model_name if (model_name is not None) else 'model_name'

        if pose is None:
            pose = Pose()
            pose.position = Point(0, 0, 0)
            pose.orientation = Quaternion(0, 0, 0, 1)
        msg.pose = pose

        msg.scale = scale if (scale is not None) else Vector3(1, 1, 1)

        msg.twist = twist if (twist is not None) else Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))

        if reference_frame is None or reference_frame == '':
            msg.reference_frame = 'world'
        else:
            msg.reference_frame = reference_frame

        return msg

    def empty_gazebo_world(self):
        """
        Clean up the ROS connected running instance.
        Remove all models and all lights.
        """

        world_properties = self.get_world_properties_proxy()

        for model in world_properties.model_names:
            logger.info("Cleaning model %s.", model)
            self.delete_model_proxy(model)

        logger.info("Cleaning lights")
        self.delete_lights_proxy()

    def wait_for_backend_rendering(self):
        """
        Wait for the backend rendering environment to be ready. Blocks until all world models and
        lights are loaded in the rendering scene (only if needed, if there are no sensors or
        cameras then a scene will not exist and this will not block).

        This is necessary for sensors (e.g. cameras) that depend on the rendering environment to be
        immediately usable. The simulation will otherwise run, but the cameras and sensors will not
        publish data until they are fully loaded.
        """

        self.wait_for_rendering_proxy()
