"""
Tests the Gazebo loading helper class.
"""

import unittest
import os
from geometry_msgs.msg import Point, Pose, Quaternion
from lxml import etree, objectify
from mock import patch, call, MagicMock
from hbp_nrp_cle.robotsim import GZROS_S_SPAWN_SDF_ENTITY, GZROS_S_GET_WORLD_PROPERTIES, \
    GZROS_S_SET_MODEL_STATE, GZROS_S_DELETE_MODEL, GZROS_S_DELETE_LIGHT, GZROS_S_DELETE_LIGHTS, \
    GZROS_S_GET_LIGHTS_NAME, GZROS_S_WAIT_FOR_RENDERING
from hbp_nrp_cle.robotsim.GazeboHelper import GazeboHelper
from testfixtures import LogCapture


class TestGazeboHelper(unittest.TestCase):

    def setUp(self):
        self.mock_env = patch('hbp_nrp_cle.robotsim.GazeboHelper.os.environ').start()
        self.mock_env.get = MagicMock(return_value=None)
        self.mock_wait_for_service = patch('hbp_nrp_cle.robotsim.GazeboHelper.rospy.wait_for_service').start()
        self.mock_service_proxy = patch('hbp_nrp_cle.robotsim.GazeboHelper.rospy.ServiceProxy').start()
        self.gazebo_helper = GazeboHelper()

    def tearDown(self):
        self.mock_env.stop()
        self.mock_wait_for_service.stop()
        self.mock_service_proxy.stop()

    def test_gazebo_helper_init(self):
        waited = sorted([self.mock_wait_for_service.call_args_list[x][0][0]
            for x in xrange(len(self.mock_wait_for_service.call_args_list))])
        proxied = sorted([self.mock_service_proxy.call_args_list[x][0][0]
            for x in xrange(len(self.mock_service_proxy.call_args_list))])
        services = sorted([
            GZROS_S_SPAWN_SDF_ENTITY,
            GZROS_S_GET_WORLD_PROPERTIES,
            GZROS_S_SET_MODEL_STATE,
            GZROS_S_DELETE_MODEL,
            GZROS_S_DELETE_LIGHT,
            GZROS_S_DELETE_LIGHTS,
            GZROS_S_GET_LIGHTS_NAME,
            GZROS_S_WAIT_FOR_RENDERING
        ])

        self.assertEquals(services, waited)
        self.assertEquals(services, proxied)

    def test_load_gazebo_model_file(self):
        self.gazebo_helper.load_sdf_entity = MagicMock()

        with LogCapture('hbp_nrp_cle.user_notifications') as logcapture:
            test_pose = Pose()
            test_pose.position = Point(1, 2, 3)
            test_pose.orientation = Quaternion(4, 5, 6, 7)
            wpath = os.path.join(os.path.abspath(os.path.dirname(__file__)), "sample_model.sdf")
            self.gazebo_helper.load_gazebo_model_file("toto", wpath, test_pose)

            self.assertEqual(self.gazebo_helper.load_sdf_entity.call_args_list[0][0][0], "toto")

            actual_XML = objectify.fromstring(self.gazebo_helper.load_sdf_entity.call_args_list[0][0][1])
            actual_normalized_string = etree.tostring(actual_XML)
            expected_XML = objectify.fromstring("""<?xml version="1.0" ?>
                <sdf version="1.5">
                  <model name='vr_poster'>
                    <pose>0 0 0 0 0 0</pose>
                    <static>1</static>
                    <link name='body'>
                      <visual name='vr_poster'>
                        <cast_shadows>1</cast_shadows>
                        <geometry>
                          <mesh>
                            <uri>model://viz_poster/meshes/viz_poster.dae</uri>
                          </mesh>
                        </geometry>
                      </visual>
                    </link>
                  </model>
                </sdf>
                """)
            expected_normalized_string = etree.tostring(expected_XML)
            self.assertEqual(actual_normalized_string, expected_normalized_string)

            self.assertEqual(self.gazebo_helper.load_sdf_entity.call_count, 1)
            self.assertEqual(self.gazebo_helper.load_sdf_entity.call_args_list[0][0][2], test_pose)
            logcapture.check(('hbp_nrp_cle.user_notifications', 'DEBUG',
                              '%s successfully loaded in Gazebo' % wpath))

    def test_parse_world_file(self):

        def file_to_normalized_xml_string(file_name):
            file_path = os.path.join(os.path.abspath(os.path.dirname(__file__)), file_name)
            with open(file_path, 'r') as file:
                sdf_string = file.read()
            return normalize_xml(sdf_string)

        def normalize_xml(xml_string):
            sdf_XML = objectify.fromstring(xml_string)
            return etree.tostring(sdf_XML) # normalised sdf

        sample_world_sdf = file_to_normalized_xml_string("sample_world.sdf")

        # call target function
        models, lights = GazeboHelper.parse_world_file(sample_world_sdf)

        normalised_sun1_sdf = file_to_normalized_xml_string('sun1.sdf')
        normalised_sun2_sdf = file_to_normalized_xml_string('sun2.sdf')
        normalised_ground_plane_sdf = file_to_normalized_xml_string('ground_plane.sdf')

        self.assertEquals(normalize_xml(models['ground_plane']['model_sdf']), normalised_ground_plane_sdf)
        self.assertEquals(normalize_xml(lights['sun1']), normalised_sun1_sdf)
        self.assertEquals(normalize_xml(lights['sun2']), normalised_sun2_sdf)

    def test_load_gazebo_world(self):
        self.gazebo_helper.load_sdf_entity = MagicMock()

        fake_sdf = '<sdf/>'
        fake_models = {'ground_plane': {'model_sdf': fake_sdf, 'model_state_sdf': fake_sdf}}
        fake_lights = {'sun1': fake_sdf, 'sun2': fake_sdf}

        self.gazebo_helper.load_gazebo_world(fake_models, fake_lights)

        self.assertEqual(len(self.gazebo_helper.load_sdf_entity.call_args_list), 3)
        self.gazebo_helper.load_sdf_entity.assert_any_call("sun1", fake_sdf)
        self.gazebo_helper.load_sdf_entity.assert_any_call("sun2", fake_sdf)
        self.gazebo_helper.load_sdf_entity.assert_any_call("ground_plane", fake_sdf)

    def test_load_gazebo_world_file(self):
        self.gazebo_helper.load_gazebo_world = MagicMock()

        wpath = os.path.join(os.path.abspath(os.path.dirname(__file__)), "sample_world.sdf")

        # models = {'ground_plane': {'model_sdf': sdfGP, 'model_state_sdf': None} }
        # lights = {'sun1': sdf1, 'sun2': sdf1}
        models, lights = self.gazebo_helper.load_gazebo_world_file(wpath)

        self.gazebo_helper.load_gazebo_world.assert_called_with(models, lights)

    def test_load_sdf_entity(self):
        instance = self.gazebo_helper.spawn_entity_proxy
        sdf_xml = """<?xml version="1.0" ?>
        <sdf version="1.5">
          <model name='vr_poster'>
            <pose>0 0 0 0 0 0</pose>
            <static>1</static>
            <link name='body'>
              <visual name='vr_poster'>
                <cast_shadows>1</cast_shadows>
                <geometry>
                  <mesh>
                    <uri>model://viz_poster/meshes/viz_poster.dae</uri>
                  </mesh>
                </geometry>
              </visual>
            </link>
          </model>
        </sdf>"""
        self.gazebo_helper.load_sdf_entity("toto", sdf_xml)
        arg_initial_pose = self.gazebo_helper.spawn_entity_proxy.call_args_list[0][0][3]
        ptn = arg_initial_pose.position
        orn = arg_initial_pose.orientation
        self.assertEquals((ptn.x, ptn.y, ptn.z), (0, 0, 0))
        self.assertEquals((orn.x, orn.y, orn.z, orn.w), (0, 0, 0, 1))
        self.assertEquals(instance.call_args_list[0][0][0], "toto")

        # Testing with given pose
        test_pose = Pose()
        test_pose.position = Point(5, 3, 5)
        test_pose.orientation = Quaternion(1, 2, 3, 4)
        self.gazebo_helper.load_sdf_entity("toto", sdf_xml, test_pose)
        arg_initial_pose = self.gazebo_helper.spawn_entity_proxy.call_args_list[1][0][3]
        self.assertEquals(arg_initial_pose, test_pose)

        # Testing with invalid XML
        self.assertRaises(etree.XMLSyntaxError, self.gazebo_helper.load_sdf_entity, "toto", "invalid XML string")

        sdf_xml = """<?xml version="1.0" ?>
        <sdf version="1.5">
            <light name='sun2' type='directional'>
              <cast_shadows>0</cast_shadows>
              <pose>0 0 10 0 -0 0</pose>
              <diffuse>0.4 0.4 0.4 1</diffuse>
              <specular>0.1 0.1 0.1 1</specular>
              <direction>0.8 0 0.6</direction>
              <attenuation>
                <range>20</range>
                <constant>0.5</constant>
                <linear>0.01</linear>
                <quadratic>0.001</quadratic>
              </attenuation>
            </light>
        </sdf>"""
        self.gazebo_helper.load_sdf_entity("light", sdf_xml)
        arg_initial_pose = self.gazebo_helper.spawn_entity_proxy.call_args_list[2][0][3]
        ptn = arg_initial_pose.position
        orn = arg_initial_pose.orientation
        self.assertEquals((ptn.x, ptn.y, ptn.z), (0, 0, 0))
        self.assertEquals((orn.x, orn.y, orn.z, orn.w), (0, 0, 0, 1))
        self.assertEquals(instance.call_args_list[2][0][0], "light")

        # Testing with given pose
        test_pose = Pose()
        test_pose.position = Point(5, 3, 5)
        test_pose.orientation = Quaternion(1, 2, 3, 4)
        self.gazebo_helper.load_sdf_entity("light", sdf_xml, test_pose)
        arg_initial_pose = self.gazebo_helper.spawn_entity_proxy.call_args_list[3][0][3]
        self.assertEquals(arg_initial_pose, test_pose)

        # Testing with invalid XML
        self.assertRaises(etree.XMLSyntaxError, self.gazebo_helper.load_sdf_entity, "light", "invalid XML string")

    def test_empty_gazebo_world(self):
        self.gazebo_helper.empty_gazebo_world()
        self.assertGreater(self.mock_service_proxy.call_count, 0)
        self.assertGreater(self.mock_wait_for_service.call_count, 0)

    def test_set_model_pose(self):
      none_pose = Pose()
      none_pose.position = Point(0, 0, 0)
      none_pose.orientation = Quaternion(0, 0, 0, 1)

      self.gazebo_helper.set_model_pose('robot', None)

      arg_model_state = self.gazebo_helper.set_model_state_proxy.call_args_list[0][0][0]
      self.assertEquals(arg_model_state.model_name, 'robot')
      self.assertEquals(arg_model_state.pose, none_pose)

      custom_pose = Pose()
      custom_pose.position = Point(0, 7, 0)
      custom_pose.orientation = Quaternion(0, 0, 0, 1)

      self.gazebo_helper.set_model_pose('robot', custom_pose)

      arg_model_state = self.gazebo_helper.set_model_state_proxy.call_args_list[1][0][0]
      self.assertEquals(arg_model_state.model_name, 'robot')
      self.assertEquals(arg_model_state.pose, custom_pose)

if __name__ == "__main__":
    unittest.main()
