"""
Tests the Gazebo loading helper class.
"""

import unittest
import os
from geometry_msgs.msg import Point, Pose, Quaternion
from gazebo_msgs.srv import SetModelState
from lxml import etree, objectify
from mock import patch, call, MagicMock, Mock
from hbp_nrp_cle.robotsim import ROS_S_SPAWN_SDF_LIGHT, ROS_S_SPAWN_SDF_MODEL
from hbp_nrp_cle.robotsim.GazeboLoadingHelper import load_light_sdf, load_gazebo_sdf, load_gazebo_model_file, load_gazebo_world_file, \
    empty_gazebo_world, set_model_pose, TIMEOUT
from testfixtures import log_capture, LogCapture


class TestGazeboLoadingHelper(unittest.TestCase):

    def setUp(self):
        self.mock_env = patch('hbp_nrp_cle.robotsim.GazeboLoadingHelper.os.environ').start()
        self.mock_env.get = MagicMock(return_value=None)

    def tearDown(self):
        self.mock_env.stop()

    @patch('hbp_nrp_cle.robotsim.GazeboLoadingHelper.load_gazebo_sdf')
    def test_load_gazebo_model_file(self, mocked_load_gazebo_sdf):
        with LogCapture('hbp_nrp_cle.robotsim.GazeboLoadingHelper') as logcapture:
            test_pose = Pose()
            test_pose.position = Point(1, 2, 3)
            test_pose.orientation = Quaternion(4, 5, 6, 7)
            wpath = os.path.join(os.path.abspath(os.path.dirname(__file__)), "sample_model.sdf")
            load_gazebo_model_file("toto", wpath, test_pose)

            self.assertEqual(mocked_load_gazebo_sdf.call_args_list[0][0][0], "toto")

            actual_XML = objectify.fromstring(mocked_load_gazebo_sdf.call_args_list[0][0][1])
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

            self.assertEqual(mocked_load_gazebo_sdf.call_count, 1)
            self.assertEqual(mocked_load_gazebo_sdf.call_args_list[0][0][2], test_pose)
            logcapture.check(('hbp_nrp_cle.robotsim.GazeboLoadingHelper', 'INFO',
                              '%s successfully loaded in Gazebo' % wpath))


    @patch('hbp_nrp_cle.robotsim.GazeboLoadingHelper.load_light_sdf')
    @patch('hbp_nrp_cle.robotsim.GazeboLoadingHelper.load_gazebo_sdf')
    def test_load_gazebo_world_file(self, mocked_load_gazebo_sdf, mocked_load_light_sdf):
        with LogCapture('hbp_nrp_cle.robotsim.GazeboLoadingHelper') as logcapture:
            wpath = os.path.join(os.path.abspath(os.path.dirname(__file__)), "sample_world.sdf")
            load_gazebo_world_file(wpath)
            expected_load_gazebo_calls_args = [["ground_plane","""<?xml version=\"1.0\" ?>\n<sdf version="1.5"><model name="ground_plane">
            <static>1</static>
            <link name="link">
              <collision name="collision">
                 <geometry>
                  <plane>
                    <normal>0 0 1</normal>
                    <size>100 100</size>
                  </plane>
                </geometry>
                <surface>
                  <friction>
                    <ode>
                      <mu>100</mu>
                      <mu2>50</mu2>
                    </ode>
                  </friction>
                  <contact>
                    <ode/>
                  </contact>
                  <bounce/>
                </surface>
                <max_contacts>10</max_contacts>
              </collision>
              <visual name="visual">
                <cast_shadows>0</cast_shadows>
                <geometry>
                  <plane>
                    <normal>0 0 1</normal>
                    <size>100 100</size>
                  </plane>
                </geometry>
                <material>
                  <script>
                    <uri>file://media/materials/scripts/gazebo.material</uri>
                    <name>Gazebo/Grey</name>
                  </script>
                </material>
              </visual>
            </link></model></sdf>"""]]

            for i, args in enumerate(expected_load_gazebo_calls_args):
                for (j, (a, b)) in enumerate(zip(mocked_load_gazebo_sdf.call_args_list[i][0], args)):
                    if j % 2:
                        # TODO: use an external library to compare XML trees, or avoid comparison
                        pass
                    else:
                        self.assertEquals(a, b)

            expected_load_light_calls_args = [["sun1","""<?xml version=\"1.0\" ?>\n<sdf version="1.5"><light name="sun1" type="directional">
            <cast_shadows>0</cast_shadows>
            <pose>0 0 10 0 -0 0</pose>
            <diffuse>0.3 0.3 0.3 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
            <direction>-0.8 0 0.6</direction>
            <attenuation>
              <range>20</range>
              <constant>0.5</constant>
              <linear>0.01</linear>
              <quadratic>0.001</quadratic>
            </attenuation> </light></sdf>"""],["sun2","""<?xml version=\"1.0\" ?>\n<sdf version="1.5"><light name="sun2" type="directional">
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
            </attenuation></light></sdf>"""]]

            for i, args in enumerate(expected_load_light_calls_args):
                for (j, (a, b)) in enumerate(zip(mocked_load_light_sdf.call_args_list[i][0], args)):
                    if j % 2:
                        # TODO: use an external library to compare XML trees, or avoid comparison
                        pass
                    else:
                        self.assertEquals(a, b)

            self.assertEqual(len(mocked_load_gazebo_sdf.call_args_list), 1)
            self.assertEqual(len(mocked_load_light_sdf.call_args_list), 2)
            logcapture.check(('hbp_nrp_cle.robotsim.GazeboLoadingHelper', 'INFO',
                              'Loading light "sun1" in Gazebo.'),
                             ('hbp_nrp_cle.robotsim.GazeboLoadingHelper', 'INFO',
                              'Loading light "sun2" in Gazebo.'),
                             ('hbp_nrp_cle.robotsim.GazeboLoadingHelper', 'INFO',
                              'Loading model "ground_plane" in Gazebo.'),
                             ('hbp_nrp_cle.robotsim.GazeboLoadingHelper', 'INFO',
                              '%s successfully loaded in Gazebo' % wpath))

    @patch('hbp_nrp_cle.robotsim.GazeboLoadingHelper.rospy.wait_for_service')
    @patch('hbp_nrp_cle.robotsim.GazeboLoadingHelper.rospy.ServiceProxy')
    def test_load_gazebo_sdf(self, mock_proxy, mock_wait_service):
        mock_service_proxy_callee = Mock()
        mock_proxy.return_value = mock_service_proxy_callee
        instance = mock_proxy.return_value
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
        load_gazebo_sdf("toto", sdf_xml)
        arg_initial_pose = mock_service_proxy_callee.call_args_list[0][0][3]
        ptn = arg_initial_pose.position
        orn = arg_initial_pose.orientation
        self.assertEquals((ptn.x, ptn.y, ptn.z), (0, 0, 0))
        self.assertEquals((orn.x, orn.y, orn.z, orn.w), (0, 0, 0, 1))
        self.assertEquals(mock_proxy.call_args_list[0][0][0], ROS_S_SPAWN_SDF_MODEL)
        self.assertEquals(instance.call_args_list[0][0][0], "toto")
        self.assertEquals(mock_wait_service.call_args_list[0][0][0], ROS_S_SPAWN_SDF_MODEL)

        # Testing with given pose
        test_pose = Pose()
        test_pose.position = Point(5, 3, 5)
        test_pose.orientation = Quaternion(1, 2, 3, 4)
        load_gazebo_sdf("toto", sdf_xml, test_pose)
        arg_initial_pose = mock_service_proxy_callee.call_args_list[1][0][3]
        self.assertEquals(arg_initial_pose, test_pose)

        # Testing with invalid XML
        self.assertRaises(etree.XMLSyntaxError, load_gazebo_sdf, "toto", "invalid XML string")

    @patch('hbp_nrp_cle.robotsim.GazeboLoadingHelper.rospy.wait_for_service')
    @patch('hbp_nrp_cle.robotsim.GazeboLoadingHelper.rospy.ServiceProxy')
    def test_load_light_sdf(self, mock_proxy, mock_wait_service):
        mock_service_proxy_callee = Mock()
        mock_proxy.return_value = mock_service_proxy_callee
        instance = mock_proxy.return_value
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
        load_light_sdf("light", sdf_xml)
        arg_initial_pose = mock_service_proxy_callee.call_args_list[0][0][3]
        ptn = arg_initial_pose.position
        orn = arg_initial_pose.orientation
        self.assertEquals((ptn.x, ptn.y, ptn.z), (0, 0, 0))
        self.assertEquals((orn.x, orn.y, orn.z, orn.w), (0, 0, 0, 1))
        self.assertEquals(mock_proxy.call_args_list[0][0][0], ROS_S_SPAWN_SDF_LIGHT)
        self.assertEquals(instance.call_args_list[0][0][0], "light")
        self.assertEquals(mock_wait_service.call_args_list[0][0][0], ROS_S_SPAWN_SDF_LIGHT)

        # Testing with given pose
        test_pose = Pose()
        test_pose.position = Point(5, 3, 5)
        test_pose.orientation = Quaternion(1, 2, 3, 4)
        load_light_sdf("light", sdf_xml, test_pose)
        arg_initial_pose = mock_service_proxy_callee.call_args_list[1][0][3]
        self.assertEquals(arg_initial_pose, test_pose)

        # Testing with invalid XML
        self.assertRaises(etree.XMLSyntaxError, load_light_sdf, "light", "invalid XML string")

    @patch('hbp_nrp_cle.robotsim.GazeboLoadingHelper.rospy.wait_for_service')
    @patch('hbp_nrp_cle.robotsim.GazeboLoadingHelper.rospy.ServiceProxy')
    def test_empty_gazebo_world(self, mock_proxy, mock_wait_service):
        empty_gazebo_world()
        self.assertGreater(mock_proxy.call_count, 0)
        self.assertGreater(mock_wait_service.call_count, 0)

    @patch('hbp_nrp_cle.robotsim.GazeboLoadingHelper.rospy.wait_for_service')
    @patch('hbp_nrp_cle.robotsim.GazeboLoadingHelper.rospy.ServiceProxy')
    def test_set_model_pose(self, mock_proxy, mock_wait_service):
      mock_service_proxy_callee = Mock()
      mock_proxy.return_value = mock_service_proxy_callee

      none_pose = Pose()
      none_pose.position = Point(0, 0, 0)
      none_pose.orientation = Quaternion(0, 0, 0, 1)

      set_model_pose('robot', None)

      mock_wait_service.assert_called_with('gazebo/set_model_state', TIMEOUT)
      mock_proxy.assert_called_with('gazebo/set_model_state', SetModelState)

      arg_model_state = mock_service_proxy_callee.call_args_list[0][0][0]
      self.assertEquals(arg_model_state.model_name, 'robot')
      self.assertEquals(arg_model_state.pose, none_pose)

      custom_pose = Pose()
      custom_pose.position = Point(0, 7, 0)
      custom_pose.orientation = Quaternion(0, 0, 0, 1)

      set_model_pose('robot', custom_pose)

      mock_wait_service.assert_called_with('gazebo/set_model_state', TIMEOUT)
      mock_proxy.assert_called_with('gazebo/set_model_state', SetModelState)

      arg_model_state = mock_service_proxy_callee.call_args_list[1][0][0]
      self.assertEquals(arg_model_state.model_name, 'robot')
      self.assertEquals(arg_model_state.pose, custom_pose)

if __name__ == "__main__":
    unittest.main()
