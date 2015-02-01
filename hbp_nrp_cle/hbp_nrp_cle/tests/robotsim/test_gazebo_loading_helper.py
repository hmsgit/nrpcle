"""
Tests the Gazebo loading helper class.
"""

import unittest
import os
from geometry_msgs.msg import Point, Pose, Quaternion
from lxml import etree, objectify
from mock import patch, call, MagicMock, Mock
from hbp_nrp_cle.tests.robotsim.ros_test_topics import ROSComTest
from hbp_nrp_cle.robotsim.GazeboLoadingHelper import load_gazebo_sdf, load_gazebo_model_file, load_gazebo_world_file
from testfixtures import log_capture, LogCapture


class TestGazeboLoadingHelper(unittest.TestCase):

    @log_capture('hbp_nrp_cle.robotsim.GazeboLoadingHelper')
    def test_load_gazebo_model_file(self, logcapture):
        with patch("hbp_nrp_cle.robotsim.GazeboLoadingHelper.load_gazebo_sdf") as mocked_load_gazebo_sdf:
            test_pose = Pose()
            test_pose.position = Point(1, 2, 3)
            test_pose.orientation = Quaternion(4, 5, 6, 7)
            load_gazebo_model_file("toto", "../tests/robotsim/sample_model.sdf", test_pose)
            
            self.assertEqual(mocked_load_gazebo_sdf.call_args_list[0][0][0], "toto")

            actualXML = objectify.fromstring(mocked_load_gazebo_sdf.call_args_list[0][0][1])
            actualNormalizedString = etree.tostring(actualXML)
            expectedXML = objectify.fromstring("""<?xml version="1.0" ?>
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
            expectedNormalizedString = etree.tostring(expectedXML)
            self.assertEqual(actualNormalizedString, expectedNormalizedString)

            self.assertEqual(mocked_load_gazebo_sdf.call_args_list[0][0][2], test_pose)
            logcapture.check(('hbp_nrp_cle.robotsim.GazeboLoadingHelper', 'INFO',
                          '../tests/robotsim/sample_model.sdf successfully loaded in Gazebo'))

    @log_capture('hbp_nrp_cle.robotsim.GazeboLoadingHelper')
    def test_load_gazebo_world_file(self, logcapture):
        with patch("hbp_nrp_cle.robotsim.GazeboLoadingHelper.load_gazebo_sdf") as mocked_load_gazebo_sdf:
            load_gazebo_world_file("../tests/robotsim/sample_world.sdf")
        expected_calls_args = [[
                            "sun1",
                            """<?xml version=\"1.0\" ?>\n<sdf version="1.5"><light name="sun1" type="directional">
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
      </attenuation>
    </light></sdf>"""]
                            ,
                          [
                        "sun2",
                        """<?xml version=\"1.0\" ?>\n<sdf version="1.5"><light name="sun2" type="directional">
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
    </light></sdf>"""
                          ],
                          [
                         "ground_plane",
                         """<?xml version=\"1.0\" ?>\n<sdf version="1.5"><model name="ground_plane">
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
      </link>
    </model></sdf>"""
                            ]
                          ]
        self.assertEqual(len(mocked_load_gazebo_sdf.call_args_list), len(expected_calls_args))
        for actual, expected in zip(mocked_load_gazebo_sdf.call_args_list, expected_calls_args):
            self.assertEqual(actual[0][0], expected[0])
            actualXML = objectify.fromstring(actual[0][1])
            actualNormalizedString = etree.tostring(actualXML)
            expectedXML = objectify.fromstring(expected[1])
            expectedNormalizedString = etree.tostring(expectedXML)
            self.assertEqual(actualNormalizedString, expectedNormalizedString)
        logcapture.check(('hbp_nrp_cle.robotsim.GazeboLoadingHelper', 'INFO',
                          'Loading light "sun1" in Gazebo'),
                         ('hbp_nrp_cle.robotsim.GazeboLoadingHelper', 'INFO',
                          'Loading light "sun2" in Gazebo'),
                         ('hbp_nrp_cle.robotsim.GazeboLoadingHelper', 'INFO',
                          'Loading model "ground_plane" in Gazebo'),
                         ('hbp_nrp_cle.robotsim.GazeboLoadingHelper', 'INFO',
                          '../tests/robotsim/sample_world.sdf successfully loaded in Gazebo'))

    def test_load_gazebo_sdf(self):
        with patch("rospy.ServiceProxy") as mocked_service_proxy:
            mocked_load_model_proxy = Mock()
            instance = mocked_service_proxy.return_value
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
</sdf>
                """
            load_gazebo_sdf("toto", sdf_xml)
            self.assertEqual(instance.call_args_list[0][0][0], "toto")

            actualXML = objectify.fromstring(instance.call_args_list[0][0][1])
            actualNormalizedString = etree.tostring(actualXML)
            expectedXML = objectify.fromstring(sdf_xml)
            expectedNormalizedString = etree.tostring(expectedXML)
            self.assertEqual(actualNormalizedString, expectedNormalizedString)

            self.assertEqual(instance.call_args_list[0][0][2], "")

            expected_pose = Pose()
            expected_pose.position = Point(0, 0, 0)
            expected_pose.orientation = Quaternion(0, 0, 0, 1)
            self.assertEqual(instance.call_args_list[0][0][3], expected_pose)

            self.assertEqual(instance.call_args_list[0][0][4], "")

if __name__ == "__main__":
    unittest.main()
