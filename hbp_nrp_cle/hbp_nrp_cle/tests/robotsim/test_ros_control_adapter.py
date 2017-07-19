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
from mock import patch, PropertyMock, Mock, MagicMock
from hbp_nrp_cle.robotsim.RosControlAdapter import RosControlAdapter
import unittest
from testfixtures import LogCapture
from gazebo_msgs.srv import GetWorldProperties, GetLightsName

__author__ = 'Lorenzo Vannucci, Sebastian Krach'


class TestRosControlAdapter(unittest.TestCase):

    def select_mock(self, *args, **kwargs):
        mock = {
            '/gazebo/get_physics_properties': self.__get_physics,
            '/gazebo/get_world_properties': self.__get_world,
            '/gazebo/set_physics_properties': self.__set_physics,
            '/gazebo/pause_physics': self.__pause_physics,
            '/gazebo/unpause_physics': self.__unpause_physics,
            '/gazebo/reset_sim': self.__reset_sim,
            '/gazebo/end_world': self.__end_world,
            # These are due to the GazeboHelper
            '/gazebo/spawn_sdf_entity': self.__spawn_sdf_entity,
            '/gazebo/set_model_state': self.__set_model_state,
            '/gazebo/delete_model': self.__delete_model,
            '/gazebo/delete_light': self.__delete_light,
            '/gazebo/delete_lights': self.__delete_lights,
            '/gazebo/get_lights_name': self.__get_lights_name,
            '/gazebo/wait_for_rendering': self.__wait_for_rendering
        }[args[0]]

        for key, value in kwargs.items():
            setattr(mock, key, value)

        return mock

    @patch('hbp_nrp_cle.robotsim.RosControlAdapter.AsynchonousRospyServiceProxy')
    @patch('hbp_nrp_cle.robotsim.RosCommunicationAdapter.rospy.wait_for_service')
    @patch('hbp_nrp_cle.robotsim.RosCommunicationAdapter.rospy.ServiceProxy')
    def setUp(self, rospy_proxy, rospy_wfs, async_proxy_mock):
        self.__advance_simulation = MagicMock()
        self.__get_physics = MagicMock()
        self.__get_world = MagicMock()
        self.__set_physics = MagicMock()
        self.__pause_physics = MagicMock()
        self.__unpause_physics = MagicMock()
        self.__reset_sim = MagicMock()
        self.__end_world = MagicMock()
        # These are due to GazeboHelper
        self.__spawn_sdf_entity = MagicMock()
        self.__set_model_state = MagicMock()
        self.__delete_light = MagicMock()
        self.__delete_model = MagicMock()
        self.__delete_lights = MagicMock()
        self.__get_lights_name = MagicMock()
        self.__wait_for_rendering = MagicMock()

        async_proxy_mock.return_value = self.__advance_simulation

        rospy_proxy.side_effect = self.select_mock

        self._rca = RosControlAdapter()
        self._rca.initialize()

    @patch('hbp_nrp_cle.robotsim.RosControlAdapter.AsynchonousRospyServiceProxy')
    @patch('hbp_nrp_cle.robotsim.RosCommunicationAdapter.rospy.wait_for_service')
    @patch('hbp_nrp_cle.robotsim.RosCommunicationAdapter.rospy.ServiceProxy')
    def test_initialize(self, rospy_proxy, rospy_wfs, async_proxy_mock):

        self.__pause_physics = MagicMock()
        self.__get_physics = MagicMock()
        physics_mock = MagicMock(pause=False, time_step=1.0)
        physics_mock.pause = False
        physics_mock.time_step = 1.0
        self.__get_physics.return_value = physics_mock

        rospy_proxy.side_effect = self.select_mock

        with LogCapture('hbp_nrp_cle.robotsim.RosControlAdapter') as l:
            rca = RosControlAdapter()
            self.assertFalse(self.__pause_physics.called)
            rca.initialize()
            self.assertTrue(self.__pause_physics.called)
            self.assertEqual(rca.time_step, 1.0)

            l.check(('hbp_nrp_cle.robotsim.RosControlAdapter', 'INFO',
                     'Robot control adapter initialized'))

    def test_time_step(self):
        with LogCapture('hbp_nrp_cle.robotsim.RosControlAdapter') as logcapture:
            self.assertEquals(self.__get_physics.call_count, 1)
            self.__set_physics.return_value=True

            self.assertTrue(self._rca.set_time_step(0.01))
            self.assertEquals(self._rca.time_step, 0.01)

            self.__set_physics.return_value=False
            self.assertFalse(self._rca.set_time_step(42))
            self.assertEquals(self._rca.time_step, 0.01)
            logcapture.check(('hbp_nrp_cle.robotsim.RosControlAdapter', 'INFO',
                              'new time step = 0.010000'),
                             ('hbp_nrp_cle.robotsim.RosControlAdapter', 'WARNING',
                              'impossible to set the new time step'))

            self.assertEquals(self.__get_physics.call_count, 3)

    def test_is_paused(self):
        self.__get_physics.return_value.pause = True
        self.assertTrue(self._rca.is_paused)

        self.__get_physics.return_value.pause = False
        self.assertFalse(self._rca.is_paused)

    def test_is_alive(self):
        self.__get_world.return_value.success = True
        self.assertTrue(self._rca.is_alive)

        self.__get_world.return_value.success = False
        self.assertFalse(self._rca.is_alive)

    def test_run_step_logging(self):
        with LogCapture('hbp_nrp_cle.robotsim.RosControlAdapter') as logcapture:
            self.__set_physics.return_value = True
            self._rca.set_time_step(0.01)
            with self.assertRaises(ValueError):
                self._rca.run_step(0.0001)
            logcapture.check(('hbp_nrp_cle.robotsim.RosControlAdapter', 'INFO',
                              'new time step = 0.010000'),
                             ('hbp_nrp_cle.robotsim.RosControlAdapter', 'ERROR',
                              'dt is not multiple of the physics time step'))

    def test_run_step(self):
        future_mock = MagicMock()
        self.__advance_simulation.return_value = future_mock
        future_mock.result.return_value = 100

        self._rca.set_time_step(0.01)

        result = self._rca.run_step(0.01)
        self.__advance_simulation.assert_called_with(1.0)

        self.assertEqual(result, 100)

    def test_run_step_async(self):
        future_mock = MagicMock()
        self.__advance_simulation.return_value = future_mock
        self._rca.set_time_step(0.01)
        result = self._rca.run_step_async(0.01)
        self.assertEqual(result, future_mock)
        self.__advance_simulation.assert_called_with(1.0)

    def test_reset(self):
        with LogCapture('hbp_nrp_cle.robotsim.RosControlAdapter') as logcapture:
            self.__set_physics.return_value=True
            self._rca.set_time_step(0.01)
            self._rca.run_step(0.05)
            self._rca.reset()
            logcapture.check(('hbp_nrp_cle.robotsim.RosControlAdapter', 'INFO',
                              'new time step = 0.010000'),
                             ('hbp_nrp_cle.robotsim.RosControlAdapter', 'DEBUG',
                              'Advancing simulation'),
                             ('hbp_nrp_cle.robotsim.RosControlAdapter', 'INFO',
                              'Resetting the world simulation'))

    def test_reset_world(self):
        rca_module_name = 'hbp_nrp_cle.robotsim.RosControlAdapter'
        user_notifications_module_name = 'hbp_nrp_cle.user_notifications'

        with LogCapture(rca_module_name) as logcapture:
            with LogCapture(user_notifications_module_name) as logcapture_user_notif:
                # mock world (which is loaded during simulation startup)
                mock_sdf = '<sdf/>'
                world_models_sdf = {'model1': {'model_sdf': mock_sdf, 'model_state_sdf': mock_sdf},
                                    'model2': {'model_sdf': mock_sdf, 'model_state_sdf': mock_sdf} }
                world_lights_sdf = {"light1": mock_sdf, "light2": mock_sdf}

                # mock current world status
                mock_world_properties = GetWorldProperties()
                mock_world_properties.model_names = ["model1", "model2", "robot"]

                mock_world_lights = GetLightsName()
                mock_world_lights.light_names = ["light1", "light2", "sun"]

                # mock ros services
                self._rca.gazebo_helper.get_lights_name_proxy = Mock(return_value=mock_world_lights)
                self._rca._RosControlAdapter__get_world_properties = Mock(return_value=mock_world_properties)
                # delete services
                self._rca.gazebo_helper.delete_light_proxy = Mock(return_value=True)
                self._rca.gazebo_helper.delete_model_proxy = Mock(return_value=True)

                # spawn sdf service
                self._rca.gazebo_helper.spawn_sdf_entity_proxy = Mock(return_value=True)

                # call the method under test
                self._rca.reset_world(world_models_sdf, world_lights_sdf)

                # check deletion
                lights_to_delete_and_respawn = ["light1", "light2"]
                models_to_delete_and_respawn = ["model1", "model2"]

                # LIGHTS
                for light in lights_to_delete_and_respawn:
                    self._rca.gazebo_helper.delete_light_proxy.assert_any_call(light)

                # MODELS
                self.assertTrue(False in map(lambda c: 'robot' in c[0],
                                             self._rca.gazebo_helper.delete_model_proxy.call_args_list),
                                'The robot must not be deleted from the scene')

                for model in models_to_delete_and_respawn:
                    self._rca.gazebo_helper.delete_model_proxy.assert_any_call(model)

                # check reLoading
                self.assertFalse(False in map(lambda c: c[0][0] in models_to_delete_and_respawn + lights_to_delete_and_respawn,
                                              self._rca.gazebo_helper.spawn_sdf_entity_proxy.call_args_list),
                                 'Some entity has not been re-spawned')

                # check log
                logcapture.check(
                    (rca_module_name, 'DEBUG', 'Resetting the Environment'),

                    (rca_module_name, 'DEBUG', "active_model_set: set(['model2', 'model1'])"),
                    (rca_module_name, 'DEBUG', "original_model_set: frozenset(['model2', 'model1'])"),

                    (rca_module_name, 'DEBUG', "active_lights_set: set(['light2', 'light1'])"),
                    (rca_module_name, 'DEBUG', "original_lights_set: frozenset(['light2', 'light1'])")
                )

                # check log on user notification logger
                logcapture_user_notif.check(
                    (user_notifications_module_name, 'INFO', "Deleting: light2"),
                    (user_notifications_module_name, 'INFO', "Deleting: light1"),

                    (user_notifications_module_name, 'INFO', "Deleting: model2"),
                    (user_notifications_module_name, 'INFO', "Deleting: model1"),

                    (user_notifications_module_name, 'INFO', "Loading: light2"),
                    (user_notifications_module_name, 'INFO', "Loading: light1"),

                    (user_notifications_module_name, 'INFO', "Loading: model2"),
                    (user_notifications_module_name, 'INFO', "Loading: model1")
                )

    def test_shutdown(self):
        with LogCapture('hbp_nrp_cle.robotsim.RosControlAdapter') as logcapture:
            self._rca.shutdown()
            logcapture.check(('hbp_nrp_cle.robotsim.RosControlAdapter', 'INFO',
                              'Shutting down the world simulation'),
                             ('hbp_nrp_cle.robotsim.RosControlAdapter', 'INFO',
                              'Robot control adapter stopped'))


if __name__ == '__main__':
    unittest.main()
