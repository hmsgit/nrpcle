from hbp_nrp_cle.robotsim.LuganoVizClusterGazebo import LuganoVizClusterGazebo

import unittest
from mock import patch, Mock, MagicMock

__author__ = 'Alessandro Ambrosano'


class TestLuganoVizClusterGazebo(unittest.TestCase):

    def setUp(self):
        self.instance = LuganoVizClusterGazebo()

    @patch('pexpect.spawn')
    def test_spawn_ssh_slurm_frontend(self, mock_spawn):
        mock_spawn().sendline = Mock()

        # Successful call
        mock_spawn().expect = Mock(return_value=0)
        mock_spawn.call_count = 0
        self.instance._LuganoVizClusterGazebo__spawn_ssh_SLURM_frontend()
        self.assertEqual(mock_spawn.call_count, 1)
        self.assertNotEqual(mock_spawn().sendline, 0)

        # Missing password
        mock_spawn().expect = Mock(return_value=1)
        self.assertRaises(Exception, self.instance._LuganoVizClusterGazebo__spawn_ssh_SLURM_frontend)

        # Timeout
        mock_spawn().expect = Mock(return_value=2)
        self.assertRaises(Exception, self.instance._LuganoVizClusterGazebo__spawn_ssh_SLURM_frontend)

    @patch('pexpect.spawn')
    def test_allocate_job(self, mock_spawn):
        self.instance._LuganoVizClusterGazebo__spawn_ssh_SLURM_frontend = mock_spawn
        mock_spawn().sendline = Mock()

        # Successful call
        mock_spawn().expect = Mock(return_value=1)
        mock_spawn().after = 'JobState=RUNNING'
        mock_spawn.call_count = 0
        self.instance._LuganoVizClusterGazebo__allocate_job()
        self.assertEqual(mock_spawn.call_count, 1)
        self.assertNotEqual(mock_spawn().sendline.call_count, 0)

        # Job not running
        mock_spawn().expect = Mock(return_value=1)
        mock_spawn().after = 'JobState=WRONG'
        self.assertRaises(Exception, self.instance._LuganoVizClusterGazebo__allocate_job)

        # Kerberos authentication missing
        mock_spawn().expect = Mock(return_value=2)
        self.assertRaises(Exception, self.instance._LuganoVizClusterGazebo__allocate_job)

        # General error
        mock_spawn().expect = Mock(return_value=3)
        self.assertRaises(Exception, self.instance._LuganoVizClusterGazebo__allocate_job)

        self.instance = LuganoVizClusterGazebo()

    @patch('pexpect.spawn')
    def test_deallocate_job(self, mock_spawn):
        self.instance._LuganoVizClusterGazebo__allocation_process = mock_spawn()
        mock_spawn().sendline = Mock()

        self.instance._LuganoVizClusterGazebo__deallocate_job()
        self.assertTrue(True in ['exit' in x for x in [str(y) for y in mock_spawn().sendline.mock_calls]])
        self.assertEqual('UNDEFINED', self.instance._LuganoVizClusterGazebo__state)
        self.assertIsNone(self.instance._LuganoVizClusterGazebo__allocation_process)

        self.instance = LuganoVizClusterGazebo()

    @patch('pexpect.spawn')
    def test_start_fake_x(self, mock_spawn):
        mock_spawn().expect = Mock(return_value=1)
        mock_spawn.call_count = 0

        self.instance._LuganoVizClusterGazebo__start_fake_X()
        self.assertEqual(mock_spawn.call_count, 1)
        self.assertNotEqual(mock_spawn().expect.call_count, 0)

        mock_spawn().expect = Mock(return_value = 3)
        self.assertRaises(Exception, self.instance._LuganoVizClusterGazebo__start_fake_X)

    @patch('pexpect.spawn')
    def test_spawn_vglconnect(self, mock_spawn):
        # failure: __node is None
        self.instance._LuganoVizClusterGazebo__node = None
        self.assertRaises(Exception, self.instance._LuganoVizClusterGazebo__spawn_vglconnect)

        # failure: __allocation_process is None
        self.instance = LuganoVizClusterGazebo()
        self.instance._LuganoVizClusterGazebo__allocation_process = None
        self.assertRaises(Exception, self.instance._LuganoVizClusterGazebo__spawn_vglconnect)

        self.instance._LuganoVizClusterGazebo__node = 'something that won\'t be used'
        self.instance._LuganoVizClusterGazebo__allocation_process = mock_spawn()

        mock_spawn().sendline = Mock()
        mock_spawn().expect = Mock(return_value=0)
        mock_spawn.call_count = 0

        # successful call
        self.instance._LuganoVizClusterGazebo__spawn_vglconnect()
        self.assertEqual(mock_spawn.call_count, 1)
        self.assertNotEqual(mock_spawn().sendline.call_count, 0)

        # password missing
        mock_spawn().expect = Mock(return_value=1)
        self.assertRaises(Exception, self.instance._LuganoVizClusterGazebo__spawn_vglconnect)

        # generic error
        mock_spawn().expect = Mock(return_value=2)
        self.assertRaises(Exception, self.instance._LuganoVizClusterGazebo__spawn_vglconnect)

        self.instance = LuganoVizClusterGazebo()

    @patch('pexpect.spawn')
    def test_create_remote_working_directory(self, mock_spawn):
        self.instance._LuganoVizClusterGazebo__spawn_vglconnect = mock_spawn

        # failure: __node is None
        self.instance._LuganoVizClusterGazebo__node = None
        self.assertRaises(Exception, self.instance._LuganoVizClusterGazebo__create_remote_working_directory)

        # failure: __allocation_process is None
        self.instance = LuganoVizClusterGazebo()
        self.instance._LuganoVizClusterGazebo__allocation_process = None
        self.assertRaises(Exception, self.instance._LuganoVizClusterGazebo__create_remote_working_directory)

        self.instance._LuganoVizClusterGazebo__node = 'something that won\'t be used'
        self.instance._LuganoVizClusterGazebo__allocation_process = mock_spawn()

        mock_spawn().sendline = Mock()
        mock_spawn().expect = Mock()
        mock_spawn.call_count = 0

        # successful call
        self.instance._LuganoVizClusterGazebo__create_remote_working_directory()
        self.assertEqual(mock_spawn.call_count, 1)
        self.assertNotEqual(mock_spawn().sendline, 0)
        self.assertNotEqual(mock_spawn().expect, 0)

        self.instance = LuganoVizClusterGazebo()

    @patch('pexpect.spawn')
    def test_clean_remote_files(self, mock_spawn):
        self.instance._LuganoVizClusterGazebo__spawn_vglconnect = mock_spawn
        mock_spawn().sendline = Mock()

        # successful call
        self.instance._LuganoVizClusterGazebo__node = 'something that won\'t be used'
        self.instance._LuganoVizClusterGazebo__allocation_process = mock_spawn()
        self.instance._LuganoVizClusterGazebo__remote_working_directory = 'some directory'

        mock_spawn.call_count = 0
        self.instance._LuganoVizClusterGazebo__clean_remote_files()
        self.assertEqual(mock_spawn.call_count, 1)
        self.assertNotEqual(mock_spawn().sendline.call_count, 0)

        self.instance = LuganoVizClusterGazebo()

    @patch('pexpect.spawn')
    def test_start_xvnc(self, mock_spawn):
        self.instance._LuganoVizClusterGazebo__spawn_vglconnect = mock_spawn
        mock_spawn().sendline = Mock()

        # failure: __node is None
        self.instance._LuganoVizClusterGazebo__node = None
        self.assertRaises(Exception, self.instance._LuganoVizClusterGazebo__start_xvnc)

        # failure: __allocation_process is None
        self.instance = LuganoVizClusterGazebo()
        self.instance._LuganoVizClusterGazebo__allocation_process = None
        self.assertRaises(Exception, self.instance._LuganoVizClusterGazebo__start_xvnc)

        self.instance._LuganoVizClusterGazebo__node = 'something that won\'t be used'
        self.instance._LuganoVizClusterGazebo__allocation_process = mock_spawn()

        mock_spawn().sendline = Mock()

        # error creating vnc
        mock_spawn().expect = Mock(return_value=1)
        self.assertRaises(Exception, self.instance._LuganoVizClusterGazebo__start_xvnc)

        # successful call
        mock_spawn().expect = Mock(return_value=0)
        mock_spawn.call_count = 0
        self.instance._LuganoVizClusterGazebo__start_xvnc()
        self.assertEqual(mock_spawn.call_count, 1)
        self.assertNotEqual(mock_spawn().sendline.call_count, 0)
        self.assertNotEqual(mock_spawn().expect.call_count, 0)

        self.instance = LuganoVizClusterGazebo()

    @patch('hbp_nrp_cle.robotsim.LuganoVizClusterGazebo.os')
    def test_sync_models(self, mock_os):
        # failure: __node is None
        self.instance = LuganoVizClusterGazebo()
        self.instance._LuganoVizClusterGazebo__node = None
        self.assertRaises(Exception, self.instance._LuganoVizClusterGazebo__sync_models)

        # failure: __allocation_process is None
        self.instance = LuganoVizClusterGazebo()
        self.instance._LuganoVizClusterGazebo__allocation_process = None
        self.assertRaises(Exception, self.instance._LuganoVizClusterGazebo__sync_models)

        # failure: __remote_working_directory is None
        self.instance = LuganoVizClusterGazebo()
        self.instance._LuganoVizClusterGazebo__remote_working_directory = None
        self.assertRaises(Exception, self.instance._LuganoVizClusterGazebo__sync_models)

        # successful call
        self.instance = LuganoVizClusterGazebo()
        self.instance._LuganoVizClusterGazebo__node = 'not none'
        self.instance._LuganoVizClusterGazebo__allocation_process = 'this neither'
        self.instance._LuganoVizClusterGazebo__remote_working_directory = 'None is not here'
        self.instance._LuganoVizClusterGazebo__sync_models()
        self.assertNotEqual(mock_os.system.call_count, 0)

        self.instance = LuganoVizClusterGazebo()


    @patch('pexpect.spawn')
    def test_start_gazebo(self, mock_spawn):
        # failure: __node is None
        self.instance = LuganoVizClusterGazebo()
        self.instance._LuganoVizClusterGazebo__node = None
        self.assertRaises(Exception, self.instance._LuganoVizClusterGazebo__start_gazebo, 'random_master_uri')

        # failure: __allocation_process is None
        self.instance = LuganoVizClusterGazebo()
        self.instance._LuganoVizClusterGazebo__allocation_process = None
        self.assertRaises(Exception, self.instance._LuganoVizClusterGazebo__start_gazebo, 'random_master_uri')

        # failure: __remote_working_directory is None
        self.instance = LuganoVizClusterGazebo()
        self.instance._LuganoVizClusterGazebo__remote_working_directory = None
        self.assertRaises(Exception, self.instance._LuganoVizClusterGazebo__start_gazebo, 'random_master_uri')

        # failure: __remote_display_port is -1
        self.instance = LuganoVizClusterGazebo()
        self.instance._LuganoVizClusterGazebo__remote_display_port = -1
        self.assertRaises(Exception, self.instance._LuganoVizClusterGazebo__start_gazebo, 'random_master_uri')

        self.instance = LuganoVizClusterGazebo()
        self.instance._LuganoVizClusterGazebo__node = 'not none'
        self.instance._LuganoVizClusterGazebo__allocation_process = 'this neither'
        self.instance._LuganoVizClusterGazebo__remote_working_directory = 'None is not here'
        self.instance._LuganoVizClusterGazebo__remote_display_port = 1
        self.instance._LuganoVizClusterGazebo__spawn_vglconnect = mock_spawn

        mock_spawn().sendline = Mock()
        mock_spawn().expect = Mock(return_value=1)
        self.assertRaises(Exception, self.instance._LuganoVizClusterGazebo__start_gazebo, 'random_master_uri')

        # successful call
        mock_spawn().expect = Mock(return_value=0)
        mock_spawn.call_count = 0

        self.instance._LuganoVizClusterGazebo__start_gazebo('random_master_uri')
        self.assertEqual(mock_spawn.call_count, 1)
        self.assertNotEqual(mock_spawn().sendline.call_count, 0)
        self.assertNotEqual(mock_spawn().expect, 0)

        self.instance = LuganoVizClusterGazebo()

    def test_start(self):
        self.instance._LuganoVizClusterGazebo__allocate_job = Mock()
        self.instance._LuganoVizClusterGazebo__start_fake_X = Mock()
        self.instance._LuganoVizClusterGazebo__create_remote_working_directory = Mock()
        self.instance._LuganoVizClusterGazebo__sync_models = Mock()
        self.instance._LuganoVizClusterGazebo__start_xvnc = Mock()
        self.instance._LuganoVizClusterGazebo__start_gazebo = Mock()

        self.instance.start('random_master_uri')
        self.assertEqual(self.instance._LuganoVizClusterGazebo__allocate_job.call_count, 1)
        self.assertEqual(self.instance._LuganoVizClusterGazebo__start_fake_X.call_count, 1)
        self.assertEqual(self.instance._LuganoVizClusterGazebo__create_remote_working_directory.call_count, 1)
        self.assertEqual(self.instance._LuganoVizClusterGazebo__sync_models.call_count, 1)
        self.assertEqual(self.instance._LuganoVizClusterGazebo__start_xvnc.call_count, 1)
        self.assertEqual(self.instance._LuganoVizClusterGazebo__start_gazebo.call_count, 1)
        self.instance = LuganoVizClusterGazebo()

    def test_stop(self):
        self.instance._LuganoVizClusterGazebo__clean_remote_files = Mock()
        self.instance._LuganoVizClusterGazebo__deallocate_job = Mock()

        self.instance._LuganoVizClusterGazebo__remote_xvnc_process = Mock()
        self.instance._LuganoVizClusterGazebo__gazebo_remote_process = Mock()
        self.instance._LuganoVizClusterGazebo__x_server_process = Mock()

        self.instance.stop()
        self.assertEqual(self.instance._LuganoVizClusterGazebo__clean_remote_files.call_count, 1)
        self.assertEqual(self.instance._LuganoVizClusterGazebo__deallocate_job.call_count, 1)

        self.instance = LuganoVizClusterGazebo()

    def test_restart(self):
        self.instance.start = Mock()
        self.instance.stop = Mock()
        self.instance.restart('')
        self.assertEqual(self.instance.start.call_count, 1)
        self.assertEqual(self.instance.stop.call_count, 1)

if __name__ == '__main__':
    unittest.main()
