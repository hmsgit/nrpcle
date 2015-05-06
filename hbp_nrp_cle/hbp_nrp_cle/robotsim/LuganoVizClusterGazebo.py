#!/usr/bin/env python
"""
This module contains the classes needed to have all gazebo services running on the Lugano viz
cluster.
"""

import re
import pexpect
import time
import logging
import os
import sys
import netifaces
from hbp_nrp_cle.robotsim.GazeboInterface import IGazeboServerInstance
from random import randint


logger = logging.getLogger('hbp_nrp_cle')


# This will be the method called by the pexpect object to log.
# pexpect is expecting kwargs unused argument.
def _log_write(*args, **kwargs):  # pylint: disable=unused-argument
    """
    Translation between pexpect log to file mechanism and python logging module
    """
    content = args[0]
    # let's ignore other params, pexpect only use one arg AFAIK
    if content in [' ', '', '\n', '\r', '\r\n']:
        return  # don't log empty lines
    for eol in ['\r\n', '\r', '\n']:
        # remove ending EOL, the logger will add it anyway
        content = re.sub(r'\%s$' % eol, '', content)
    return logger.info(content)  # call the logger info method with the reworked content


def _set_up_logger():
    """
    Configure the root logger of the application
    :param: logfile_name: name of the file created to collect logs
    """
    # give the logger the methods required by pexpect
    logger.write = _log_write
    logger.flush = lambda: None

_set_up_logger()


class LuganoVizClusterGazebo(IGazeboServerInstance):
    """
    Represents an instance of gzserver running on the Lugano viz cluster.
    """

    CLUSTER_SLURM_FRONTEND = 'ssh -K bbpsoatest@bbpviz1.cscs.ch'
    # SLURM salloc calls allocates a node on the cluster. From salloc man page:
    #
    # salloc - Obtain a SLURM job allocation (a set of nodes), execute a command,and then release
    # the allocation when the command is finished.
    # SYNOPSIS
    # salloc [options] [<command> [command args]]
    #
    # -c, --cpus-per-task=<ncpus>
    # Advise the SLURM controller that ensuing job steps will require ncpusnumber of processors
    # per task. Without this option, the controller willjust try to allocate one processor per
    # task.
    # For instance,consider an application that has 4 tasks, each requiring 3 processors. If
    # ourcluster is comprised of quad-processors nodes and we simply ask for12 processors, the
    # controller might give us only 3 nodes. However, by usingthe --cpus-per-task=3 options, the
    # controller knows that each task requires3 processors on the same node, and the controller
    # will grant an allocationof 4 nodes, one for each of the 4 tasks.
    #
    # -I, --immediate[=<seconds>]
    # exit if resources are not available within thetime period specified.If no argument is given,
    # resources must be available immediatelyfor the request to succeed.By default, --immediate is
    # off, and the commandwill block until resources become available. Since this option'sargument
    # is optional, for proper parsing the single letter option mustbe followed immediately with the
    # value and not include a space betweenthem. For example "-I60" and not "-I 60".
    #
    # --gres=<list>
    # Specifies a comma delimited list of generic consumable resources.The format of each entry on
    # the list is "name[:count[*cpu]]".The name is that of the consumable resource.The count is the
    # number of those resources with a default value of 1.The specified resources will be allocated
    # to the job on each nodeallocated unless "*cpu" is appended, in which case the resourceswill
    # be allocated on a per cpu basis.The available generic consumable resources is configurable
    # by the systemadministrator.A list of available generic consumable resources will be printed
    # and thecommand will exit if the option argument is "help".Examples of use
    # include "--gres=gpus:2*cpu,disk=40G" and "--gres=help".
    #
    # -t, --time=<time>
    # Set a limit on the total run time of the job allocation. If therequested time limit exceeds
    # the partition's time limit, the job willbe left in a PENDING state (possibly indefinitely).
    # The default timelimit is the partition's default time limit. When the time limit is reached,
    # each task in each job step is sent SIGTERM followed by SIGKILL. Theinterval between signals
    # is specified by the SLURM configurationparameter KillWait. A time limit of zero requests
    # that no timelimit be imposed. Acceptable time formats include "minutes","minutes:seconds",
    # "hours:minutes:seconds", "days-hours","days-hours:minutes" and "days-hours:minutes:seconds".
    #
    # -p, --partition=<partition_names>
    # Request a specific partition for the resource allocation. If not specified,the default
    # behavior is to allow the slurm controller to select the defaultpartition as designated by
    # the system administrator. If the job can use morethan one partition, specify their names
    # in a comma separate list and the oneoffering earliest initiation will be used.
    #
    # -A, --account=<account>
    # Charge resources used by this job to specified account.The account is an arbitrary string.
    # The account name maybe changed after job submission using the scontrolcommand.
    ALLOCATION_COMMAND = ("salloc --immediate=25 --time=10:00:00 -p interactive"
                          " -c 4 --account=proj16 --gres=gpu:1")
    DEALLOCATION_COMMAND = 'scancel %s'
    KINIT_COMMAND = 'kinit bbpsoatest@INTRANET.EPFL.CH -k -t /etc/krb5.keytab.d/bbpsoatest.keytab'
    NODE_DOMAIN = '.cscs.ch'
    # Timeout used for pexpect ssh connection calls.
    TIMEOUT = 10
    # Timeout used for pexpect calls that should return immediately (default pexpect timeout is 30
    # seconds).
    SMALL_TIMEOUT = 2
    DEFAULT_GZSERVER_PORT = 11345

    def __init__(self, notification_fn=lambda x, y: ()):
        """
        :param notification_fn: A function used to notify the current status.
        (default: lambda x: ())
        """
        self.__allocation_process = None
        self.__x_server_process = None
        self.__remote_xvnc_process = None
        self.__gazebo_remote_process = None
        self.__remote_working_directory = None
        self.__remote_display_port = -1
        self.__job_ID = None
        # Holds the state of the SLURM job. The states are defined in SLURM.
        self.__state = "UNDEFINED"
        self.__node = None
        self.__notification_fn = notification_fn

    def __spawn_ssh_SLURM_frontend(self):
        """
        Return a pexpect object connected to the SLURM frontend.
        SLURM (Simple Linux Utility for Resource Management) is the entry point to
        allocate or manage jobs on the cluster.
        """
        ssh_SLURM_frontend_process = pexpect.spawn('bash',
                                                   logfile=logger)
        ssh_SLURM_frontend_process.sendline(self.KINIT_COMMAND)
        ssh_SLURM_frontend_process.sendline(self.CLUSTER_SLURM_FRONTEND)
        result = ssh_SLURM_frontend_process.expect(['[bbpsoatest@bbpviz1 ~]$',
                                                    'password',
                                                    pexpect.TIMEOUT], self.TIMEOUT)
        if result == 1:
            raise(Exception("SLURM front-end node can't be used without password."))
        if result == 2:
            raise(Exception("Cannot connect to the SLURM front-end node."))

        logger.info("Connected to the SLURM front-end node.")
        self.__notification_fn("Connected to the SLURM front-end node.", False)
        return ssh_SLURM_frontend_process

    def __allocate_job(self):
        """
        Allocate a new job on the cluster. Return a pexpect object with a ssh
        connection to the allocated node. If once call exit on this object, the
        allocation will be cancelled.
        """
        self.__allocation_process = self.__spawn_ssh_SLURM_frontend()
        self.__allocation_process.sendline(self.ALLOCATION_COMMAND)
        result = self.__allocation_process.expect(['Granted job allocation ([0-9]+)',
                                                   'Submitted batch job [0-9]+',
                                                   'error: spank-auks: cred forwarding failed',
                                                   'error: .+'])
        if result == 2:
            raise(Exception("Kerberos authentication missing"))
        elif result == 3:
            raise(Exception("Job allocation failed: " + self.__allocation_process.after))

        # Find out which node has been allocated
        self.__job_ID = self.__allocation_process.match.groups()[0]
        self.__allocation_process.sendline("scontrol show job " + str(self.__job_ID))
        self.__allocation_process.expect('JobState=[A-Z]+')
        self.__state = self.__allocation_process.after[9:]
        if self.__state != 'RUNNING':
            raise Exception("Job is not running.")
        self.__allocation_process.expect(r' NodeList=(\w+)')
        self.__node = self.__allocation_process.match.groups()[0]

    def __deallocate_job(self):
        """
        Deallocate a job previously allocated through __allocate_job. The idea is
        that exiting from the terminal opened through the allocation causes SLURM
        to complete the Job.
        """
        if self.__allocation_process is not None:
            self.__allocation_process.sendline('exit')
            self.__allocation_process = None
            self.__job_ID = None
            self.__state = "UNDEFINED"
            self.__node = None

    def __start_fake_X(self):
        """
        Start an in memory graphical server. Xvfb or X virtual framebuffer is a display server
        implementing the X11 display server protocol. In contrast to other display servers Xvfb
        performs all graphical operations in memory without showing any screen output. The goal
        is to be able to use vglconnect from the local node to the remote viz cluster node. For
        that, we do need an XServer.
        """
        self.__x_server_process = pexpect.spawn('Xvfb :1', logfile=logger)
        result = self.__x_server_process.expect(['Server is already active for display',
                                                 'Initializing built-in extension',
                                                 pexpect.TIMEOUT])
        if result == 2:
            raise(Exception("Cannot start Xvfb"))

    def __spawn_vglconnect(self):
        """
        Return a pexpect object connected to the allocated viz cluster node.
        """
        if self.__node is None or self.__allocation_process is None:
            raise(Exception("Cannot connect to a cluster node without a proper Job allocation."))

        vglconnect_process = pexpect.spawn('bash',
                                           env={"DISPLAY": ":1"},
                                           logfile=logger)
        vglconnect_process.sendline(self.KINIT_COMMAND)
        vglconnect_process.sendline(('vglconnect bbpsoatest@' +
                                     self.__node +
                                     self.NODE_DOMAIN))
        # We do expect a prompt here
        result = vglconnect_process.expect([r'\[bbpsoatest@' + self.__node + r'\ ~\]\$',
                                            'password',
                                            pexpect.TIMEOUT])
        if result == 1:
            raise(Exception("Viz cluster node can't be used without password."))
        if result == 2:
            raise(Exception("Cannot connect to node."))

        return vglconnect_process  # This object has to live until the end.

    def __create_remote_working_directory(self):
        """
        Create a temporary working directory on the allocated viz cluster node.
        """
        if self.__node is None or self.__allocation_process is None:
            raise(Exception("Cannot connect to a cluster node without a proper Job allocation."))
        create_temporary_folder_process = self.__spawn_vglconnect()
        create_temporary_folder_process.sendline('mktemp -d')
        create_temporary_folder_process.expect(r'\/tmp\/[a-zA-Z0-9\.]+')
        self.__remote_working_directory = create_temporary_folder_process.after

    def __clean_remote_working_directory(self):
        """
        Remove the temporary remote working directory
        """
        if (self.__node is not None and
                self.__allocation_process is not None and
                self.__remote_working_directory is not None):
            sync_models_process = self.__spawn_vglconnect()
            sync_models_process.sendline('rm -rf ' + self.__remote_working_directory)

    def __start_xvnc(self):
        """
        Start a remote Xvnc server. This is the only (known to us) way to have Gazebo using
        the graphic card.
        """
        if self.__node is None or self.__allocation_process is None:
            raise(Exception("Cannot connect to a cluster node without a proper Job allocation."))

        self.__remote_xvnc_process = self.__spawn_vglconnect()
        self.__remote_display_port = randint(10, 100)
        self.__remote_xvnc_process.sendline('Xvnc :' + str(self.__remote_display_port))
        result = self.__remote_xvnc_process.expect(['created VNC server for screen 0',
                                                    pexpect.TIMEOUT], self.TIMEOUT)
        if result == 1:
            self.__remote_display_port = -1
            raise(Exception("Cannot start Xvnc"))

    def __sync_models(self):
        """
        Copy the local models (assumed to be in $HOME/.gazebo/models) to the remote
        viz cluster node
        """
        if self.__node is None or self.__allocation_process is None:
            raise(Exception("Cannot connect to a cluster node without a proper Job allocation."))
        if self.__remote_working_directory is None:
            raise(Exception("Syncing the models cannot work without a remote working directory"))
        os.system(
            "scp -r $HOME/.gazebo/models bbpsoatest@" +
            self.__node + self.NODE_DOMAIN +
            ":" +
            self.__remote_working_directory)

    # TODO check outputs !
    def __install_ros_dependencies(self):
        """
        Install all the needed ROS dependencies on the allocated viz cluster node
        (in the remote working directory).
        """
        if self.__node is None or self.__allocation_process is None:
            raise(Exception("Cannot connect to a cluster node without a proper Job allocation."))
        if self.__remote_working_directory is None:
            raise(Exception("Installing ROS dependencies cannot work without" +
                            " a remote working directory"))
        install_ros_dependencies_process = self.__spawn_vglconnect()
        install_ros_dependencies_process.sendline('cd ' + self.__remote_working_directory)
        install_ros_dependencies_process.sendline('source /opt/rh/python27/enable')
        install_ros_dependencies_process.sendline('virtualenv ros')
        install_ros_dependencies_process.sendline('source ros/bin/activate')
        install_ros_dependencies_process.sendline('pip install rospkg')
        install_ros_dependencies_process.sendline('pip install catkin_pkg')
        install_ros_dependencies_process.sendline('pip install netifaces')
        install_ros_dependencies_process.sendline('pip install PIL')

    def __start_gazebo(self, ros_master_uri):
        """
        Start gazebo on the remote server
        """
        if self.__node is None or self.__allocation_process is None:
            raise(Exception("Cannot connect to a cluster node without a proper Job allocation."))
        if self.__remote_display_port == -1:
            raise(Exception("Gazebo needs a remote X Server running"))
        if self.__remote_working_directory is None:
            raise(Exception("Gazebo needs a remote working directory"))

        self.__gazebo_remote_process = self.__spawn_vglconnect()

        self.__gazebo_remote_process.sendline('source /opt/rh/python27/enable')
        self.__gazebo_remote_process.sendline('export DISPLAY=:' + str(self.__remote_display_port))
        self.__gazebo_remote_process.sendline('export ROS_MASTER_URI=' + ros_master_uri)
        self.__gazebo_remote_process.sendline(
            'source /nfs4/bbp.epfl.ch/sw/neurorobotics/ros_gazebo_venv/bbpviz.cscs.ch/bin/activate')
        self.__gazebo_remote_process.sendline(
            'export GAZEBO_MODEL_PATH=' + self.__remote_working_directory + '/models')

        # loading the environment modules configuration files
        self.__gazebo_remote_process.sendline(
            'export MODULEPATH=$MODULEPATH:/nfs4/bbp.epfl.ch/sw/neurorobotics/modulefiles')
        self.__gazebo_remote_process.sendline(
            'export MODULEPATH=$MODULEPATH:/nfs4/bbp.epfl.ch/sw/module/modulefiles')

        # source environment modules init file
        self.__gazebo_remote_process.sendline('source /usr/share/Modules/init/bash 2> /dev/null')

        # load the modules
        self.__gazebo_remote_process.sendline('module load ros/hydro-rhel6-x86_64-gcc4.4')
        self.__gazebo_remote_process.sendline('module load gazebo/4.0-rhel6-x86_64-gcc4.8.2')
        self.__gazebo_remote_process.sendline('module load sdf/2.0-rhel6-x86_64-gcc4.4')
        self.__gazebo_remote_process.sendline('module load ogre/1.8.1-rhel6-x86_64-gcc4.8.2')
        self.__gazebo_remote_process.sendline('module load boost/1.55zlib-rhel6-x86_64-gcc4.4')
        self.__gazebo_remote_process.sendline('module load opencv/2.4.9-rhel6-x86_64-gcc4.8.2')
        self.__gazebo_remote_process.sendline('module load tbb/4.0.5-rhel6-x86_64-gcc4.4')
        self.__gazebo_remote_process.sendline(
            'module load ros-hbp-packages/hydro-rhel6-x86_64-gcc4.4')
        self.__gazebo_remote_process.sendline(
            'module load ros-thirdparty/hydro-rhel6-x86_64-gcc4.4')

        # source ROS, Gazebo and our plugins
        self.__gazebo_remote_process.sendline('source $ROS_SETUP_FILE')
        self.__gazebo_remote_process.sendline('source $GAZEBO_RESOURCE_PATH/setup.sh')
        self.__gazebo_remote_process.sendline('source $ROS_THIRDPARTY_PACKAGES_SETUP_FILE')
        self.__gazebo_remote_process.sendline('source $ROS_HBP_PACKAGES_SETUP_FILE')
        ros_plugins_folder = ("/nfs4/bbp.epfl.ch/sw/neurorobotics/ros-hbp-packages/hydro/"
                              "rhel-6.5-x86_64/gcc-4.4.7/x86_64/lib/")
        self.__gazebo_remote_process.sendline(
            'vglrun $GAZEBO_BIN_DIR/gzserver ' +
            '-s ' + ros_plugins_folder + 'libgazebo_ros_api_plugin.so ' +
            '-s ' + ros_plugins_folder + 'libgazebo_ros_paths_plugin.so ' +
            '--verbose')

        result = self.__gazebo_remote_process.expect(['Gazebo multi-robot simulator',
                                                      pexpect.TIMEOUT])

        if result == 1:
            raise(Exception("Error while starting gazebo" + self.__gazebo_remote_process.after))

    def start(self, ros_master_uri):
        """
        Start gzserver on the Lugano viz cluster
        """

        self.__notification_fn("Starting gzserver", False)
        logger.info('Allocating one job on the vizualization cluster')
        self.__notification_fn('Allocating one job on the vizualization cluster', False)
        self.__allocate_job()
        logger.info('Start an XServer without attached screen')
        self.__notification_fn('Start an XServer without attached screen', False)
        self.__start_fake_X()
        logger.info('Sync models on the remote node')
        self.__notification_fn('Sync models on the remote node', False)
        self.__create_remote_working_directory()
        self.__sync_models()
        logger.info('Install ROS python dependencies')
        self.__notification_fn('Install ROS python dependencies', False)
        self.__install_ros_dependencies()
        logger.info('Start Xvnc on the remote node')
        self.__notification_fn('Start Xvnc on the remote node', False)
        self.__start_xvnc()
        logger.info('Start gzserver on the remote node')
        self.__notification_fn('Start gzserver on the remote node', False)
        self.__start_gazebo(ros_master_uri)

    @property
    def gazebo_master_uri(self):
        """
        Returns a string containing the gzserver master
        URI (like:'http://bbpviz001.cscs.ch:11345')
        """
        if self.__node is not None:
            return ('http://' +
                    self.__node +
                    self.NODE_DOMAIN +
                    ':' +
                    str(self.DEFAULT_GZSERVER_PORT))
        else:
            return None

    def stop(self):
        self.__notification_fn("Stopping gzserver", False)
        self.__clean_remote_working_directory()
        self.__deallocate_job()

    def restart(self, ros_master_uri):
        self.__notification_fn("Restarting gzserver", False)
        self.stop()
        self.start(ros_master_uri)


def _get_roscore_master_uri():
    """
    Return roscore master URI. If the env variable ROS_MASTER_URI is not set,
    then construct it like this: http:// + local_ip + :11311
    """

    master_uri = os.environ.get("ROS_MASTER_URI")
    if not master_uri:
        local_ip = netifaces.ifaddresses('eth0')[netifaces.AF_INET][0]['addr']
        master_uri = 'http://' + local_ip + ':11311'
    return master_uri

# Useful to test out the code.
if __name__ == '__main__':
    logger.setLevel(logging.DEBUG)
    log_format = '%(asctime)s [%(threadName)-12.12s] [%(name)-12.12s] [%(levelname)s]  %(message)s'
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setFormatter(logging.Formatter(log_format))
    logger.setLevel(logging.DEBUG)
    logger.addHandler(console_handler)
    gazebo = LuganoVizClusterGazebo()
    gazebo.start(_get_roscore_master_uri())
    time.sleep(100)
    gazebo.stop()
