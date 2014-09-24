#!/usr/bin/env python2
"""Docstring"""

# pylint: disable=W0401
# pylint: disable=E0611
# pylint: disable=F0401
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup()
d['packages'] = ['gazebo_ros']
d['package_dir'] = {'': 'src'}

setup(**d)
