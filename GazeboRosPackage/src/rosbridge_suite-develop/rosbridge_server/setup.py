#!/usr/bin/env python

#pylint: skip-file

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['backports', 'backports.ssl_match_hostname',
              'tornado', 'tornado.platform'],
    package_dir={'': 'src'}
)

setup(**d)
