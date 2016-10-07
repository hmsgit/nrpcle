'''setup.py'''

# pylint: disable=F0401,E0611,W0142

try:
    from setuptools import setup
except ImportError:
    from distutils.core import setup

import hbp_nrp_cle
import pip

from pip.req import parse_requirements
from optparse import Option
options = Option("--workaround")
options.skip_requirements_regex = None
reqs_file = './requirements.txt'
# Hack for old pip versions
# Versions greater than 1.x have a required parameter "session" in
# parse_requirements
if pip.__version__.startswith('1.'):
    install_reqs = parse_requirements(reqs_file, options=options)
else:
    from pip.download import PipSession  # pylint:disable=no-name-in-module
    options.isolated_mode = False
    install_reqs = parse_requirements(  # pylint:disable=unexpected-keyword-arg
        reqs_file,
        session=PipSession,
        options=options
    )
reqs = [str(ir.req) for ir in install_reqs]

config = {
    'description': 'Python Implementation of Closed Loop Engine',
    'author': 'HBP Neurorobotics',
    'url': 'http://neurorobotics.net',
    'author_email': 'neurorobotics@humanbrainproject.eu',
    'version': hbp_nrp_cle.__version__,
    # numpy needs to be installed before scipy
    # see http://stackoverflow.com/questions/27021270/how-to-handle-dependency-on-scipy-in-setup-py
    'setup_requires': ['numpy'],
    'install_requires': reqs,
    'packages': ['hbp_nrp_cle',
                 'hbp_nrp_cle.brainsim',
                 'hbp_nrp_cle.brainsim.pynn',
                 'hbp_nrp_cle.brainsim.pynn.devices',
                 'hbp_nrp_cle.brainsim.common',
                 'hbp_nrp_cle.brainsim.common.devices',
                 'hbp_nrp_cle.brainsim.pynn_nest',
                 'hbp_nrp_cle.brainsim.pynn_nest.devices',
                 'hbp_nrp_cle.cle',
                 'hbp_nrp_cle.logs',
                 'hbp_nrp_cle.mocks',
                 'hbp_nrp_cle.mocks.brainsim',
                 'hbp_nrp_cle.mocks.brainsim.__devices',
                 'hbp_nrp_cle.mocks.cle',
                 'hbp_nrp_cle.mocks.robotsim',
                 'hbp_nrp_cle.tf_framework',
                 'hbp_nrp_cle.tf_framework.spike_generators',
                 'hbp_nrp_cle.robotsim'],
    'package_data': {
        'hbp_nrp_cle': ['config.ini']
    },
    'scripts': [],
    'name': 'hbp-nrp-cle',
    'include_package_data': True,
}

setup(**config)
