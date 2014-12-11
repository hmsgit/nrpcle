'''setup.py'''

# pylint: disable=F0401,E0611,W0142

try:
    from setuptools import setup
except ImportError:
    from distutils.core import setup

import hbp_nrp_cle

from pip.req import parse_requirements
install_reqs = parse_requirements('requirements.txt')
from optparse import Option
options = Option("--workaround")
options.skip_requirements_regex = None
install_reqs = parse_requirements("./requirements.txt", options=options)
reqs = [str(ir.req) for ir in install_reqs]

config = {
    'description': 'Python Implementation of Closed Loop Engine',
    'author': 'hinkel',
    'url': 'https://bbpteam.epfl.ch/project/spaces/display/HSP10/Neurorobotics+Platform+Home',
    'author_email': 'bbp-dev-neuroroboticsplatform@groupes.epfl.ch',
    'version': hbp_nrp_cle.__version__,
    'install_requires': reqs,
    'packages': ['hbp_nrp_cle', 'hbp_nrp_cle.brainsim', 'hbp_nrp_cle.brainsim.__devices',
                 'hbp_nrp_cle.cle', 'hbp_nrp_cle.mocks', 'hbp_nrp_cle.mocks.brainsim',
                 'hbp_nrp_cle.mocks.brainsim.__devices', 'hbp_nrp_cle.mocks.cle',
                 'hbp_nrp_cle.mocks.robotsim', 'hbp_nrp_cle.tf_framework',
                 'hbp_nrp_cle.tf_framework.spike_generators', 'hbp_nrp_cle.robotsim'],
    'scripts': [],
    'name': 'hbp-nrp-cle',
    'include_package_data': True,
}

setup(**config)
