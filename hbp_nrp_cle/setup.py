'''setup.py'''

try:
    from setuptools import setup
except ImportError:
    from distutils.core import setup # pylint:disable=no-name-in-module, import-error

import hbp_nrp_cle
import pip
from optparse import Option
options = Option("--workaround")
options.skip_requirements_regex = None
reqs_file = './requirements.txt'

# Hack for old pip versions
if pip.__version__.startswith('10.'):
    # Versions greater or equal to 10.x don't rely on pip.req.parse_requirements
    install_reqs = list(val.strip() for val in open(reqs_file))
    reqs = install_reqs
elif pip.__version__.startswith('1.'):
    # Versions 1.x rely on pip.req.parse_requirements
    # but don't require a "session" parameter
    from pip.req import parse_requirements # pylint:disable=no-name-in-module, import-error
    install_reqs = parse_requirements(reqs_file, options=options)
    reqs = [str(ir.req) for ir in install_reqs]
else:
    # Versions greater than 1.x but smaller than 10.x rely on pip.req.parse_requirements
    # and requires a "session" parameter
    from pip.req import parse_requirements # pylint:disable=no-name-in-module, import-error
    from pip.download import PipSession  # pylint:disable=no-name-in-module, import-error
    options.isolated_mode = False
    install_reqs = parse_requirements(  # pylint:disable=unexpected-keyword-arg
        reqs_file,
        session=PipSession,
        options=options
    )
    reqs = [str(ir.req) for ir in install_reqs]

# workaround to avoid compilation of multiple numpy versions - see NRRPLT-4130
cython_req = next(r for r in reqs if r.startswith('cython'))
numpy_req = next(r for r in reqs if r.startswith('numpy'))
if pip.__version__.startswith('10.'):
    import subprocess
    subprocess.check_call(
        ["python", '-m', 'pip', 'install', "--no-clean", "--user", cython_req, numpy_req]
    )
else:
    pip.main(['install', '--no-clean', cython_req, numpy_req]) # pylint:disable=no-member

config = {
    'description': 'Python Implementation of Closed Loop Engine',
    'author': 'HBP Neurorobotics',
    'url': 'http://neurorobotics.net',
    'author_email': 'neurorobotics@humanbrainproject.eu',
    'version': hbp_nrp_cle.__version__,
    'install_requires': reqs,
    'packages': ['hbp_nrp_cle',
                 'hbp_nrp_cle.brainsim',
                 'hbp_nrp_cle.brainsim.pynn',
                 'hbp_nrp_cle.brainsim.pynn.devices',
                 'hbp_nrp_cle.brainsim.common',
                 'hbp_nrp_cle.brainsim.common.devices',
                 'hbp_nrp_cle.brainsim.pynn_nest',
                 'hbp_nrp_cle.brainsim.pynn_nest.devices',
                 'hbp_nrp_cle.brainsim.pynn_spiNNaker',
                 'hbp_nrp_cle.brainsim.pynn_spiNNaker.devices',
                 'hbp_nrp_cle.brainsim.nengo',
                 'hbp_nrp_cle.brainsim.nengo.devices',
                 'hbp_nrp_cle.cle',
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
