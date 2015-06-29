"""
This module loads the configuration file for the CLE.
*DO NOT* import config with "from hbp_nrp_cle.config import config",
use "import hbp_nrp_cle.config" and then "hbp_nrp_cle.config.config-" or
"from hbp_nrp_cle import config" and then "config.config" to refer
to config.
In this way, you avoid the creation of multiple ConfigParser objects,
and use only the one created the first time you import the module, in
a singleton fashion.
"""

import ConfigParser
import logging
import os
import netifaces


logger = logging.getLogger(__name__)

config = ConfigParser.ConfigParser()
config.read(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'config.ini'))

logger.info('Checking network interfaces.')
for iface in config.get('network', 'interfaces').split(','):
    logger.info('Trying %s... ', iface)
    # check if iface is available on the system
    if iface in netifaces.interfaces():
        addr = netifaces.ifaddresses(iface)
        # check if iface is connected
        if (netifaces.AF_INET in addr):
            logger.info('OK')
            config.set('network', 'main-interface', iface)
            break
        else:
            logger.info('NOT CONNECTED')
    else:
        logger.info('NOT AVAILABLE')

try:
    config.get('network', 'main-interface')
except ConfigParser.NoOptionError:
    config.set('network', 'main-interface', 'lo')
