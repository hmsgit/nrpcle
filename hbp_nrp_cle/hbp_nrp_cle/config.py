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
