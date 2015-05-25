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
import os


config = ConfigParser.ConfigParser()
config.read(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'config.ini'))
