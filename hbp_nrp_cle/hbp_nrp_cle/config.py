"""
This module loads the configuration file for the CLE.
*DO NOT* import conf this with "from hbp_nrp_cle.config import conf",
use "import hbp_nrp_cle.config" and then "hbp_nrp_cle.config.conf" or
"from hbp_nrp_cle import config" and then "config.conf" to refer
to conf.
In this way, you avoid the creation of multiple ConfigParser objects,
and use only the one created the first time you import the module, in
a singleton fashion.
"""

import ConfigParser
import os


conf = ConfigParser.ConfigParser()
conf.read(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'config.ini'))
