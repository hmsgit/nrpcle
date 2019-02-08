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
'''
NengoBrainLoader.py
moduleauthor: krach@fzi.de
'''
# pylint: disable=unused-import
from hbp_nrp_cle.brainsim.common.PythonBrainLoader import load_py_network
import logging

logger = logging.getLogger("NengoBrainLoader")


def setup_access_to_population(brain_module, *populations): # pragma: no cover
    """
    Sets up the access to the population

    :param brain_module: The brain module
    :param populations: A dictionary of the populations and their ids
    """
    try:
        for p in populations:
            brain_module.__dict__[p.name] = p.population
    except AttributeError:
        if len(populations) > 0:
            raise Exception(
                "Could not initialize populations, no circuit found")
